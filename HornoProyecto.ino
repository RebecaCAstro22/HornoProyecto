#include <Arduino.h>
#include <Wire.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
#else
  #include <WiFi.h>
  #include <WebServer.h>
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <Preferences.h>
#include <ESPmDNS.h>

// =======================================
// HORNO PROYECTO 4.0 GG - CONTROL DEFINITIVO
// =======================================

// PINES DEL ESP32
const int PIN_LM35  = 34; // Sensor LM35 (ADC1_CH6)
const int PIN_TRIAC = 23; // Salida al optoacoplador del TRIAC (Secadora)
const int PIN_SDA   = 21; // OLED SDA
const int PIN_SCL   = 22; // OLED SCL

// OLED
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledDisponible = false;

// WIFI AP
const char* ssid     = "Horno4_0";
const char* password = "Horno2026";

// WIFI DE LA CASA (se guarda desde http://192.168.4.1/wifi, no va en el codigo)
// El ESP32 queda en las DOS redes: la de la casa (http://horno.local) y la suya propia
String wifiCasaSsid = "";
String wifiCasaPass = "";
bool wifiCasaConectado = false;

#if defined(ESP8266)
ESP8266WebServer server(80);
#else
WebServer server(80);
#endif

// VARIABLES DE CONTROL
float objetivo = 40.0;            // Temperatura deseada
float tolerancia = 2.0;           // Tolerancia (+/- °C)
String nivelPotencia = "high";    // "low", "med", "high"
const float ALPHA = 0.15;         // Filtro para suavizar lectura

float temperatura = 25.0;         // Temperatura actual
bool sensorValido = true;
bool calefaccion = false;

// MODOS
enum Modo {
  APAGADO,
  MANUAL,
  AUTOMATICO
};
Modo modo = AUTOMATICO;
bool manualEncendido = false;

// FASE DEL MODO AUTOMATICO
// NORMAL : lejos del objetivo -> TRIAC continuo (FULL) para subir rapido
// RAFAGA : cerca del objetivo -> pulsos ON/OFF para mantener
// ESPERA : TRIAC apagado
String faseAuto = "ESPERA";
const unsigned long RAFAGA_PERIODO_MS = 2000;  // ventana de la rafaga
const unsigned long RAFAGA_ON_MS      = 1000;  // tiempo encendido dentro de la ventana (50%)

// TIPO DE CONTROL DENTRO DEL MODO AUTOMATICO (/set?ctl=rafaga|pid|autotune)
// RAFAGA_FIJA : el de siempre -> FULL lejos, rafaga fija 50% cerca, apagado al llegar
// PID         : FULL lejos (T < objetivo - bandaPID); cerca, el PID decide cuanto
//               dura el encendido de cada rafaga (0..100% de RAFAGA_PERIODO_MS)
// AUTOTUNE    : prende/apaga alrededor del objetivo, mide como oscila el horno
//               y calcula Kp, Ki, Kd solo. Al terminar pasa a PID.
enum TipoControl { RAFAGA_FIJA, PID_CTRL, AUTOTUNE };
TipoControl tipoControl = RAFAGA_FIJA;

// GANANCIAS DEL PID (salida = fraccion 0..1 de la ventana; error en °C)
// Valores INICIALES; lo ideal es correr el autoajuste una vez. Se guardan en flash.
float Kp = 0.20;     // 1 °C abajo -> +20 % de encendido
float Ki = 0.010;    // por °C·s acumulado
float Kd = 1.00;     // por °C/s de subida (frena antes de pasarse)
float bandaPID = 5.0;               // °C debajo del objetivo donde entra el PID (fuera = FULL)
const float TAU_DERIVADA = 2.0;     // s, filtro de la derivada (el LM35 tiene ruido)

float pidIntegral = 0.0;
float pidDuty = 0.0;                // ultima salida del PID (0..1)
float pidDerivada = 0.0;            // °C/s filtrada
float tempAnterior = NAN;
unsigned long inicioVentana = 0;
unsigned long onMsVentana = 0;      // ms encendido en la rafaga en curso
bool pidActivo = false;             // true = dentro de la banda, rafagas del PID

// CALIBRACION DEL SENSOR (comparar contra un termometro de referencia)
float offsetCal = 0.0;              // °C que se suman a la lectura del LM35

// AUTOAJUSTE (metodo del rele de Astrom-Hagglund + reglas de Tyreus-Luyben)
const float AT_HIST = 0.3;                       // °C de histeresis del rele
const int   AT_CICLOS_DESCARTE = 1;              // el primer ciclo es transitorio
const int   AT_CICLOS_MEDIR = 3;
const unsigned long AT_TIMEOUT_MS = 30UL * 60UL * 1000UL;
bool atSalida = true;
int atCiclo = 0;                    // ciclos completos medidos (contando el descartado)
bool atArrancado = false;           // ya hubo el primer cruce hacia arriba
unsigned long atInicio = 0, atUltimoCorte = 0;
float atMax = -1000, atMin = 1000, atSumaAmp = 0, atSumaPer = 0;
String atEstado = "";               // mensaje para el dashboard

// PRUEBA COMPLETA AUTOMATICA (ver actualizarPrueba)
enum EtapaPrueba { P_INACTIVA, P_ENFRIA1, P_RAFAGA, P_AUTOTUNE, P_ENFRIA2, P_PID, P_LISTA, P_CANCELADA };
EtapaPrueba etapa = P_INACTIVA;
bool pruebaEnfriando = false;           // fuerza la secadora apagada mientras enfria
unsigned long pruebaMinutos = 15;       // duracion de la prueba de rafagas y de la de PID
const unsigned long ENFRIA_MAX_MS = 30UL * 60UL * 1000UL;
const float ENFRIA_MARGEN = 2.0;        // enfria hasta ambiente-inicial + 2 °C
float pruebaTbase = 25.0, pruebaObjetivo = 40.0;
unsigned long etapaInicio = 0;
String pruebaNota = "";
const int LOG_MAX = 5400;               // 3 h a una muestra cada 2 s
const unsigned long LOG_CADA_MS = 2000;
int16_t logT[LOG_MAX];                  // temperatura x10
uint8_t logInfo[LOG_MAX];               // etapa (bits 0-3) + secadora (bit 4)
int logN = 0;
unsigned long ultimoLog = 0;
int idxIni[8], idxFin[8];

Preferences prefs;

// DIAGNOSTICO DEL SENSOR (detecta el LM35 desconectado)
float ultimaLecturaMv = 0;        // mV promedio de la ultima lectura
float ultimoRuidoMv = 0;          // diferencia max-min entre muestras (mV)
int lecturasMalas = 0;            // lecturas malas seguidas
int lecturasBuenas = 0;           // lecturas buenas seguidas
const int LECTURAS_PARA_FALLA = 5;      // 5 x 100 ms = 0.5 s para declarar desconectado
const int LECTURAS_PARA_RECUPERAR = 10; // 1 s estable para volver a confiar
const float RUIDO_MAX_MV = 150.0f;      // el LM35 da una señal estable; un pin al aire salta mucho
// Con el LM35 desconectado el ADC se queda abajo (pocos mV). Una lectura real en
// interiores siempre esta arriba de esto, asi que lo de abajo se toma como desconectado.
const float TEMP_MIN_VALIDA = 16.0f;

// SEGURIDAD POR SOBRECALENTAMIENTO REAL
// >= 65 °C reales: secadora APAGADA en cualquier modo (incluso manual) hasta bajar de 60 °C
const float PROTECCION_ON  = 65.0f;
const float PROTECCION_OFF = 60.0f;
bool proteccionActiva = false;

// DEMO DE SOBRECALENTAMIENTO (para la presentacion)
// Reporta 65 °C durante 6 s para que salgan las alertas. No calienta de verdad:
// mientras dura, la secadora queda APAGADA a la fuerza.
const float DEMO_TEMP = 65.0f;
const unsigned long DEMO_MS = 6000;
unsigned long demoHasta = 0;
bool demoActiva() { return demoHasta != 0 && (long)(demoHasta - millis()) > 0; }
void iniciarDemo() {
  demoHasta = millis() + DEMO_MS;
  if (demoHasta == 0) demoHasta = 1;
  Serial.println("[DEMO] Sobrecalentamiento simulado: 65 C por 6 s (secadora apagada)");
}

// TEMPORIZADORES
unsigned long ultimaLectura = 0;
const unsigned long INTERVALO = 100;

unsigned long ultimoRefrescoOLED = 0;
const unsigned long INTERVALO_OLED = 500;

// =======================================
// SALIDA AL TRIAC
// =======================================
void actualizarSalida(bool encender) {
  calefaccion = encender;
  digitalWrite(PIN_TRIAC, encender ? HIGH : LOW);
}

// =======================================
// GUARDAR / CARGAR AJUSTES EN FLASH
// =======================================
void guardarAjustes() {
  prefs.putFloat("kp", Kp);
  prefs.putFloat("ki", Ki);
  prefs.putFloat("kd", Kd);
  prefs.putFloat("banda", bandaPID);
  prefs.putFloat("cal", offsetCal);
  prefs.putUChar("ctl", tipoControl == AUTOTUNE ? (uint8_t)PID_CTRL : (uint8_t)tipoControl);
}

void cargarAjustes() {
  prefs.begin("horno", false);
  Kp = prefs.getFloat("kp", Kp);
  Ki = prefs.getFloat("ki", Ki);
  Kd = prefs.getFloat("kd", Kd);
  bandaPID = prefs.getFloat("banda", bandaPID);
  offsetCal = prefs.getFloat("cal", offsetCal);
  wifiCasaSsid = prefs.getString("wssid", "");
  wifiCasaPass = prefs.getString("wpass", "");
  uint8_t c = prefs.getUChar("ctl", (uint8_t)RAFAGA_FIJA);
  tipoControl = (c == PID_CTRL) ? PID_CTRL : RAFAGA_FIJA;
}

// =======================================
// LECTURA DIRECTA Y CALIBRADA DEL LM35
// =======================================
// Atenuacion 0 dB: rango ~100-950 mV (10-95 °C) -> mas resolucion y mejor
// linealidad que 11 dB. analogReadMilliVolts usa la calibracion de fabrica (eFuse).
float leerLM35() {
  uint32_t sumaMv = 0;
  uint32_t mvMin = 100000, mvMax = 0;
  const int MUESTRAS = 64;

  for (int i = 0; i < MUESTRAS; i++) {
    uint32_t m = analogReadMilliVolts(PIN_LM35);
    sumaMv += m;
    if (m < mvMin) mvMin = m;
    if (m > mvMax) mvMax = m;
    delayMicroseconds(50);
  }

  float mv = (float)sumaMv / MUESTRAS;
  ultimaLecturaMv = mv;
  ultimoRuidoMv = (float)(mvMax - mvMin);

  // 10 mV por cada grado Celsius + ajuste contra termometro de referencia
  return mv / 10.0f + offsetCal;
}

void medirTemperatura() {
  float cruda = leerLM35();

  // Lectura buena: dentro de rango fisico y sin ruido de pin al aire
  bool lecturaOK = isfinite(cruda) && cruda >= TEMP_MIN_VALIDA && cruda <= 120.0f
                   && ultimoRuidoMv <= RUIDO_MAX_MV;

  if (lecturaOK) {
    lecturasMalas = 0;
    if (!sensorValido) {
      lecturasBuenas++;
      if (lecturasBuenas >= LECTURAS_PARA_RECUPERAR) {
        sensorValido = true;
        lecturasBuenas = 0;
        temperatura = cruda; // reinicia el filtro con el valor real
        tempAnterior = NAN;
        Serial.println("[SENSOR] LM35 reconectado");
      }
    } else {
      temperatura = (ALPHA * cruda) + ((1.0f - ALPHA) * temperatura);
    }
  } else {
    lecturasBuenas = 0;
    if (sensorValido) {
      lecturasMalas++;
      if (lecturasMalas >= LECTURAS_PARA_FALLA) {
        sensorValido = false;
        Serial.printf("[ALERTA] LM35 desconectado (%.0f mV, ruido %.0f mV)\n",
                      ultimaLecturaMv, ultimoRuidoMv);
      }
    }
  }

  // Seguridad por temperatura REAL (la demo no cuenta)
  if (sensorValido && !proteccionActiva && temperatura >= PROTECCION_ON) {
    proteccionActiva = true;
    Serial.printf("[ALERTA] Sobrecalentamiento real: %.1f C -> secadora apagada\n", temperatura);
  } else if (proteccionActiva && temperatura < PROTECCION_OFF) {
    proteccionActiva = false;
    Serial.println("[SEGURIDAD] Temperatura normal otra vez");
  }
}

// =======================================
// NOMBRE DEL MODO
// =======================================
String nombreModo() {
  if (modo == MANUAL) return "MANUAL";
  if (modo == AUTOMATICO) return "AUTOMATICO";
  return "APAGADO";
}

String nombreControl() {
  if (tipoControl == PID_CTRL) return "PID";
  if (tipoControl == AUTOTUNE) return "AUTOTUNE";
  return "RAFAGA";
}

// =======================================
// RAFAGA DEL PID CON PRECISION DE MILISEGUNDOS
// (se llama en cada vuelta del loop, no solo cada 100 ms)
// =======================================
void aplicarVentanaPID() {
  if (modo != AUTOMATICO || tipoControl != PID_CTRL || !pidActivo || pruebaEnfriando) return;
  if (demoActiva() || proteccionActiva || !sensorValido) return;   // las seguridades mandan

  unsigned long ahora = millis();
  if (ahora - inicioVentana >= RAFAGA_PERIODO_MS) {
    // Nueva ventana: congela el duty actual del PID
    inicioVentana = ahora;
    unsigned long onMs = (unsigned long)(pidDuty * RAFAGA_PERIODO_MS + 0.5f);
    if (onMs < 50) onMs = 0;                                      // pulso inutil (< 3 ciclos de red)
    if (onMs > RAFAGA_PERIODO_MS - 50) onMs = RAFAGA_PERIODO_MS;  // casi lleno -> lleno
    onMsVentana = onMs;
  }
  faseAuto = (onMsVentana == 0) ? "ESPERA" : "RAFAGA";
  actualizarSalida((ahora - inicioVentana) < onMsVentana);
}

// =======================================
// AUTOAJUSTE: TERMINAR Y CALCULAR GANANCIAS
// =======================================
void terminarAutoajuste() {
  int n = atCiclo - AT_CICLOS_DESCARTE;
  float a = atSumaAmp / n;                       // amplitud media (°C)
  float pu = atSumaPer / n;                      // periodo medio (s)
  float aEf = sqrtf(fmaxf(a * a - AT_HIST * AT_HIST, 0.01f * a * a));
  float ku = (4.0f * 0.5f) / (PI * aEf);         // rele de 0..1 -> amplitud d = 0.5

  // Tyreus-Luyben: poco sobrepaso, robusto para procesos termicos con retardo
  float kp = ku / 2.2f;
  float ti = 2.2f * pu;
  float td = pu / 6.3f;
  Kp = constrain(kp, 0.0f, 5.0f);
  Ki = constrain(kp / ti, 0.0f, 1.0f);
  Kd = constrain(kp * td, 0.0f, 50.0f);

  atEstado = "Listo: Ku=" + String(ku, 3) + " Pu=" + String(pu, 1) + "s amp=" + String(a, 2) + "C";
  Serial.printf("[AUTOAJUSTE] Ku=%.3f Pu=%.1fs amp=%.2fC -> Kp=%.3f Ki=%.4f Kd=%.3f\n",
                ku, pu, a, Kp, Ki, Kd);

  tipoControl = PID_CTRL;
  pidActivo = false;
  tempAnterior = NAN;
  guardarAjustes();
}

void iniciarAutoajuste() {
  tipoControl = AUTOTUNE;
  atSalida = true;
  atCiclo = 0;
  atArrancado = false;
  atInicio = millis();
  atUltimoCorte = 0;
  atMax = -1000; atMin = 1000;
  atSumaAmp = 0; atSumaPer = 0;
  atEstado = "Subiendo al objetivo...";
}

// =======================================
// PRUEBA COMPLETA AUTOMATICA (3 EN 1)
// Enfria -> Prueba 1 rafagas fijas -> Prueba 2 autoajuste -> enfria -> Prueba 3 PID
// Guarda una muestra cada 2 s y al final compara rafagas vs PID.
// =======================================
const char* NOMBRE_ETAPA[] = {
  "Inactiva", "Enfriando antes de empezar", "Prueba 1/3: rafagas fijas",
  "Prueba 2/3: autoajuste PID", "Enfriando antes del PID", "Prueba 3/3: PID",
  "Terminada", "Cancelada"
};
const char* CORTO_ETAPA[] = { "", "ENFRIANDO", "1/3 RAFAGAS", "2/3 AUTOAJ", "ENFRIANDO", "3/3 PID", "PRUEBA LISTA", "CANCELADA" };

bool pruebaCorriendo() {
  return etapa != P_INACTIVA && etapa != P_LISTA && etapa != P_CANCELADA;
}

void cambiarEtapa(EtapaPrueba e) {
  if (pruebaCorriendo()) idxFin[etapa] = logN;
  etapa = e;
  etapaInicio = millis();
  idxIni[e] = logN;
  idxFin[e] = logN;
  pruebaEnfriando = (e == P_ENFRIA1 || e == P_ENFRIA2);

  if (e == P_RAFAGA) {
    tipoControl = RAFAGA_FIJA;
  } else if (e == P_AUTOTUNE) {
    iniciarAutoajuste();
  } else if (e == P_PID) {
    tipoControl = PID_CTRL;
    pidActivo = false;
    tempAnterior = NAN;
  } else if (e == P_LISTA) {
    // Termino: deja el horno APAGADO por seguridad (nadie esta mirando)
    modo = APAGADO;
    actualizarSalida(false);
    guardarAjustes();
  }
  Serial.printf("[PRUEBA] -> %s (T=%.2f C)\n", NOMBRE_ETAPA[e], temperatura);
}

void iniciarPrueba(unsigned long minutos) {
  logN = 0;
  ultimoLog = 0;
  for (int i = 0; i < 8; i++) { idxIni[i] = -1; idxFin[i] = -1; }
  pruebaMinutos = minutos;
  pruebaTbase = temperatura;
  pruebaObjetivo = objetivo;
  pruebaNota = "";
  modo = AUTOMATICO;
  manualEncendido = false;
  cambiarEtapa(P_ENFRIA1);
}

void cancelarPrueba(const char* motivo) {
  if (!pruebaCorriendo()) return;
  idxFin[etapa] = logN;
  Serial.printf("[PRUEBA] Cancelada: %s\n", motivo);
  pruebaNota += String("Cancelada: ") + motivo + ". ";
  etapa = P_CANCELADA;
  pruebaEnfriando = false;
  if (tipoControl == AUTOTUNE) { tipoControl = PID_CTRL; pidActivo = false; }
}

void actualizarPrueba() {
  if (!pruebaCorriendo()) return;
  unsigned long ahora = millis();
  unsigned long enEtapa = ahora - etapaInicio;

  // Registro cada 2 s
  if ((ultimoLog == 0 || ahora - ultimoLog >= LOG_CADA_MS) && logN < LOG_MAX) {
    ultimoLog = ahora;
    logT[logN] = (int16_t)lroundf(temperatura * 10.0f);
    logInfo[logN] = (uint8_t)etapa | (calefaccion ? 0x10 : 0);
    logN++;
  }

  if (modo != AUTOMATICO) { cancelarPrueba("se cambio el modo"); return; }

  switch (etapa) {
    case P_ENFRIA1:
    case P_ENFRIA2:
      if (temperatura <= pruebaTbase + ENFRIA_MARGEN || enEtapa > ENFRIA_MAX_MS) {
        if (enEtapa > ENFRIA_MAX_MS) {
          pruebaNota += String("No enfrio del todo antes de ") +
                        (etapa == P_ENFRIA1 ? "las rafagas" : "el PID") +
                        " (quedo en " + String(temperatura, 1) + " C). ";
        }
        cambiarEtapa(etapa == P_ENFRIA1 ? P_RAFAGA : P_PID);
      }
      break;
    case P_RAFAGA:
      // El autoajuste no necesita arrancar en frio: sigue directo
      if (enEtapa >= pruebaMinutos * 60000UL) cambiarEtapa(P_AUTOTUNE);
      break;
    case P_AUTOTUNE:
      if (tipoControl != AUTOTUNE) {
        if (atEstado.startsWith("Cancelado")) {
          pruebaNota += "El autoajuste no termino; el PID uso las ganancias anteriores. ";
        }
        cambiarEtapa(P_ENFRIA2);
      }
      break;
    case P_PID:
      if (enEtapa >= pruebaMinutos * 60000UL) cambiarEtapa(P_LISTA);
      break;
    default:
      break;
  }
  if (logN >= LOG_MAX && pruebaCorriendo()) {
    pruebaNota += "Se lleno la memoria del registro. ";
  }
}

// Metricas de una prueba (rafagas o PID) a partir del registro
String metricasJSON(EtapaPrueba e) {
  int a = idxIni[e];
  int b = (etapa == e) ? logN : idxFin[e];
  if (a < 0 || b - a < 15) return "{\"ok\":false}";

  float obj = pruebaObjetivo;
  float tLleg = -1, mx = -1000;
  for (int i = a; i < b; i++) {
    float T = logT[i] / 10.0f;
    if (tLleg < 0 && T >= obj - 0.5f) tLleg = (i - a) * (LOG_CADA_MS / 1000.0f);
    if (T > mx) mx = T;
  }
  // Regimen: los ultimos 5 min (o la segunda mitad si la prueba fue mas corta)
  int n = (b - a) / 2;
  if (n > 150) n = 150;
  float sAbs = 0, s = 0, s2 = 0, mn = 1000, mxr = -1000;
  for (int i = b - n; i < b; i++) {
    float err = logT[i] / 10.0f - obj;
    sAbs += fabsf(err); s += err; s2 += err * err;
    if (err < mn) mn = err;
    if (err > mxr) mxr = err;
  }
  String j = "{\"ok\":true";
  j += ",\"llegada\":" + String(tLleg, 0);
  j += ",\"pico\":" + String(mx - obj, 2);
  j += ",\"errMedio\":" + String(sAbs / n, 2);
  j += ",\"rms\":" + String(sqrtf(s2 / n), 2);
  j += ",\"sesgo\":" + String(s / n, 2);
  j += ",\"banda\":" + String(mxr - mn, 2);
  j += ",\"regimenMin\":" + String(n * (LOG_CADA_MS / 1000.0f) / 60.0f, 1);
  j += "}";
  return j;
}

const char PRUEBA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="es"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Prueba completa del horno</title>
<style>
  body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;background:#0c1527;color:#f8fafc;margin:0;padding:18px}
  .c{max-width:900px;margin:0 auto;display:flex;flex-direction:column;gap:14px}
  .box{background:#132238;border:1px solid #24395a;border-radius:12px;padding:14px}
  h1{font-size:1.3rem;margin:0} h2{font-size:1rem;margin:0 0 8px}
  button,a.b{background:#2563eb;color:#fff;border:0;border-radius:8px;padding:9px 14px;font-weight:700;cursor:pointer;text-decoration:none;font-size:.9rem;display:inline-block}
  button.g{background:#475569}
  input{width:60px;padding:7px;border-radius:6px;border:1px solid #475569;background:#0c1527;color:#f8fafc}
  table{width:100%;border-collapse:collapse;font-size:.9rem} td,th{padding:7px;border-bottom:1px solid #24395a;text-align:center}
  th:first-child,td:first-child{text-align:left}
  .win{color:#34d399;font-weight:800} .m{opacity:.75;font-size:.85rem}
  .bar{height:8px;background:#24395a;border-radius:4px;overflow:hidden} .bar>div{height:100%;background:#38bdf8;width:0}
  svg{width:100%;height:auto;background:#0c1527;border-radius:8px}
</style></head><body><div class="c">
<h1>Prueba completa: rafagas vs PID</h1>
<div class="box">
  <div id="etapa" style="font-size:1.05rem;font-weight:800">...</div>
  <div class="m" id="detalle"></div>
  <div class="bar" style="margin:10px 0"><div id="prog"></div></div>
  <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
    Minutos por prueba <input id="min" type="number" value="15" min="5" max="40">
    <button onclick="iniciar()">Iniciar prueba completa</button>
    <button class="g" onclick="cancelar()">Cancelar</button>
    <a class="b" href="/prueba.csv">Descargar CSV</a>
    <a class="b" style="background:#475569" href="/">Volver al panel</a>
  </div>
  <p class="m">Usa el objetivo que este puesto en el panel. Empieza con el horno frio y la tapa cerrada, sonda en su lugar.
  No cambies el modo ni el objetivo mientras corre (eso cancela la prueba). Al terminar el horno queda APAGADO.</p>
  <div class="m" id="nota" style="color:#fbbf24"></div>
</div>
<div class="box"><h2>Resultados</h2>
  <table><thead><tr><th>Medida</th><th>Rafagas fijas</th><th>PID autoajustado</th></tr></thead><tbody id="tabla"></tbody></table>
  <p class="m" id="at"></p>
</div>
<div class="box"><h2>Temperatura desde que arranca cada prueba</h2>
  <svg id="graf" viewBox="0 0 860 320"></svg>
  <div class="m"><span style="color:#fb923c">&#9632;</span> Rafagas fijas &nbsp; <span style="color:#38bdf8">&#9632;</span> PID &nbsp; <span style="color:#94a3b8">- -</span> objetivo</div>
</div>
</div>
<script>
const FILAS = [
  ['llegada','Tiempo hasta llegar (obj - 0.5 &deg;C)','s',0],
  ['pico','Sobrepaso maximo','&deg;C',2],
  ['errMedio','Error medio en regimen','&deg;C',2],
  ['rms','Error RMS en regimen','&deg;C',2],
  ['banda','Oscilacion (max - min) en regimen','&deg;C',2],
  ['sesgo','Sesgo (+ arriba / - abajo)','&deg;C',2]
];
let datos = null;
async function iniciar(){
  const m = document.getElementById('min').value;
  if(!confirm('Va a correr las 3 pruebas seguidas (aprox. 1 a 2 horas). El horno debe estar frio y con la tapa cerrada. Empezar?')) return;
  await fetch('/prueba/iniciar?min='+m); leer();
}
async function cancelar(){ await fetch('/prueba/cancelar'); leer(); }
function fmt(v,d){ return (v===undefined||v===null||v<0&&d===0) ? '--' : Number(v).toFixed(d); }
async function leer(){
  try{ datos = await (await fetch('/prueba.json')).json(); }catch(e){ return; }
  document.getElementById('etapa').textContent = datos.nombre;
  const min = Math.floor(datos.seg/60), s = datos.seg%60;
  document.getElementById('detalle').textContent =
    `En esta etapa: ${min} min ${s} s - Ahora ${datos.temp.toFixed(2)} C - Objetivo ${datos.objetivo.toFixed(1)} C - Ambiente al iniciar ${datos.tbase.toFixed(1)} C` +
    (datos.etapa===3 ? ' - ' + datos.at : '');
  const pasos = {0:0,1:5,2:25,3:50,4:65,5:80,6:100,7:0};
  document.getElementById('prog').style.width = pasos[datos.etapa] + '%';
  document.getElementById('nota').textContent = datos.nota;
  const r = datos.rafaga, p = datos.pid;
  let h = '';
  for (const [k,txt,u,d] of FILAS){
    let cr='', cp='';
    if (r.ok && p.ok){
      const a = k==='sesgo'||k==='pico' ? Math.abs(r[k]) : r[k], b = k==='sesgo'||k==='pico' ? Math.abs(p[k]) : p[k];
      if (k==='llegada'){ if(r[k]>=0&&p[k]>=0){ if(a<b) cr='win'; else if(b<a) cp='win'; } }
      else { if(a<b) cr='win'; else if(b<a) cp='win'; }
    }
    h += `<tr><td>${txt}</td><td class="${cr}">${r.ok?fmt(r[k],d)+' '+u:'--'}</td><td class="${cp}">${p.ok?fmt(p[k],d)+' '+u:'--'}</td></tr>`;
  }
  document.getElementById('tabla').innerHTML = h;
  document.getElementById('at').textContent = `Ganancias PID: Kp ${datos.kp} - Ki ${datos.ki} - Kd ${datos.kd}` + (datos.at ? ' - Autoajuste: ' + datos.at : '') +
    (r.ok&&p.ok ? ` - Regimen = ultimos ${p.regimenMin} min de cada prueba. Verde = mejor.` : '');
}
async function grafica(){
  let txt; try{ txt = await (await fetch('/prueba.csv')).text(); }catch(e){ return; }
  const filas = txt.trim().split('\n').slice(1).map(l=>l.split(','));
  const series = {2:[],5:[]};
  for (const f of filas){ const e = +f[2]; if(series[e]) series[e].push(+f[3]); }
  const obj = datos ? datos.objetivo : 40;
  const todas = series[2].concat(series[5], [obj]);
  if (todas.length < 2) return;
  let lo = Math.floor(Math.min(...todas)) - 1, hi = Math.ceil(Math.max(...todas)) + 1;
  const n = Math.max(series[2].length, series[5].length, 2);
  const W=860,H=320,L=44,R=10,T=10,B=28;
  const x = i => L + (W-L-R) * i/(n-1), y = v => T + (H-T-B) * (hi-v)/(hi-lo);
  let s = '';
  for (let v=lo; v<=hi; v+=Math.max(1,Math.round((hi-lo)/8))) s += `<line x1="${L}" x2="${W-R}" y1="${y(v)}" y2="${y(v)}" stroke="#24395a"/><text x="${L-6}" y="${y(v)+4}" fill="#94a3b8" font-size="11" text-anchor="end">${v}</text>`;
  const totalMin = (n-1)*2/60;
  for (let m=0; m<=totalMin; m+=Math.max(1,Math.round(totalMin/10))) s += `<text x="${x(m*30)}" y="${H-8}" fill="#94a3b8" font-size="11" text-anchor="middle">${m} min</text>`;
  s += `<line x1="${L}" x2="${W-R}" y1="${y(obj)}" y2="${y(obj)}" stroke="#94a3b8" stroke-dasharray="6 5"/>`;
  const linea = (arr,col) => arr.length>1 ? `<polyline fill="none" stroke="${col}" stroke-width="2" points="${arr.map((v,i)=>x(i).toFixed(1)+','+y(v).toFixed(1)).join(' ')}"/>` : '';
  s += linea(series[2],'#fb923c') + linea(series[5],'#38bdf8');
  document.getElementById('graf').innerHTML = s;
}
leer().then(grafica); setInterval(leer, 3000); setInterval(grafica, 15000);
</script></body></html>
)rawliteral";

// =======================================
// ALGORITMO DE CONTROL TERMICO (AUTONOMO)
// =======================================
void actualizarControl() {
  // 0. SEGURIDADES (antes que cualquier modo)
  if (demoActiva()) {                       // demo: nunca calienta
    faseAuto = "DEMO";
    pidDuty = 0.0f;
    actualizarSalida(false);
    return;
  }
  if (proteccionActiva) {                   // >= 65 °C reales
    faseAuto = "PROTECCION";
    pidDuty = 0.0f;
    actualizarSalida(false);
    return;
  }
  if (!sensorValido && modo == AUTOMATICO) { // sin sensor no sabe la temperatura
    faseAuto = "SIN_SENSOR";
    pidDuty = 0.0f;
    actualizarSalida(false);
    return;
  }

  // 1. MODO APAGADO
  if (modo == APAGADO) {
    actualizarSalida(false);
    return;
  }

  // 2. MODO MANUAL (Pruebas del usuario)
  if (modo == MANUAL) {
    actualizarSalida(manualEncendido);
    return;
  }

  // Prueba completa: entre pruebas la secadora queda apagada para enfriar
  if (pruebaEnfriando) {
    faseAuto = "ESPERA";
    pidDuty = 0.0f;
    actualizarSalida(false);
    return;
  }

  // 3a. AUTOAJUSTE: rele ON/OFF alrededor del objetivo, mide amplitud y periodo
  if (tipoControl == AUTOTUNE) {
    unsigned long ahora = millis();
    if (ahora - atInicio > AT_TIMEOUT_MS) {
      atEstado = "Cancelado: 30 min sin terminar (revisa que llegue al objetivo)";
      tipoControl = PID_CTRL;
      pidActivo = false;
      return;
    }

    atMax = fmaxf(atMax, temperatura);
    atMin = fminf(atMin, temperatura);

    if (atSalida && temperatura > objetivo + AT_HIST) {
      // Corte ON -> OFF: aqui empieza/termina cada ciclo
      atSalida = false;
      if (!atArrancado) {
        atArrancado = true;                       // primer cruce: solo marca el inicio
      } else {
        atCiclo++;
        if (atCiclo > AT_CICLOS_DESCARTE) {
          atSumaAmp += (atMax - atMin) / 2.0f;
          atSumaPer += (ahora - atUltimoCorte) / 1000.0f;
        }
      }
      atUltimoCorte = ahora;
      atMax = temperatura; atMin = temperatura;

      if (atCiclo >= AT_CICLOS_DESCARTE + AT_CICLOS_MEDIR) {
        terminarAutoajuste();
        return;
      }
      if (atArrancado) {
        atEstado = "Midiendo ciclo " + String(atCiclo + 1) + " de " +
                   String(AT_CICLOS_DESCARTE + AT_CICLOS_MEDIR);
      }
    }
    else if (!atSalida && temperatura < objetivo - AT_HIST) {
      atSalida = true;
    }

    faseAuto = atSalida ? "NORMAL" : "ESPERA";
    pidDuty = atSalida ? 1.0f : 0.0f;
    actualizarSalida(atSalida);
    return;
  }

  // 3b. MODO AUTOMATICO CON PID: FULL lejos, rafagas de ancho variable cerca
  if (tipoControl == PID_CTRL) {
    // dt real (a veces /set o /modo llaman aqui fuera del tick de 100 ms)
    static unsigned long ultimoPID = 0;
    unsigned long tPID = millis();
    float dt = (tPID - ultimoPID) / 1000.0f;
    if (dt < 0.05f) return;           // llamada demasiado seguida: no recalcula
    if (dt > 1.0f) dt = INTERVALO / 1000.0f;
    ultimoPID = tPID;

    float error = objetivo - temperatura;

    // Derivada sobre la medicion, filtrada (sin filtro el ruido del ADC la vuelve loca)
    if (isfinite(tempAnterior)) {
      float dCruda = (temperatura - tempAnterior) / dt;
      float b = dt / (TAU_DERIVADA + dt);
      pidDerivada += b * (dCruda - pidDerivada);
    } else {
      pidDerivada = 0.0f;
    }
    tempAnterior = temperatura;

    if (error > bandaPID) {
      // Lejos del objetivo -> FULL, igual que siempre. El PID se queda en reposo.
      faseAuto = "NORMAL";
      pidActivo = false;
      pidIntegral = 0.0f;
      pidDuty = 1.0f;
      actualizarSalida(true);
      return;
    }

    if (!pidActivo) {
      // Recien entra a la banda: ventana nueva en el siguiente aplicarVentanaPID()
      pidActivo = true;
      pidIntegral = 0.0f;
      inicioVentana = millis() - RAFAGA_PERIODO_MS;
    }

    // Integral con anti-windup: solo acumula si la salida no esta saturada
    // en la misma direccion del error
    float salidaSinI = Kp * error - Kd * pidDerivada;
    float prueba = salidaSinI + Ki * (pidIntegral + error * dt);
    if ((prueba < 1.0f || error < 0) && (prueba > 0.0f || error > 0)) {
      pidIntegral += error * dt;
    }
    pidDuty = constrain(salidaSinI + Ki * pidIntegral, 0.0f, 1.0f);

    aplicarVentanaPID();
    return;
  }

  // 3c. MODO AUTOMATICO (rafaga fija): potencia NORMAL para subir, RAFAGA para mantener
  {
    float minRango = objetivo - tolerancia;

    if (temperatura >= objetivo) {
      // Alcanzo el objetivo -> APAGA la secadora
      faseAuto = "ESPERA";
      actualizarSalida(false);
    }
    else if (temperatura < minRango) {
      // Lejos del objetivo -> potencia normal (continua)
      faseAuto = "NORMAL";
      actualizarSalida(true);
    }
    else {
      // Cerca del objetivo -> rafaga: encendido RAFAGA_ON_MS de cada RAFAGA_PERIODO_MS
      faseAuto = "RAFAGA";
      actualizarSalida((millis() % RAFAGA_PERIODO_MS) < RAFAGA_ON_MS);
    }
  }
}

// =======================================
// PANTALLA OLED
// =======================================
void actualizarOLED() {
  if (!oledDisponible) return;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  // Arriba: la IP en el Wi-Fi de la casa (si esta conectado) para saber a donde entrar
  if (wifiCasaConectado) oled.println(WiFi.localIP().toString());
  else                   oled.println("HORNO 4.0 GG");
  oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Lectura Actual
  oled.setCursor(0, 15);
  oled.print("ACT: ");
  oled.setTextSize(2);
  if (demoActiva()) {
    oled.setCursor(42, 13);
    oled.print(DEMO_TEMP, 1);
    oled.setTextSize(1);
    oled.println(" C");
  } else if (sensorValido) {
    oled.setCursor(42, 13);
    oled.print(temperatura, 1);
    oled.setTextSize(1);
    oled.println(" C");
  } else {
    oled.setTextSize(1);
    oled.setCursor(42, 15);
    oled.println("SIN SENSOR!");
  }

  // Objetivo y Modo
  oled.setTextSize(1);
  oled.setCursor(0, 31);
  oled.print("OBJ: ");
  oled.print(objetivo, 1);
  oled.print(" C [");
  oled.print(nombreModo());
  oled.println("]");

  // Salida TRIAC
  oled.setCursor(0, 47);
  oled.print("SECADORA: ");
  oled.print(calefaccion ? "ON" : "OFF");
  if (modo == AUTOMATICO) {
    oled.print(" (");
    oled.print(faseAuto);
    oled.print(")");
    if (tipoControl == PID_CTRL) {
      oled.setCursor(0, 56);
      oled.print("PID ");
      oled.print((int)(pidDuty * 100));
      oled.print("%");
    }
    if (etapa != P_INACTIVA) {
      // Durante/despues de la prueba completa la ultima linea muestra la etapa
      oled.fillRect(0, 56, 128, 8, SSD1306_BLACK);
      oled.setCursor(0, 56);
      oled.print(CORTO_ETAPA[etapa]);
      if (pruebaCorriendo()) {
        oled.print(" ");
        oled.print((millis() - etapaInicio) / 60000UL);
        oled.print("m");
      }
    } else if (tipoControl == AUTOTUNE) {
      oled.setCursor(0, 56);
      oled.print("AUTOAJUSTE ");
      oled.print(atArrancado ? atCiclo + 1 : 0);
      oled.print("/");
      oled.print(AT_CICLOS_DESCARTE + AT_CICLOS_MEDIR);
    }
  }

  // Avisos de seguridad en la ultima linea (tienen prioridad)
  if (demoActiva() || proteccionActiva) {
    oled.fillRect(0, 56, 128, 8, SSD1306_BLACK);
    oled.setCursor(0, 56);
    oled.print(demoActiva() ? "DEMO SOBRECALENTAM." : "!! SOBRECALENTADO !!");
  }

  oled.display();
}
// =======================================
// ESTADO EN JSON (el mismo para USB y para Wi-Fi)
// =======================================
String estadoJSON() {
  bool demo = demoActiva();
  String json = "{";
  json += "\"valido\":" + String(sensorValido ? "true" : "false");
  json += ",\"sensor\":\"" + String(sensorValido ? "OK" : "DESCONECTADO") + "\"";
  json += ",\"mv\":" + String(ultimaLecturaMv, 0);
  // Durante la demo de sobrecalentamiento se reporta 65 °C (solo para mostrar la alerta)
  json += ",\"temperatura\":" + (demo ? String(DEMO_TEMP, 1) : (sensorValido ? String(temperatura, 1) : String("null")));
  json += ",\"objetivo\":" + String(objetivo, 1);
  json += ",\"tolerancia\":" + String(tolerancia, 1);
  json += ",\"potencia\":\"" + nivelPotencia + "\"";
  json += ",\"calefaccion\":" + String(calefaccion ? "true" : "false");
  json += ",\"modo\":\"" + nombreModo() + "\"";
  json += ",\"manualEncendido\":" + String(manualEncendido ? "true" : "false");
  json += ",\"fase\":\"" + String(modo == AUTOMATICO ? faseAuto : "") + "\"";
  json += ",\"control\":\"" + nombreControl() + "\"";
  json += ",\"duty\":" + String(pidDuty * 100.0f, 0);
  json += ",\"kp\":" + String(Kp, 3);
  json += ",\"ki\":" + String(Ki, 4);
  json += ",\"kd\":" + String(Kd, 3);
  json += ",\"banda\":" + String(bandaPID, 1);
  json += ",\"cal\":" + String(offsetCal, 2);
  json += ",\"temp2\":" + String(demo ? DEMO_TEMP : temperatura, 2);
  json += ",\"at\":\"" + atEstado + "\"";
  json += ",\"prueba\":\"" + String(etapa == P_INACTIVA ? "" : NOMBRE_ETAPA[etapa]) + "\"";
  json += ",\"demo\":" + String(demo ? "true" : "false");
  json += ",\"demoSeg\":" + String(demo ? (demoHasta - millis() + 999) / 1000 : 0UL);
  json += ",\"proteccion\":" + String(proteccionActiva ? "true" : "false");
  json += ",\"red\":\"" + String(wifiCasaConectado ? WiFi.SSID() : "") + "\"";
  json += ",\"ip\":\"" + String(wifiCasaConectado ? WiFi.localIP().toString() : "") + "\"";
  json += "}";
  return json;
}

// =======================================
// COMANDOS (iguales por USB y por Wi-Fi)
// =======================================
// Por USB: texto terminado en salto de linea (tambien desde el Monitor Serial).
// Por Wi-Fi: /cmd?c=<comando>, o las rutas de siempre /modo y /set.
//   MODO auto | MODO off | MODO manual_on | MODO manual_off
//   SET t=40 tol=2 pot=high ctl=rafaga|pid|autotune kp= ki= kd= banda= cal= ref=
//   PRUEBA iniciar 15 | PRUEBA cancelar
//   DEMO sobrecalentamiento | DEMO fin

// Busca "clave=valor" dentro del comando (con espacio antes para no confundir t= con pot=)
String valorDe(const String& cmd, const String& clave) {
  String c = " " + cmd + " ";
  int i = c.indexOf(" " + clave + "=");
  if (i < 0) return "";
  i += clave.length() + 2;
  int f = c.indexOf(' ', i);
  return c.substring(i, f);
}

void procesarComando(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("MODO ")) {
    String m = cmd.substring(5);
    if (m == "off") {
      modo = APAGADO;
      manualEncendido = false;
      actualizarSalida(false);
    } else if (m == "auto") {
      modo = AUTOMATICO;
      manualEncendido = false;
    } else if (m == "manual_on") {
      modo = MANUAL;
      manualEncendido = true;
    } else if (m == "manual_off") {
      modo = MANUAL;
      manualEncendido = false;
      actualizarSalida(false);
    }
  }
  else if (cmd.startsWith("SET")) {
    String v;
    v = valorDe(cmd, "t");
    if (v.length()) {
      float nuevo = v.toFloat();
      if (nuevo >= 20.0 && nuevo <= 65.0) {
        if (fabsf(nuevo - objetivo) > 0.05f) cancelarPrueba("se cambio el objetivo");
        objetivo = nuevo;
      }
    }
    v = valorDe(cmd, "tol");
    if (v.length()) { float x = v.toFloat(); if (x >= 0.5 && x <= 5.0) tolerancia = x; }
    v = valorDe(cmd, "pot");
    if (v == "low" || v == "med" || v == "high") nivelPotencia = v;

    // TIPO DE CONTROL Y AJUSTE DEL PID
    bool guardar = false;
    v = valorDe(cmd, "ctl");
    if (v.length()) {
      guardar = true;
      cancelarPrueba("se cambio el tipo de control");
      if (v == "pid" && tipoControl != PID_CTRL) {
        tipoControl = PID_CTRL;
        pidActivo = false;          // arranca limpio
        tempAnterior = NAN;
        if (atEstado.startsWith("Midiendo") || atEstado.startsWith("Subiendo")) atEstado = "Cancelado por el usuario";
      } else if (v == "rafaga") {
        tipoControl = RAFAGA_FIJA;
        pidDuty = 0.0f;
        if (atEstado.startsWith("Midiendo") || atEstado.startsWith("Subiendo")) atEstado = "Cancelado por el usuario";
      } else if (v == "autotune") {
        iniciarAutoajuste();
      }
    }
    // Calibracion: cal = offset directo en °C, o ref = lo que marca el termometro de referencia
    v = valorDe(cmd, "cal");
    if (v.length()) { float x = v.toFloat(); if (x >= -10 && x <= 10) { offsetCal = x; guardar = true; } }
    v = valorDe(cmd, "ref");
    if (v.length()) {
      float ref = v.toFloat();
      float nuevo = offsetCal + (ref - temperatura);
      if (ref > 0 && nuevo >= -10 && nuevo <= 10) { offsetCal = nuevo; temperatura = ref; tempAnterior = NAN; guardar = true; }
    }
    v = valorDe(cmd, "kp");    if (v.length()) { float x = v.toFloat(); if (x >= 0 && x <= 5)   { Kp = x; guardar = true; } }
    v = valorDe(cmd, "ki");    if (v.length()) { float x = v.toFloat(); if (x >= 0 && x <= 1)   { Ki = x; guardar = true; } }
    v = valorDe(cmd, "kd");    if (v.length()) { float x = v.toFloat(); if (x >= 0 && x <= 50)  { Kd = x; guardar = true; } }
    v = valorDe(cmd, "banda"); if (v.length()) { float x = v.toFloat(); if (x >= 1 && x <= 20)  { bandaPID = x; guardar = true; } }
    if (guardar) guardarAjustes();
  }
  else if (cmd.startsWith("PRUEBA ")) {
    if (cmd.startsWith("PRUEBA iniciar")) {
      long m = cmd.substring(14).toInt();
      if (m < 5) m = 15;
      if (m > 40) m = 40;
      iniciarPrueba((unsigned long)m);
    } else if (cmd == "PRUEBA cancelar") {
      cancelarPrueba("por el usuario");
    }
  }
  else if (cmd.startsWith("DEMO ")) {
    if (cmd == "DEMO sobrecalentamiento") iniciarDemo();
    else if (cmd == "DEMO fin") demoHasta = 0;
  }

  actualizarControl();
}

// =======================================
// COMUNICACION POR CABLE USB (SERIAL)
// =======================================
// ESP32 -> dashboard: una linea JSON cada 500 ms con el estado.
unsigned long ultimoEnvio = 0;
const unsigned long INTERVALO_ENVIO = 500;
String bufferComando = "";

void enviarEstado() {
  Serial.println(estadoJSON());
}

void leerComandos() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufferComando.length()) {
        procesarComando(bufferComando);
        enviarEstado(); // respuesta inmediata para que el dashboard se actualice
      }
      bufferComando = "";
    } else if (bufferComando.length() < 120) {
      bufferComando += c;
    }
  }
}

// =======================================
// CABECERAS CORS
// =======================================
void enviarCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
}

// =======================================
// RUTAS DEL SERVIDOR (WI-FI)
// =======================================
void configurarWeb() {

  // PREFLIGHT CORS
  server.on("/estado", HTTP_OPTIONS, []() { enviarCORS(); server.send(204); });
  server.on("/set",    HTTP_OPTIONS, []() { enviarCORS(); server.send(204); });
  server.on("/modo",   HTTP_OPTIONS, []() { enviarCORS(); server.send(204); });
  server.on("/cmd",    HTTP_OPTIONS, []() { enviarCORS(); server.send(204); });

  // PAGINA PRINCIPAL: el dashboard completo es index.html (se abre desde la PC)
  server.on("/", []() {
    String h = "<!DOCTYPE html><html lang=es><head><meta charset=UTF-8><meta name=viewport content='width=device-width,initial-scale=1'>"
               "<title>Horno 4.0</title><style>body{font-family:sans-serif;background:#0c1527;color:#f8fafc;max-width:520px;margin:30px auto;padding:0 16px}"
               "a{display:block;background:#2563eb;color:#fff;padding:12px;border-radius:8px;text-decoration:none;margin:10px 0;font-weight:700}"
               ".s{background:#132238;padding:12px;border-radius:8px}</style></head><body><h2>Horno 4.0</h2>"
               "<p class=s>Temperatura: <b>" + String(demoActiva() ? DEMO_TEMP : temperatura, 1) + " &deg;C</b> &middot; Objetivo " + String(objetivo, 1) +
               " &deg;C &middot; " + nombreModo() + " / " + nombreControl() + "</p>"
               "<p>El panel completo es <b>index.html</b>: abrelo en la PC y se conecta solo por Wi-Fi (horno.local) o con el boton Conectar USB.</p>"
               "<a href=/prueba>Prueba completa (rafagas vs PID)</a><a href=/wifi>Configurar Wi-Fi</a><a href=/estado style='background:#475569'>Estado en JSON</a>"
               "</body></html>";
    server.send(200, "text/html", h);
  });

  // DATOS JSON
  server.on("/estado", HTTP_GET, []() {
    enviarCORS();
    server.send(200, "application/json", estadoJSON());
  });

  // COMANDO DE TEXTO (el mismo protocolo que por USB)
  server.on("/cmd", HTTP_GET, []() {
    enviarCORS();
    procesarComando(server.arg("c"));
    server.send(200, "text/plain", "OK");
  });

  // MODO (compatibilidad: /modo?m=auto)
  server.on("/modo", HTTP_GET, []() {
    enviarCORS();
    procesarComando("MODO " + server.arg("m"));
    server.send(200, "text/plain", "OK");
  });

  // SETPOINT Y AJUSTES (compatibilidad: /set?t=40&ctl=pid ...)
  server.on("/set", HTTP_GET, []() {
    enviarCORS();
    String cmd = "SET";
    for (int i = 0; i < server.args(); i++) {
      cmd += " " + server.argName(i) + "=" + server.arg(i);
    }
    procesarComando(cmd);
    server.send(200, "text/plain", "OK");
  });

  // PRUEBA COMPLETA
  server.on("/prueba", HTTP_GET, []() {
    server.send_P(200, "text/html", PRUEBA_HTML);
  });
  server.on("/prueba/iniciar", HTTP_GET, []() {
    enviarCORS();
    unsigned long m = server.hasArg("min") ? server.arg("min").toInt() : 15;
    if (m < 5) m = 5;
    if (m > 40) m = 40;
    iniciarPrueba(m);
    server.send(200, "text/plain", "OK");
  });
  server.on("/prueba/cancelar", HTTP_GET, []() {
    enviarCORS();
    cancelarPrueba("por el usuario");
    server.send(200, "text/plain", "OK");
  });
  server.on("/prueba.json", HTTP_GET, []() {
    enviarCORS();
    String j = "{\"etapa\":" + String((int)etapa);
    j += ",\"nombre\":\"" + String(NOMBRE_ETAPA[etapa]) + "\"";
    j += ",\"seg\":" + String(pruebaCorriendo() ? (millis() - etapaInicio) / 1000UL : 0UL);
    j += ",\"min\":" + String(pruebaMinutos);
    j += ",\"temp\":" + String(temperatura, 2);
    j += ",\"objetivo\":" + String(etapa == P_INACTIVA ? objetivo : pruebaObjetivo, 1);
    j += ",\"tbase\":" + String(etapa == P_INACTIVA ? temperatura : pruebaTbase, 1);
    j += ",\"nota\":\"" + pruebaNota + "\"";
    j += ",\"at\":\"" + atEstado + "\"";
    j += ",\"kp\":" + String(Kp, 3) + ",\"ki\":" + String(Ki, 4) + ",\"kd\":" + String(Kd, 3);
    j += ",\"rafaga\":" + metricasJSON(P_RAFAGA);
    j += ",\"pid\":" + metricasJSON(P_PID);
    j += "}";
    server.send(200, "application/json", j);
  });
  server.on("/prueba.csv", HTTP_GET, []() {
    enviarCORS();
    server.sendHeader("Content-Disposition", "attachment; filename=prueba_horno.csv");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/csv", "");
    server.sendContent("muestra,tiempo_s,etapa,temp_C,secadora,objetivo_C\n");
    String bloque = "";
    for (int i = 0; i < logN; i++) {
      bloque += String(i) + "," + String(i * (LOG_CADA_MS / 1000UL)) + "," + String(logInfo[i] & 0x0F) + "," +
                String(logT[i] / 10.0f, 1) + "," + String((logInfo[i] & 0x10) ? 1 : 0) + "," + String(pruebaObjetivo, 1) + "\n";
      if (bloque.length() > 1200) { server.sendContent(bloque); bloque = ""; }
    }
    if (bloque.length()) server.sendContent(bloque);
    server.sendContent("");
  });

  // CONFIGURAR WIFI DE LA CASA
  server.on("/wifi", HTTP_GET, []() {
    String opciones = "";
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n && i < 20; i++) {
      String nombre = WiFi.SSID(i);
      if (nombre.length() == 0 || opciones.indexOf(">" + nombre + "<") >= 0) continue;
      nombre.replace("\"", "&quot;");
      opciones += "<option value=\"" + nombre + "\">" + nombre + "</option>";
    }
    WiFi.scanDelete();
    String estado = wifiCasaConectado
      ? "Conectado a <b>" + WiFi.SSID() + "</b> &middot; <a href=\"http://horno.local\">horno.local</a> &middot; IP " + WiFi.localIP().toString()
      : (wifiCasaSsid.length() ? "Intentando conectar a <b>" + wifiCasaSsid + "</b>..." : "Sin Wi-Fi de casa configurado");
    String h = "<!DOCTYPE html><html lang=es><head><meta charset=UTF-8><meta name=viewport content='width=device-width,initial-scale=1'>"
               "<title>Wi-Fi del horno</title><style>body{font-family:sans-serif;background:#0c1527;color:#f8fafc;max-width:420px;margin:30px auto;padding:0 16px}"
               "input,select,button{width:100%;padding:10px;margin:6px 0 12px;border-radius:8px;border:1px solid #475569;background:#132238;color:#f8fafc;font-size:1rem}"
               "button{background:#2563eb;border:0;font-weight:700}a{color:#38bdf8}.s{background:#132238;padding:10px;border-radius:8px;font-size:.9rem}</style></head><body>"
               "<h2>Wi-Fi del horno</h2><p class=s>" + estado + "</p>"
               "<form method=POST action=/wifi>"
               "<label>Red (solo 2.4 GHz)</label><select onchange=\"document.getElementById('s').value=this.value\"><option value=''>-- elegir --</option>" + opciones + "</select>"
               "<input id=s name=ssid placeholder='Nombre de la red' value=\"" + wifiCasaSsid + "\">"
               "<label>Clave</label><input name=pass type=password placeholder='Clave del Wi-Fi'>"
               "<button>Guardar y conectar</button></form>"
               "<form method=POST action=/wifi><input type=hidden name=borrar value=1><button style='background:#475569'>Olvidar red</button></form>"
               "<p style='font-size:.85rem;opacity:.8'>La red propia <b>Horno4_0</b> (192.168.4.1) sigue funcionando siempre como respaldo.</p></body></html>";
    server.send(200, "text/html", h);
  });

  server.on("/wifi", HTTP_POST, []() {
    if (server.hasArg("borrar")) {
      wifiCasaSsid = ""; wifiCasaPass = "";
      prefs.remove("wssid"); prefs.remove("wpass");
      WiFi.disconnect(false, true);
    } else if (server.arg("ssid").length() > 0) {
      wifiCasaSsid = server.arg("ssid");
      wifiCasaPass = server.arg("pass");
      prefs.putString("wssid", wifiCasaSsid);
      prefs.putString("wpass", wifiCasaPass);
      WiFi.disconnect(false, false);
      WiFi.begin(wifiCasaSsid.c_str(), wifiCasaPass.c_str());
    }
    server.sendHeader("Location", "/wifi");
    server.send(303);
  });

  // enableCORS(true) duplicaba Access-Control-Allow-Origin (ya lo pone enviarCORS)
  server.begin();
}

// =======================================
// SETUP
// =======================================
void setup() {
  pinMode(PIN_TRIAC, OUTPUT);
  digitalWrite(PIN_TRIAC, LOW);
  actualizarSalida(false);

  Serial.begin(115200);

  pinMode(PIN_LM35, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LM35, ADC_0db);   // 0-950 mV: suficiente para 10-95 °C, mas fino

  cargarAjustes();

  Wire.begin(PIN_SDA, PIN_SCL);
  oledDisponible = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Lectura inicial del sensor
  temperatura = leerLM35();
  if (!isfinite(temperatura) || temperatura < TEMP_MIN_VALIDA) {
    temperatura = 24.0f;
  }

  actualizarControl();

  // AP propio + cliente del Wi-Fi de la casa al mismo tiempo
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(true);
  bool wifiOK = WiFi.softAP(ssid, password);
  if (wifiCasaSsid.length() > 0) {
    WiFi.begin(wifiCasaSsid.c_str(), wifiCasaPass.c_str());
  }
  if (MDNS.begin("horno")) {
    MDNS.addService("http", "tcp", 80);
  }
  configurarWeb();

  if (wifiOK) {
    Serial.println("\n==================================");
    Serial.println("  HORNO 4.0 GG - LISTO Y CONECTADO");
    Serial.println("==================================");
    Serial.print("Red Wi-Fi: "); Serial.println(ssid);
    Serial.print("Clave:     "); Serial.println(password);
    Serial.print("Web:       http://"); Serial.println(WiFi.softAPIP());
    Serial.println("==================================");
  }
  Serial.println("HORNO 4.0 GG - listo (USB + Wi-Fi)");

  actualizarOLED();
}

// =======================================
// LOOP PRINCIPAL
// =======================================
void loop() {
  server.handleClient();
  leerComandos();

  unsigned long ahora = millis();

  // Control térmico cada 100 ms
  if (ahora - ultimaLectura >= INTERVALO) {
    ultimaLectura = ahora;
    medirTemperatura();
    actualizarPrueba();
    actualizarControl();
  }

  // Rafaga del PID: se revisa en cada vuelta para cortar justo a tiempo
  aplicarVentanaPID();

  // Aviso cuando entra o se cae el Wi-Fi de la casa
  bool conectado = (WiFi.status() == WL_CONNECTED);
  if (conectado != wifiCasaConectado) {
    wifiCasaConectado = conectado;
    if (conectado) {
      Serial.printf("[WIFI CASA] Conectado a %s -> http://horno.local  o  http://%s\n",
                    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    } else {
      Serial.println("[WIFI CASA] Desconectado (sigue la red Horno4_0)");
    }
  }

  // Estado al dashboard por USB cada 500 ms (una linea JSON)
  if (ahora - ultimoEnvio >= INTERVALO_ENVIO) {
    ultimoEnvio = ahora;
    enviarEstado();
  }

  // Pantalla OLED cada 500 ms
  if (ahora - ultimoRefrescoOLED >= INTERVALO_OLED) {
    ultimoRefrescoOLED = ahora;
    actualizarOLED();
  }
}
