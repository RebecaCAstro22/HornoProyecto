#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

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
// NORMAL : lejos del objetivo (T < objetivo - tolerancia) -> TRIAC continuo para subir rapido
// RAFAGA : cerca del objetivo (objetivo - tolerancia <= T < objetivo) -> pulsos ON/OFF para mantener
// ESPERA : T >= objetivo -> TRIAC apagado
String faseAuto = "ESPERA";
const unsigned long RAFAGA_PERIODO_MS = 2000;  // ventana de la rafaga
const unsigned long RAFAGA_ON_MS      = 1000;  // tiempo encendido dentro de la ventana (50%)

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
// LECTURA DIRECTA Y CALIBRADA DEL LM35
// =======================================
// Diagnóstico del sensor (para detectar el LM35 desconectado)
float ultimaLecturaMv = 0;        // mV promedio de la última lectura
float ultimoRuidoMv = 0;          // diferencia max-min entre muestras (mV)
int lecturasMalas = 0;            // lecturas malas seguidas
int lecturasBuenas = 0;           // lecturas buenas seguidas
const int LECTURAS_PARA_FALLA = 5;     // 5 x 100 ms = 0.5 s para declarar desconectado
const int LECTURAS_PARA_RECUPERAR = 10; // 1 s estable para volver a confiar
const float RUIDO_MAX_MV = 150.0f;      // el LM35 da una señal estable; un pin al aire salta mucho
// El ADC del ESP32 (atenuación 11 dB) no baja de ~140 mV aunque la entrada esté en 0 V.
// Por eso, con el LM35 desconectado, la lectura se queda fija en ~14 °C.
// Una lectura real en interiores siempre está arriba de esto, así que todo lo que quede
// por debajo de este valor se toma como sensor desconectado.
const float TEMP_MIN_VALIDA = 16.0f;

float leerLM35() {
  uint32_t sumaRaw = 0;
  uint32_t sumaMv = 0;
  int rawMin = 4095, rawMax = 0;
  const int MUESTRAS = 32;

  for (int i = 0; i < MUESTRAS; i++) {
    int r = analogRead(PIN_LM35);
    sumaRaw += r;
    if (r < rawMin) rawMin = r;
    if (r > rawMax) rawMax = r;
    sumaMv += analogReadMilliVolts(PIN_LM35);
    delayMicroseconds(100);
  }

  ultimoRuidoMv = ((rawMax - rawMin) * 3300.0f) / 4095.0f;

  float rawProm = (float)sumaRaw / MUESTRAS;
  float mvCore = (float)sumaMv / MUESTRAS;
  
  // Cálculo analógico de respaldo: 4095 = 3300 mV
  float mvCalc = (rawProm * 3300.0f) / 4095.0f;

  // Si analogReadMilliVolts da valor realista usamos mvCore, de lo contrario mvCalc
  float mv = (mvCore > 30.0f) ? mvCore : mvCalc;
  ultimaLecturaMv = mv;

  // 10 mV por cada grado Celsius
  float t = mv / 10.0f;

  return t;
}

// =======================================
// NOMBRE DEL MODO
// =======================================
String nombreModo() {
  if (modo == MANUAL) return "MANUAL";
  if (modo == AUTOMATICO) return "AUTOMATICO";
  return "APAGADO";
}

// =======================================
// ALGORITMO DE CONTROL TERMICO (AUTONOMO)
// =======================================
void actualizarControl() {
  float cruda = leerLM35();

  // Lectura buena: dentro de rango físico y sin ruido de pin al aire
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

  // SEGURIDAD: sin sensor el modo automático no sabe la temperatura -> secadora apagada
  if (!sensorValido && modo == AUTOMATICO) {
    faseAuto = "SIN_SENSOR";
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

  // 3. MODO AUTOMATICO: potencia NORMAL para subir, RAFAGA para mantener
  if (modo == AUTOMATICO) {
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
  oled.println("HORNO 4.0 GG");
  oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  // Lectura Actual
  oled.setCursor(0, 15);
  oled.print("ACT: ");
  oled.setTextSize(2);
  if (sensorValido) {
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
  }

  oled.display();
}

// =======================================
// COMUNICACIÓN POR CABLE USB (SERIAL)
// =======================================
// El dashboard (index.html en Chrome/Edge) se conecta por USB.
// ESP32 -> dashboard: una línea JSON cada 500 ms con el estado.
// Dashboard -> ESP32: comandos de texto terminados en salto de línea:
//   MODO auto | MODO off | MODO manual_on | MODO manual_off
//   SET t=40.0 tol=2.0 pot=high
// También se pueden escribir a mano desde el Monitor Serial del Arduino IDE.

unsigned long ultimoEnvio = 0;
const unsigned long INTERVALO_ENVIO = 500;
String bufferComando = "";

void enviarEstado() {
  String json = "{";
  json += "\"valido\":" + String(sensorValido ? "true" : "false");
  json += ",\"sensor\":\"" + String(sensorValido ? "OK" : "DESCONECTADO") + "\"";
  json += ",\"mv\":" + String(ultimaLecturaMv, 0);
  json += ",\"temperatura\":" + (sensorValido ? String(temperatura, 1) : String("null"));
  json += ",\"objetivo\":" + String(objetivo, 1);
  json += ",\"tolerancia\":" + String(tolerancia, 1);
  json += ",\"potencia\":\"" + nivelPotencia + "\"";
  json += ",\"calefaccion\":" + String(calefaccion ? "true" : "false");
  json += ",\"modo\":\"" + nombreModo() + "\"";
  json += ",\"manualEncendido\":" + String(manualEncendido ? "true" : "false");
  json += ",\"fase\":\"" + String(modo == AUTOMATICO ? faseAuto : "") + "\"";
  json += "}";
  Serial.println(json);
}

// Busca "clave=valor" dentro del comando SET
String valorDe(const String& cmd, const String& clave) {
  int i = cmd.indexOf(clave + "=");
  if (i < 0) return "";
  i += clave.length() + 1;
  int f = cmd.indexOf(' ', i);
  return (f < 0) ? cmd.substring(i) : cmd.substring(i, f);
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
      actualizarSalida(true);
    } else if (m == "manual_off") {
      modo = MANUAL;
      manualEncendido = false;
      actualizarSalida(false);
    }
  }
  else if (cmd.startsWith("SET")) {
    String t = valorDe(cmd, "t");
    if (t.length()) {
      float nuevo = t.toFloat();
      if (nuevo >= 20.0 && nuevo <= 65.0) objetivo = nuevo;
    }
    String tol = valorDe(cmd, "tol");
    if (tol.length()) {
      float v = tol.toFloat();
      if (v >= 0.5 && v <= 5.0) tolerancia = v;
    }
    String pot = valorDe(cmd, "pot");
    if (pot == "low" || pot == "med" || pot == "high") nivelPotencia = pot;
  }

  actualizarControl();
  enviarEstado(); // respuesta inmediata para que el dashboard se actualice
}

void leerComandos() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      procesarComando(bufferComando);
      bufferComando = "";
    } else if (bufferComando.length() < 80) {
      bufferComando += c;
    }
  }
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
  analogSetPinAttenuation(PIN_LM35, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  oledDisponible = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Lectura inicial del sensor
  temperatura = leerLM35();
  if (!isfinite(temperatura) || temperatura < TEMP_MIN_VALIDA) {
    temperatura = 24.0f;
  }

  actualizarControl();
  Serial.println("HORNO 4.0 GG - listo (conexion por USB)");
  actualizarOLED();
}

// =======================================
// LOOP PRINCIPAL
// =======================================
void loop() {
  leerComandos();

  unsigned long ahora = millis();

  // Control térmico cada 100 ms
  if (ahora - ultimaLectura >= INTERVALO) {
    ultimaLectura = ahora;
    actualizarControl();
  }

  // Estado al dashboard cada 500 ms
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
