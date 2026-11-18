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
float leerLM35() {
  uint32_t sumaRaw = 0;
  uint32_t sumaMv = 0;
  const int MUESTRAS = 32;

  for (int i = 0; i < MUESTRAS; i++) {
    sumaRaw += analogRead(PIN_LM35);
    sumaMv += analogReadMilliVolts(PIN_LM35);
    delayMicroseconds(100);
  }

  float rawProm = (float)sumaRaw / MUESTRAS;
  float mvCore = (float)sumaMv / MUESTRAS;
  
  // Cálculo analógico de respaldo: 4095 = 3300 mV
  float mvCalc = (rawProm * 3300.0f) / 4095.0f;

  // Si analogReadMilliVolts da valor realista usamos mvCore, de lo contrario mvCalc
  float mv = (mvCore > 30.0f) ? mvCore : mvCalc;

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

  if (isfinite(cruda) && cruda >= 2.0f && cruda <= 120.0f) {
    sensorValido = true;
    temperatura = (ALPHA * cruda) + ((1.0f - ALPHA) * temperatura);
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
  oled.setCursor(42, 13);
  oled.print(temperatura, 1);
  oled.setTextSize(1);
  oled.println(" C");

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
// DASHBOARD WEB HTML EN MEMORIA FLASH
// =======================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Horno Térmico 4.0 GG - Control Lonchera</title>
  <style>
    :root {
      --bg-desktop: #0c1527;
      --bg-top-card: linear-gradient(135deg, #132238 0%, #1b3152 50%, #16263e 100%);
      --top-border: rgba(96, 165, 250, 0.2);
      --card-white: #f8fafc;
      --card-white-sub: #f1f5f9;
      --text-dark: #0f172a;
      --text-dark-muted: #64748b;
      --text-light: #f8fafc;
      --text-light-muted: #94a3b8;
      --blue-accent: #38bdf8;
      --blue-glow: rgba(56, 189, 248, 0.4);
      --orange-heat: #f97316;
      --green-safe: #10b981;
      --purple-accent: #818cf8;
      --border-light: #e2e8f0;
    }

    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    }

    body {
      background: #080f1d;
      color: var(--text-light);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 20px 28px;
      overflow-x: hidden;
    }

    /* CONTENEDOR DESKTOP COMPLETO (ANCHO PANORÁMICO) */
    .dashboard-container {
      width: 100%;
      max-width: 1420px;
      display: flex;
      flex-direction: column;
      gap: 18px;
    }

    /* BARRA SUPERIOR (HEADER DE ESCRITORIO) */
    .desktop-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 4px 0;
    }

    .brand-section {
      display: flex;
      align-items: center;
      gap: 12px;
    }

    .brand-icon {
      width: 40px;
      height: 40px;
      background: conic-gradient(from 180deg at 50% 50%, #ec4899, #8b5cf6, #3b82f6, #06b6d4, #10b981, #ec4899);
      border-radius: 50%;
      padding: 3px;
      display: flex;
      align-items: center;
      justify-content: center;
    }

    .brand-icon-inner {
      width: 100%;
      height: 100%;
      background: #0c1527;
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 1.1rem;
    }

    .brand-name {
      font-size: 1.45rem;
      font-weight: 800;
      letter-spacing: -0.5px;
      color: #ffffff;
      display: flex;
      align-items: center;
      gap: 4px;
    }

    .brand-name span {
      color: #38bdf8;
      font-weight: 400;
    }

    /* PÍLDORA CENTRAL DE ESTADO */
    .top-status-pill {
      background: rgba(19, 34, 56, 0.85);
      border: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 30px;
      padding: 6px 18px;
      display: flex;
      align-items: center;
      gap: 20px;
      font-size: 0.85rem;
      box-shadow: 0 4px 14px rgba(0, 0, 0, 0.3);
    }

    .pill-metric {
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .pill-metric .val-big {
      font-size: 1.25rem;
      font-weight: 800;
      color: #ffffff;
    }

    .pill-text-status {
      font-size: 0.78rem;
      color: #94a3b8;
    }

    .pill-text-status strong {
      color: #f87171;
    }

    /* ACCIONES HEADER DERECHO */
    .header-actions {
      display: flex;
      align-items: center;
      gap: 14px;
    }

    .mode-switch-btn {
      background: rgba(30, 41, 59, 0.8);
      border: 1px solid rgba(255, 255, 255, 0.15);
      color: var(--text-light-muted);
      padding: 6px 14px;
      border-radius: 20px;
      font-size: 0.8rem;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 6px;
      transition: all 0.2s;
    }

    .mode-switch-btn.pilot-active {
      background: rgba(99, 102, 241, 0.2);
      border-color: #818cf8;
      color: #c7d2fe;
    }

    .avatar-circle {
      width: 36px;
      height: 36px;
      border-radius: 50%;
      background: #334155;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 0.9rem;
      color: #f8fafc;
      border: 1px solid rgba(255, 255, 255, 0.15);
    }

    /* ========================================================= */
    /* VISTAS: DASHBOARD PRINCIPAL VS HISTORIAL (PÁGINAS)         */
    /* ========================================================= */
    .view-container {
      display: none;
      flex-direction: column;
      gap: 18px;
      width: 100%;
    }

    .view-container.active {
      display: flex;
      animation: fadeIn 0.25s ease-out;
    }

    @keyframes fadeIn {
      from { opacity: 0; transform: translateY(4px); }
      to { opacity: 1; transform: translateY(0); }
    }

    /* ========================================================= */
    /* SECCIÓN SUPERIOR: HERO CARD AZUL PROFUNDO (COMO LA IMAGEN)*/
    /* ========================================================= */
    .hero-panel {
      background: var(--bg-top-card);
      border: 1px solid var(--top-border);
      border-radius: 28px;
      padding: 26px 32px;
      box-shadow: 0 20px 50px rgba(0, 0, 0, 0.45), inset 0 1px 0 rgba(255, 255, 255, 0.12);
      display: grid;
      grid-template-columns: 280px 1fr 310px;
      gap: 24px;
      position: relative;
      overflow: hidden;
      min-height: 360px;
      align-items: center;
    }

    /* COLUMNA IZQUIERDA DEL HERO */
    .hero-col-left {
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      height: 100%;
    }

    .hero-title-wrap {
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 1.15rem;
      font-weight: 700;
      color: #ffffff;
      margin-bottom: 6px;
    }

    .badge-suboptimal {
      font-size: 0.72rem;
      padding: 2px 8px;
      border-radius: 12px;
      font-weight: 600;
      transition: all 0.3s;
    }

    .badge-suboptimal.stable {
      background: rgba(16, 185, 129, 0.2);
      color: #34d399;
      border: 1px solid #10b981;
    }

    .badge-suboptimal.heating {
      background: rgba(249, 115, 22, 0.2);
      color: #fb923c;
      border: 1px solid #f97316;
    }

    .badge-suboptimal.cooling {
      background: rgba(56, 189, 248, 0.2);
      color: #38bdf8;
      border: 1px solid #38bdf8;
    }

    .hero-desc {
      font-size: 0.8rem;
      color: var(--text-light-muted);
      line-height: 1.4;
      margin-bottom: 12px;
    }

    /* RADAR / PENTÁGONO TÉRMICO (IGUAL A LA IMAGEN) */
    .radar-wrapper {
      width: 100%;
      height: 180px;
      display: flex;
      align-items: center;
      justify-content: center;
      position: relative;
    }

    canvas#radarCanvas {
      width: 100%;
      height: 100%;
    }

    /* COLUMNA CENTRAL: EL GRAN ARCO TÉRMICO CON ANIMACIÓN */
    .hero-col-center {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      position: relative;
      padding: 10px 0;
    }

    .center-gauge-box {
      position: relative;
      width: 480px;
      height: 280px;
      display: flex;
      align-items: center;
      justify-content: center;
    }

    svg.main-arc-svg {
      width: 100%;
      height: 100%;
      overflow: visible;
    }

    .arc-bg-dashed {
      fill: none;
      stroke: rgba(255, 255, 255, 0.1);
      stroke-width: 1.5;
      stroke-dasharray: 4, 6;
    }

    .arc-track-thick {
      fill: none;
      stroke: rgba(255, 255, 255, 0.08);
      stroke-width: 16;
      stroke-linecap: round;
    }

    .arc-range-highlight {
      fill: none;
      stroke: rgba(16, 185, 129, 0.25);
      stroke-width: 20;
      stroke-linecap: round;
      transition: all 0.4s ease;
    }

    .arc-fill-gradient {
      fill: none;
      stroke: url(#arcGradientFlow);
      stroke-width: 16;
      stroke-linecap: round;
      stroke-dasharray: 580;
      stroke-dashoffset: 580;
      transition: stroke-dashoffset 0.6s cubic-bezier(0.2, 0.8, 0.2, 1);
      filter: drop-shadow(0 0 12px rgba(56, 189, 248, 0.6));
    }

    /* Puntos orbitales alrededor del arco (estilo imagen: Stroke, Cancer, Heart Disease, etc.) */
    .orbit-point {
      position: absolute;
      text-align: center;
      font-size: 0.68rem;
      color: #94a3b8;
      pointer-events: none;
    }

    .orbit-point .orbit-val {
      font-size: 0.85rem;
      font-weight: 700;
      color: #f8fafc;
      display: block;
    }

    .orbit-point .orbit-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: #38bdf8;
      box-shadow: 0 0 8px #38bdf8;
      margin: 3px auto;
    }

    /* Centro del Dial */
    .gauge-center-content {
      position: absolute;
      bottom: 24px;
      left: 0;
      right: 0;
      text-align: center;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
    }

    .gauge-temperature-big {
      font-size: 4.6rem;
      font-weight: 800;
      line-height: 1;
      letter-spacing: -2px;
      color: #ffffff;
      display: flex;
      align-items: flex-start;
      justify-content: center;
      text-shadow: 0 6px 25px rgba(0, 0, 0, 0.5);
    }

    .gauge-temperature-big .unit {
      font-size: 2rem;
      font-weight: 600;
      color: #38bdf8;
      margin-left: 6px;
      margin-top: 6px;
    }

    .gauge-caption {
      font-size: 0.85rem;
      color: var(--text-light-muted);
      letter-spacing: 0.5px;
      margin-top: 4px;
    }

    /* ANIMACIÓN DE MANTENIMIENTO TÉRMICO EN EL CENTRO */
    .range-pulse-indicator {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      margin-top: 10px;
      padding: 6px 16px;
      border-radius: 20px;
      font-size: 0.82rem;
      font-weight: 700;
      background: rgba(15, 23, 42, 0.7);
      border: 1px solid rgba(255, 255, 255, 0.1);
      color: #cbd5e1;
      transition: all 0.3s;
    }

    .range-pulse-indicator.active-pulse {
      background: rgba(16, 185, 129, 0.18);
      border: 1px solid #10b981;
      color: #34d399;
      box-shadow: 0 0 25px rgba(16, 185, 129, 0.4);
      animation: pulseGreen 2s infinite ease-in-out;
    }

    @keyframes pulseGreen {
      0%, 100% { transform: scale(1); box-shadow: 0 0 15px rgba(16, 185, 129, 0.3); }
      50% { transform: scale(1.03); box-shadow: 0 0 32px rgba(16, 185, 129, 0.6); }
    }

    .range-pulse-indicator.heating-pulse {
      background: rgba(249, 115, 22, 0.2);
      border: 1px solid #f97316;
      color: #fb923c;
      animation: pulseOrange 1.4s infinite ease-in-out;
    }

    @keyframes pulseOrange {
      0%, 100% { transform: scale(1); box-shadow: 0 0 15px rgba(249, 115, 22, 0.3); }
      50% { transform: scale(1.03); box-shadow: 0 0 30px rgba(249, 115, 22, 0.6); }
    }

    /* COLUMNA DERECHA DEL HERO */
    .hero-col-right {
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      height: 100%;
      gap: 12px;
    }

    .mini-stat-card {
      background: rgba(15, 23, 42, 0.5);
      border: 1px solid rgba(255, 255, 255, 0.08);
      border-radius: 18px;
      padding: 14px 18px;
      display: flex;
      flex-direction: column;
      gap: 6px;
    }

    .mini-stat-top {
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .mini-stat-val {
      font-size: 1.6rem;
      font-weight: 800;
      color: #ffffff;
    }

    .mini-stat-tag {
      font-size: 0.72rem;
      font-weight: 700;
      padding: 3px 10px;
      border-radius: 12px;
      background: #f59e0b;
      color: #000000;
    }

    .mini-stat-bar {
      width: 100%;
      height: 6px;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 3px;
      overflow: hidden;
    }

    .mini-stat-fill {
      height: 100%;
      background: #38bdf8;
      border-radius: 3px;
      width: 75%;
    }

    .mini-chart-container {
      width: 100%;
      height: 110px;
      position: relative;
    }

    canvas#previewCanvas {
      width: 100%;
      height: 100%;
    }

    .btn-view-full-report {
      width: 100%;
      background: #ffffff;
      color: #0f172a;
      border: none;
      border-radius: 20px;
      padding: 10px 16px;
      font-size: 0.85rem;
      font-weight: 700;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      transition: all 0.2s;
      box-shadow: 0 4px 14px rgba(255, 255, 255, 0.2);
    }

    .btn-view-full-report:hover {
      background: #f1f5f9;
      transform: translateY(-1px);
    }

    /* ========================================================= */
    /* FILA INFERIOR: 3 TARJETAS BLANCAS CLARAS (EXACTO A LA IMAGEN) */
    /* ========================================================= */
    .bottom-cards-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 20px;
      width: 100%;
    }

    .white-card {
      background: var(--card-white);
      border-radius: 24px;
      padding: 24px;
      color: var(--text-dark);
      box-shadow: 0 16px 36px rgba(0, 0, 0, 0.28);
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      gap: 16px;
      min-height: 380px;
      transition: transform 0.2s ease, box-shadow 0.2s ease;
    }

    .white-card:hover {
      transform: translateY(-2px);
      box-shadow: 0 20px 42px rgba(0, 0, 0, 0.35);
    }

    .white-card-header {
      display: flex;
      justify-content: space-between;
      align-items: flex-start;
    }

    .white-card-title {
      font-size: 1.1rem;
      font-weight: 800;
      color: var(--text-dark);
      letter-spacing: -0.3px;
    }

    .white-card-subtitle {
      font-size: 0.78rem;
      color: var(--text-dark-muted);
      line-height: 1.4;
      margin-top: 4px;
    }

    /* TARJETA 1: REGULACIÓN DE RANGO TÉRMICO (METHYLATION RISK SCORE) */
    .score-comparison-row {
      display: flex;
      align-items: baseline;
      gap: 12px;
      margin-top: 6px;
    }

    .score-big-number {
      font-size: 2.2rem;
      font-weight: 800;
      color: var(--text-dark);
    }

    .score-sub-label {
      font-size: 0.85rem;
      color: var(--text-dark-muted);
    }

    .score-badge-status {
      font-size: 0.75rem;
      font-weight: 700;
      padding: 4px 10px;
      border-radius: 12px;
      background: #fef3c7;
      color: #b45309;
      margin-left: auto;
    }

    /* Barra degradada interactiva con el indicador deslizante (exacto a la imagen) */
    .gradient-range-bar-box {
      margin: 12px 0 6px 0;
    }

    .gradient-track {
      width: 100%;
      height: 10px;
      border-radius: 6px;
      background: linear-gradient(90deg, #10b981 0%, #38bdf8 40%, #f59e0b 75%, #ef4444 100%);
      position: relative;
    }

    .gradient-pin-indicator {
      position: absolute;
      top: -6px;
      width: 14px;
      height: 22px;
      background: #0f172a;
      border: 2px solid #ffffff;
      border-radius: 4px;
      box-shadow: 0 2px 6px rgba(0, 0, 0, 0.4);
      transform: translateX(-50%);
      transition: left 0.4s cubic-bezier(0.2, 0.8, 0.2, 1);
    }

    .gradient-labels {
      display: flex;
      justify-content: space-between;
      font-size: 0.72rem;
      font-weight: 600;
      color: var(--text-dark-muted);
      margin-top: 6px;
    }

    /* Controles de edición del Rango */
    .controls-stepper-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      background: var(--card-white-sub);
      border-radius: 18px;
      padding: 12px;
      border: 1px solid var(--border-light);
    }

    .stepper-item {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 4px;
    }

    .stepper-label {
      font-size: 0.68rem;
      font-weight: 700;
      text-transform: uppercase;
      color: var(--text-dark-muted);
    }

    .stepper-actions {
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .btn-step-circle {
      width: 32px;
      height: 32px;
      border-radius: 50%;
      border: 1px solid #cbd5e1;
      background: #ffffff;
      font-size: 1.1rem;
      font-weight: 700;
      color: var(--text-dark);
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: all 0.15s;
    }

    .btn-step-circle:hover {
      background: #0f172a;
      color: #ffffff;
      border-color: #0f172a;
    }

    .stepper-val-text {
      font-size: 1.15rem;
      font-weight: 800;
      color: var(--text-dark);
      min-width: 50px;
      text-align: center;
    }

    /* TARJETA 2: REGULADOR DE SECADORA (IMMUNITY) */
    .immunity-percentage-row {
      display: flex;
      align-items: baseline;
      gap: 10px;
      margin-bottom: 8px;
    }

    .immunity-big-perc {
      font-size: 2.6rem;
      font-weight: 800;
      color: var(--text-dark);
    }

    .power-level-list {
      display: flex;
      flex-direction: column;
      gap: 10px;
    }

    .power-level-card-item {
      background: var(--card-white-sub);
      border: 1px solid var(--border-light);
      border-radius: 16px;
      padding: 12px 14px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      cursor: pointer;
      transition: all 0.2s ease;
    }

    .power-level-card-item:hover {
      border-color: #38bdf8;
      background: #ffffff;
    }

    .power-level-card-item.selected {
      background: #0f172a;
      color: #ffffff;
      border-color: #0f172a;
      box-shadow: 0 4px 14px rgba(15, 23, 42, 0.25);
    }

    .power-level-card-item.selected .power-name {
      color: #ffffff;
    }

    .power-level-card-item.selected .power-spec {
      background: rgba(255, 255, 255, 0.2);
      color: #ffffff;
    }

    .power-name {
      font-size: 0.88rem;
      font-weight: 700;
      color: var(--text-dark);
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .power-spec {
      font-size: 0.78rem;
      font-weight: 700;
      background: #e2e8f0;
      color: var(--text-dark);
      padding: 4px 10px;
      border-radius: 12px;
    }

    /* TARJETA 3: TIEMPOS Y DIAGNÓSTICO (INFLAMMATION) */
    .inflammation-stats-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 8px;
    }

    .timer-stat-box {
      text-align: center;
    }

    .timer-stat-box .title {
      font-size: 0.68rem;
      text-transform: uppercase;
      font-weight: 700;
      color: var(--text-dark-muted);
    }

    .timer-stat-box .val {
      font-size: 1.35rem;
      font-weight: 800;
      color: var(--text-dark);
      margin-top: 2px;
      font-variant-numeric: tabular-nums;
    }

    .explanation-callout {
      background: var(--card-white-sub);
      border-radius: 16px;
      padding: 12px 14px;
      font-size: 0.76rem;
      color: var(--text-dark-muted);
      line-height: 1.45;
      border-left: 4px solid #38bdf8;
    }

    .change-over-time-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.85rem;
      font-weight: 700;
      color: var(--text-dark);
      padding-top: 8px;
      border-top: 1px solid var(--border-light);
    }

    .change-badge-up {
      color: #10b981;
      font-weight: 700;
      display: flex;
      align-items: center;
      gap: 2px;
    }

    .bullet-diagnostics {
      display: flex;
      flex-direction: column;
      gap: 6px;
      font-size: 0.74rem;
      color: var(--text-dark-muted);
    }

    .bullet-diagnostics div {
      display: flex;
      align-items: flex-start;
      gap: 6px;
    }

    /* ========================================================= */
    /* VISTA DE HISTORIAL COMPLETO (PANTALLA COMPLETA SEPARADA)   */
    /* ========================================================= */
    .history-widescreen-panel {
      background: var(--bg-top-card);
      border: 1px solid var(--top-border);
      border-radius: 28px;
      padding: 28px 36px;
      box-shadow: 0 20px 50px rgba(0, 0, 0, 0.45);
      display: flex;
      flex-direction: column;
      gap: 20px;
    }

    .history-header-bar {
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .btn-back-dashboard {
      background: rgba(255, 255, 255, 0.1);
      border: 1px solid rgba(255, 255, 255, 0.2);
      color: #ffffff;
      padding: 8px 18px;
      border-radius: 20px;
      font-size: 0.85rem;
      font-weight: 700;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 8px;
      transition: all 0.2s;
    }

    .btn-back-dashboard:hover {
      background: rgba(255, 255, 255, 0.2);
      transform: translateX(-3px);
    }

    .history-chart-large-wrap {
      width: 100%;
      height: 430px;
      background: rgba(10, 16, 28, 0.7);
      border-radius: 20px;
      border: 1px solid rgba(255, 255, 255, 0.08);
      padding: 12px;
      position: relative;
    }

    canvas#largeHistoryChart {
      width: 100%;
      height: 100%;
      display: block;
    }

    .history-metrics-four {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 16px;
    }

    .metric-glass-box {
      background: rgba(15, 23, 42, 0.6);
      border: 1px solid rgba(255, 255, 255, 0.08);
      border-radius: 18px;
      padding: 16px;
      text-align: center;
    }

    .metric-glass-label {
      font-size: 0.72rem;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      color: var(--text-light-muted);
    }

    .metric-glass-val {
      font-size: 1.6rem;
      font-weight: 800;
      margin-top: 4px;
    }

    .history-log-table {
      background: rgba(10, 16, 28, 0.7);
      border: 1px solid rgba(255, 255, 255, 0.08);
      border-radius: 18px;
      padding: 16px;
      max-height: 200px;
      overflow-y: auto;
      font-family: monospace;
      font-size: 0.78rem;
      color: #94a3b8;
      display: flex;
      flex-direction: column;
      gap: 8px;
    }

    .log-row-item {
      display: flex;
      justify-content: space-between;
      border-bottom: 1px solid rgba(255, 255, 255, 0.05);
      padding-bottom: 6px;
    }

    .log-row-time { color: #38bdf8; font-weight: 700; }
    .log-row-msg { color: #ffffff; }

    /* CONTROLES Y PESTAÑAS DEL HISTORIAL ACADÉMICO */
    .history-tabs-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 12px;
      padding: 10px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.08);
    }

    .hist-tab-group {
      display: flex;
      gap: 8px;
    }

    .btn-hist-subtab {
      background: rgba(15, 23, 42, 0.7);
      border: 1px solid rgba(255, 255, 255, 0.1);
      color: var(--text-light-muted);
      padding: 6px 14px;
      border-radius: 14px;
      font-size: 0.78rem;
      font-weight: 700;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 6px;
      transition: all 0.2s;
    }

    .btn-hist-subtab:hover {
      background: rgba(255, 255, 255, 0.1);
      color: #ffffff;
    }

    .btn-hist-subtab.active {
      background: linear-gradient(135deg, #0284c7, #0ea5e9);
      color: #ffffff;
      border-color: #38bdf8;
      box-shadow: 0 0 14px rgba(56, 189, 248, 0.4);
    }

    .btn-export-csv {
      background: #10b981;
      color: #ffffff;
      border: none;
      padding: 7px 16px;
      border-radius: 14px;
      font-size: 0.78rem;
      font-weight: 700;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 6px;
      transition: all 0.2s;
    }

    .btn-export-csv:hover {
      background: #059669;
      transform: translateY(-1px);
    }

    /* TARJETA TEÓRICA DE MODELADO FÍSICO (PARA EL PROFESOR) */
    .theory-box {
      background: rgba(15, 23, 42, 0.75);
      border: 1px solid rgba(56, 189, 248, 0.25);
      border-radius: 18px;
      padding: 16px 20px;
      display: flex;
      flex-direction: column;
      gap: 10px;
    }

    .theory-box-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-size: 0.85rem;
      font-weight: 800;
      color: #38bdf8;
    }

    .theory-formula-chip {
      background: rgba(56, 189, 248, 0.12);
      border: 1px solid rgba(56, 189, 248, 0.3);
      padding: 4px 12px;
      border-radius: 12px;
      font-family: monospace;
      font-size: 0.82rem;
      color: #e0f2fe;
    }

    .theory-params-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 12px;
      font-size: 0.74rem;
      color: #cbd5e1;
    }

    .theory-param-item {
      background: rgba(10, 16, 28, 0.6);
      border-radius: 12px;
      padding: 8px 12px;
      border: 1px solid rgba(255, 255, 255, 0.05);
    }

    .theory-param-item strong {
      display: block;
      font-size: 0.95rem;
      color: #ffffff;
      margin-top: 2px;
    }

    /* GRÁFICA INFERIOR: TREN DE PULSOS & PICOS DE POTENCIA */
    .pulses-chart-wrap {
      width: 100%;
      height: 140px;
      background: rgba(10, 16, 28, 0.7);
      border-radius: 18px;
      border: 1px solid rgba(255, 255, 255, 0.08);
      padding: 10px;
      position: relative;
    }

    canvas#pulsesChart {
      width: 100%;
      height: 100%;
      display: block;
    }

    /* ENSAYOS GUARDADOS AUTOMÁTICAMENTE */
    .saved-essays-section {
      display: flex;
      flex-direction: column;
      gap: 10px;
    }

    .saved-essays-grid {
      display: grid;
      grid-template-columns: repeat(auto-fill, minmax(260px, 1fr));
      gap: 12px;
      max-height: 220px;
      overflow-y: auto;
    }

    .essay-card-item {
      background: rgba(15, 23, 42, 0.8);
      border: 1px solid rgba(255, 255, 255, 0.1);
      border-radius: 16px;
      padding: 12px 14px;
      cursor: pointer;
      display: flex;
      flex-direction: column;
      gap: 6px;
      transition: all 0.2s;
    }

    .essay-card-item:hover {
      border-color: #38bdf8;
      background: rgba(30, 41, 59, 0.9);
      transform: translateY(-2px);
    }

    .essay-card-item.selected {
      border-color: #10b981;
      background: rgba(16, 185, 129, 0.12);
      box-shadow: 0 0 14px rgba(16, 185, 129, 0.25);
    }

    .essay-top {
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .essay-target {
      font-size: 1.05rem;
      font-weight: 800;
      color: #ffffff;
    }

    .essay-time-tag {
      font-size: 0.7rem;
      color: #94a3b8;
    }

    .essay-meta-row {
      display: flex;
      justify-content: space-between;
      font-size: 0.72rem;
      color: #cbd5e1;
    }

    /* FRANJA DE MODOS: AUTOMÁTICO VS MANUAL DE PRUEBA */
    .mode-control-strip {
      background: var(--bg-top-card);
      border: 1px solid var(--top-border);
      border-radius: 20px;
      padding: 12px 24px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      box-shadow: 0 8px 24px rgba(0, 0, 0, 0.3);
      width: 100%;
    }

    .mode-strip-left {
      display: flex;
      align-items: center;
      gap: 14px;
    }

    .mode-strip-label {
      font-size: 0.76rem;
      font-weight: 700;
      letter-spacing: 0.8px;
      color: var(--text-light-muted);
    }

    .mode-buttons-group {
      display: flex;
      gap: 8px;
    }

    .btn-mode-select {
      background: rgba(15, 23, 42, 0.6);
      border: 1px solid rgba(255, 255, 255, 0.1);
      color: var(--text-light-muted);
      padding: 8px 16px;
      border-radius: 14px;
      font-size: 0.82rem;
      font-weight: 700;
      cursor: pointer;
      display: flex;
      align-items: center;
      gap: 6px;
      transition: all 0.2s ease;
    }

    .btn-mode-select:hover {
      background: rgba(255, 255, 255, 0.1);
      color: #ffffff;
    }

    .btn-mode-select.active.auto {
      background: linear-gradient(135deg, #1d4ed8, #2563eb);
      color: #ffffff;
      border-color: #38bdf8;
      box-shadow: 0 0 16px rgba(37, 99, 235, 0.5);
    }

    .btn-mode-select.active.manual {
      background: linear-gradient(135deg, #b45309, #d97706);
      color: #ffffff;
      border-color: #f59e0b;
      box-shadow: 0 0 16px rgba(217, 119, 6, 0.5);
    }

    .btn-mode-select.active.off {
      background: linear-gradient(135deg, #991b1b, #dc2626);
      color: #ffffff;
      border-color: #ef4444;
      box-shadow: 0 0 16px rgba(220, 38, 38, 0.5);
    }

    .manual-control-card {
      background: linear-gradient(135deg, rgba(30, 41, 59, 0.95), rgba(15, 23, 42, 0.95));
      border: 1px solid #f59e0b;
      border-radius: 20px;
      padding: 18px 24px;
      display: flex;
      flex-direction: column;
      gap: 12px;
      box-shadow: 0 12px 30px rgba(0, 0, 0, 0.4), 0 0 20px rgba(245, 158, 11, 0.2);
      width: 100%;
      animation: fadeIn 0.25s ease-out;
    }

    .manual-card-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .manual-card-title {
      font-size: 0.95rem;
      font-weight: 800;
      color: #fbbf24;
    }

    .manual-status-indicator {
      font-size: 0.8rem;
      color: #94a3b8;
    }

    .manual-status-indicator strong {
      padding: 3px 8px;
      border-radius: 8px;
      font-weight: 800;
    }

    .manual-card-desc {
      font-size: 0.8rem;
      color: #cbd5e1;
      line-height: 1.4;
    }

    .manual-actions-row {
      display: flex;
      gap: 14px;
    }

    .btn-manual-action {
      flex: 1;
      padding: 14px;
      border-radius: 14px;
      border: none;
      font-size: 0.95rem;
      font-weight: 800;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      transition: all 0.2s ease;
    }

    .btn-manual-action.on {
      background: #16a34a;
      color: white;
      box-shadow: 0 4px 14px rgba(22, 163, 74, 0.4);
    }
    .btn-manual-action.on:hover {
      background: #15803d;
      transform: translateY(-2px);
    }

    .btn-manual-action.off {
      background: #dc2626;
      color: white;
      box-shadow: 0 4px 14px rgba(220, 38, 38, 0.4);
    }
    .btn-manual-action.off:hover {
      background: #b91c1c;
      transform: translateY(-2px);
    }

    /* RESPONSIVE PARA NOTEBOOKS Y MONITORES GRANDES */
    @media (max-width: 1200px) {
      .hero-panel {
        grid-template-columns: 1fr;
      }
      .bottom-cards-grid {
        grid-template-columns: 1fr;
      }
      .mode-strip-left {
        flex-direction: column;
        align-items: flex-start;
      }
    }

    /* SLIDER INTERACTIVO DE TEMPERATURA Y PRESETS */
    .interactive-temp-slider {
      width: 100%;
      height: 14px;
      -webkit-appearance: none;
      appearance: none;
      background: linear-gradient(90deg, #38bdf8 0%, #10b981 40%, #f59e0b 70%, #ef4444 100%);
      border-radius: 10px;
      outline: none;
      cursor: pointer;
      margin: 10px 0 6px 0;
      box-shadow: inset 0 1px 4px rgba(0,0,0,0.3);
    }
    .interactive-temp-slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 26px;
      height: 26px;
      border-radius: 50%;
      background: #ffffff;
      border: 3px solid #0f172a;
      box-shadow: 0 2px 8px rgba(0,0,0,0.35);
      cursor: grab;
      transition: transform 0.12s, border-color 0.12s;
    }
    .interactive-temp-slider::-webkit-slider-thumb:hover {
      transform: scale(1.18);
      background: #38bdf8;
      border-color: #ffffff;
    }
    .quick-preset-btns {
      display: flex;
      gap: 6px;
      justify-content: space-between;
      margin: 10px 0 14px 0;
    }
    .btn-preset {
      flex: 1;
      padding: 6px 0;
      background: #ffffff;
      border: 1px solid #cbd5e1;
      border-radius: 12px;
      font-size: 0.76rem;
      font-weight: 700;
      color: #334155;
      cursor: pointer;
      transition: all 0.15s;
      text-align: center;
    }
    .btn-preset:hover {
      background: #0f172a;
      color: #ffffff;
      border-color: #0f172a;
    }
    .btn-preset.active {
      background: #38bdf8;
      color: #ffffff;
      border-color: #38bdf8;
      box-shadow: 0 2px 8px rgba(56, 189, 248, 0.4);
    }
    .target-quick-adjust-group {
      display: flex;
      align-items: center;
      gap: 8px;
    }

  </style>
</head>
<body>

  <div class="dashboard-container">

    <!-- BARRA SUPERIOR (HEADER) -->
    <header class="desktop-header">
      <div class="brand-section">
        <div class="brand-icon">
          <div class="brand-icon-inner"><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="#f97316" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M12 2c1 4 5 6 5 11a5 5 0 0 1-10 0c0-3 2-4 2-7 2 1 3 3 3 5 1-1 1-3 0-9z"/></svg></div>
        </div>
        <div class="brand-name">
          Lonchera<span>Control 4.0</span> <span style="background: #10b981; color: #ffffff; font-size: 0.75rem; font-weight: 900; padding: 2px 8px; border-radius: 8px; margin-left: 6px; box-shadow: 0 0 12px rgba(16, 185, 129, 0.7); letter-spacing: 0.5px;">GG</span>
        </div>
      </div>

      <!-- PÍLDORA CENTRAL DE ESTADO -->
      <div class="top-status-pill">
        <div class="pill-metric">
          <span>Temp Actual</span>
          <span class="val-big" id="pillTempActual">--.-°C</span>
        </div>
        <div style="width: 1px; height: 24px; background: rgba(255,255,255,0.15);"></div>
        <div class="pill-metric">
          <span>Objetivo</span>
          <span class="val-big" id="pillTempObjetivo">40.0°C</span>
        </div>
        <div class="pill-text-status" id="pillStatusMsg">
          Calentando interior con <strong>Secadora (ALTO)</strong>
        </div>
      </div>

      <!-- ACCIONES DERECHAS -->
      <div class="header-actions">
        <button class="mode-switch-btn" id="btnModeSim" onclick="toggleSimMode()">
          <span id="simIcon"><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><rect x="3" y="6" width="18" height="12" rx="4"/><path d="M8 10v4M6 12h4"/><circle cx="16" cy="12" r="1"/></svg></span>
          <span id="simLabel">Modo Piloto</span>
        </button>
        <div class="avatar-circle"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M13 2 4 14h7l-1 8 9-12h-7z"/></svg></div>
      </div>
    </header>

    <!-- ======================================================== -->
    <!-- VISTA 1: DASHBOARD PRINCIPAL EN ESCRITORIO               -->
    <!-- ======================================================== -->
    <main class="view-container active" id="viewMainDashboard">

      <!-- HERO PANEL AZUL WIDESCREEN (EXACTO A LA IMAGEN) -->
      <section class="hero-panel">

        <!-- COLUMNA IZQUIERDA: Radar de Balance Térmico -->
        <div class="hero-col-left">
          <div>
            <div class="hero-title-wrap">
              <span>Reporte Térmico</span>
              <span class="badge-suboptimal heating" id="heroBadgeStatus">Subóptimo</span>
            </div>
            <div class="hero-desc">
              Medición en tiempo real del calor interno, inercia térmica y disipación de la lonchera metálica.
            </div>
          </div>

          <div class="radar-wrapper">
            <canvas id="radarCanvas"></canvas>
          </div>
        </div>

        <!-- COLUMNA CENTRAL: Gran Arco con Nodos y Animación de Rango -->
        <div class="hero-col-center">
          <div class="center-gauge-box">

            <svg class="main-arc-svg" viewBox="0 0 460 260">
              <defs>
                <linearGradient id="arcGradientFlow" x1="0%" y1="0%" x2="100%" y2="0%">
                  <stop offset="0%" stop-color="#38bdf8" />
                  <stop offset="50%" stop-color="#10b981" />
                  <stop offset="85%" stop-color="#f59e0b" />
                  <stop offset="100%" stop-color="#ef4444" />
                </linearGradient>
              </defs>

              <!-- Órbita exterior discontinua con puntos -->
              <path class="arc-bg-dashed" d="M 40 230 A 190 190 0 0 1 420 230" />

              <!-- Pista gruesa del arco -->
              <path class="arc-track-thick" d="M 60 230 A 170 170 0 0 1 400 230" />

              <!-- Resaltado de la zona de mantenimiento programada -->
              <path class="arc-range-highlight" id="arcTargetHighlight" d="M 60 230 A 170 170 0 0 1 400 230" />

              <!-- Relleno dinámico de temperatura actual -->
              <path class="arc-fill-gradient" id="arcFillDynamic" d="M 60 230 A 170 170 0 0 1 400 230" />
            </svg>

            <!-- Puntos de referencia orbitales (como en la imagen: 4.94% Cancer, Stroke, etc.) -->
            <div class="orbit-point" style="left: 45px; top: 110px;">
              <span class="orbit-val" id="orbitMinTemp">20.0°C</span>
              <div class="orbit-dot"></div>
              <span>Ambiente</span>
            </div>

            <div class="orbit-point" style="left: 115px; top: 38px;">
              <span class="orbit-val" id="orbitRangeMin">38.0°C</span>
              <div class="orbit-dot" style="background:#10b981; box-shadow:0 0 8px #10b981;"></div>
              <span>Rango Mín</span>
            </div>

            <div class="orbit-point" style="left: 215px; top: 10px;">
              <span class="orbit-val" id="orbitSetpoint">40.0°C</span>
              <div class="orbit-dot" style="background:#38bdf8; box-shadow:0 0 8px #38bdf8;"></div>
              <span>Setpoint</span>
            </div>

            <div class="orbit-point" style="right: 115px; top: 38px;">
              <span class="orbit-val" id="orbitRangeMax">42.0°C</span>
              <div class="orbit-dot" style="background:#10b981; box-shadow:0 0 8px #10b981;"></div>
              <span>Rango Máx</span>
            </div>

            <div class="orbit-point" style="right: 45px; top: 110px;">
              <span class="orbit-val">65.0°C</span>
              <div class="orbit-dot" style="background:#ef4444; box-shadow:0 0 8px #ef4444;"></div>
              <span>Límite Seguro</span>
            </div>

            <!-- CONTENIDO CENTRAL DEL GAUGE -->
            <div class="gauge-center-content">
              <div class="gauge-temperature-big">
                <span id="centerTempVal">--.-</span><span class="unit">°C</span>
              </div>
              <div class="gauge-caption" id="centerTempSubtitle">Temperatura Interior Lonchera</div>

              <!-- ANIMACIÓN VIVA DE MANTENIMIENTO EN RANGO -->
              <div class="range-pulse-indicator heating-pulse" id="rangePulseBadge">
                <span id="rangePulseDot">●</span>
                <span id="rangePulseText">Calentando hacia el rango...</span>
              </div>
            </div>

          </div>
        </div>

        <!-- COLUMNA DERECHA: Mini Gráfico y Botón Ver Historial Completo -->
        <div class="hero-col-right">
          <div class="mini-stat-card">
            <div class="mini-stat-top">
              <div class="mini-stat-val" id="statPowerTopVal">100%</div>
              <div class="mini-stat-tag" id="statPowerTag">ALTO</div>
            </div>
            <span style="font-size:0.75rem; color:#94a3b8;">Potencia Secadora (TRIAC)</span>
            <div class="mini-stat-bar">
              <div class="mini-stat-fill" id="statPowerBarFill"></div>
            </div>
          </div>

          <div>
            <div style="font-size:0.78rem; font-weight:700; color:#cbd5e1; margin-bottom:4px;">Tendencia Reciente</div>
            <div class="mini-chart-container">
              <canvas id="previewCanvas"></canvas>
            </div>
          </div>

          <button class="btn-view-full-report" onclick="navigateToHistory()">
            <span>Ver historial completo</span>
            <span>&gt;</span>
          </button>
        </div>

      </section>

      <!-- ======================================================== -->
      <!-- FRANJA DE MODOS: AUTOMÁTICO VS MANUAL DE PRUEBA          -->
      <!-- ======================================================== -->
      <div class="mode-control-strip">
        <div class="mode-strip-left">
          <span class="mode-strip-label">MODO DE OPERACIÓN:</span>
          <div class="mode-buttons-group">
            <button class="btn-mode-select active auto" id="btnModeAuto" onclick="cambiarModoOperacion('auto')">
              <span>Automático (Autorregulación)</span>
            </button>
            <button class="btn-mode-select manual" id="btnModeManual" onclick="cambiarModoOperacion('manual')">
              <span>Manual (Prueba Secadora)</span>
            </button>
            <button class="btn-mode-select off" id="btnModeOff" onclick="cambiarModoOperacion('off')">
              <span>Apagar</span>
            </button>
          </div>
        </div>

        <div class="mode-strip-right" id="modeStripInfo">
          <span style="font-size: 0.8rem; font-weight: 700; color: #38bdf8;" id="badgeModoActivo">● Modo Automático Activo (ESP32 regulando solo)</span>
        </div>
      </div>

      <!-- SUB-PANEL DE CONTROL MANUAL (Solo visible en Modo Manual) -->
      <div class="manual-control-card" id="manualControlCard" style="display: none;">
        <div class="manual-card-header">
          <div class="manual-card-title">
            <span>Panel de Pruebas Manual (Verificar Circuito, TRIAC y Secadora)</span>
          </div>
          <div class="manual-status-indicator">
            Estado TRIAC: <strong id="manualTriacText" style="background: #334155; color: #94a3b8;">APAGADO</strong>
          </div>
        </div>
        <p class="manual-card-desc">
          Prueba directamente si el ESP32 conmuta el optoacoplador, si el TRIAC dispara y si la secadora enciende y expulsa aire caliente dentro de la lonchera, sin importar la temperatura.
        </p>
        <div class="manual-actions-row">
          <button class="btn-manual-action on" id="btnManualOn" onclick="enviarAccionManual(true)">
            <span>ENCENDER SECADORA (ON)</span>
          </button>
          <button class="btn-manual-action off" id="btnManualOff" onclick="enviarAccionManual(false)">
            <span>APAGAR SECADORA (OFF)</span>
          </button>
        </div>
      </div>

      <!-- ======================================================== -->
      <!-- FILA INFERIOR: 3 TARJETAS BLANCAS MODERNAS (COMO LA IMAGEN)-->
      <!-- ======================================================== -->
      <section class="bottom-cards-grid">

        <!-- TARJETA 1 (IZQUIERDA): RANGO TÉRMICO EDITABLE & SETPOINT -->
        <div class="white-card">
          <div>
            <div class="white-card-header">
              <div>
                <div class="white-card-title">Rango Térmico Programable</div>
                <div class="white-card-subtitle">
                  Define el setpoint y la ventana de tolerancia para mantener el calor óptimo en la lonchera.
                </div>
              </div>
            </div>

            <!-- Score y objetivo con botones directos + y - -->
            <div class="score-comparison-row">
              <div class="target-quick-adjust-group">
                <button class="btn-step-circle" onclick="changeTarget(-1)" title="Disminuir 1°C" style="width:36px; height:36px; font-size:1.3rem;">−</button>
                <div class="score-big-number"><span id="targetDisplayBig">40.0</span>°C</div>
                <button class="btn-step-circle" onclick="changeTarget(1)" title="Aumentar 1°C" style="width:36px; height:36px; font-size:1.3rem;">+</button>
              </div>
              <div class="score-sub-label">Objetivo / <span id="actualDisplaySub">--.-</span>°C Actual</div>
              <div class="score-badge-status" id="rangeBadgeText">Ajustando</div>
            </div>

            <!-- SLIDER INTERACTIVO DIRECTO (ARRASTRA O HAZ CLIC PARA CAMBIAR TEMPERATURA) -->
            <div style="margin-top: 10px;">
              <input type="range" class="interactive-temp-slider" id="targetRangeSlider" min="20" max="65" step="0.5" value="40" oninput="onTargetSliderInput(this.value)" onchange="onTargetSliderChange(this.value)">
              <div class="gradient-labels">
                <span>Frío (20°C)</span>
                <span>Objetivo: <strong id="sliderTargetEcho" style="color:#0f172a; font-weight:800;">40.0°C</strong></span>
                <span>Máx Seguro (65°C)</span>
              </div>
            </div>

            <!-- PRESETS DE UN SOLO CLIC -->
            <div class="quick-preset-btns">
              <button type="button" class="btn-preset" onclick="setTargetExact(30)">30°C</button>
              <button type="button" class="btn-preset" onclick="setTargetExact(35)">35°C</button>
              <button type="button" class="btn-preset active" id="btnPreset40" onclick="setTargetExact(40)">40°C</button>
              <button type="button" class="btn-preset" onclick="setTargetExact(45)">45°C</button>
              <button type="button" class="btn-preset" onclick="setTargetExact(50)">50°C</button>
            </div>
          </div>

          <!-- BOTONES STEPPER PARA EDITAR DESDE EL DASHBOARD -->
          <div class="controls-stepper-grid">
            <div class="stepper-item">
              <span class="stepper-label">Temperatura Objetivo</span>
              <div class="stepper-actions">
                <button class="btn-step-circle" onclick="changeTarget(-1)">−</button>
                <div class="stepper-val-text"><span id="stepperTargetVal">40.0</span>°C</div>
                <button class="btn-step-circle" onclick="changeTarget(1)">+</button>
              </div>
            </div>

            <div class="stepper-item">
              <span class="stepper-label">Margen de Rango (±)</span>
              <div class="stepper-actions">
                <button class="btn-step-circle" onclick="changeTolerance(-0.5)">−</button>
                <div class="stepper-val-text">±<span id="stepperTolVal">2.0</span>°C</div>
                <button class="btn-step-circle" onclick="changeTolerance(0.5)">+</button>
              </div>
            </div>
          </div>
        </div>

        <!-- TARJETA 2 (CENTRO): REGULADOR DE SECADORA (ALTO, MEDIO, BAJO) -->
        <div class="white-card">
          <div>
            <div class="white-card-header">
              <div>
                <div class="white-card-title">Regulador de Secadora</div>
                <div class="white-card-subtitle">
                  Control de potencia y ventilador térmico inyectado a las paredes metálicas de la lonchera.
                </div>
              </div>
            </div>

            <div class="immunity-percentage-row" style="margin-top: 10px;">
              <div class="immunity-big-perc" id="powerPercBig">100%</div>
              <div style="font-size:0.85rem; color:var(--text-dark-muted); font-weight:600;">Potencia Seleccionada</div>
            </div>
          </div>

          <!-- LISTA DE BOTONES ALTO, MEDIO, BAJO -->
          <div class="power-level-list">
            <div class="power-level-card-item selected" id="powerItemHigh" onclick="selectPowerLevel('high')">
              <div class="power-name">
                <span><svg width="16" height="16" viewBox="0 0 24 24" aria-hidden="true"><rect x="3" y="16" width="4" height="5" rx="1" fill="currentColor" stroke="currentColor" stroke-width="1.5"/><rect x="10" y="11" width="4" height="10" rx="1" fill="currentColor" stroke="currentColor" stroke-width="1.5"/><rect x="17" y="6" width="4" height="15" rx="1" fill="currentColor" stroke="currentColor" stroke-width="1.5"/></svg></span>
                <span>Nivel Alto (Calentamiento Rápido)</span>
              </div>
              <div class="power-spec">100% Continuo</div>
            </div>

            <div class="power-level-card-item" id="powerItemMed" onclick="selectPowerLevel('med')">
              <div class="power-name">
                <span><svg width="16" height="16" viewBox="0 0 24 24" aria-hidden="true"><rect x="3" y="16" width="4" height="5" rx="1" fill="currentColor" stroke="currentColor" stroke-width="1.5"/><rect x="10" y="11" width="4" height="10" rx="1" fill="currentColor" stroke="currentColor" stroke-width="1.5"/><rect x="17" y="6" width="4" height="15" rx="1" fill="none" stroke="currentColor" stroke-width="1.5"/></svg></span>
                <span>Nivel Medio (Calor Equilibrado)</span>
              </div>
              <div class="power-spec">66% Ráfagas</div>
            </div>

            <div class="power-level-card-item" id="powerItemLow" onclick="selectPowerLevel('low')">
              <div class="power-name">
                <span><svg width="16" height="16" viewBox="0 0 24 24" aria-hidden="true"><rect x="3" y="16" width="4" height="5" rx="1" fill="currentColor" stroke="currentColor" stroke-width="1.5"/><rect x="10" y="11" width="4" height="10" rx="1" fill="none" stroke="currentColor" stroke-width="1.5"/><rect x="17" y="6" width="4" height="15" rx="1" fill="none" stroke="currentColor" stroke-width="1.5"/></svg></span>
                <span>Nivel Bajo (Mantenimiento Suave)</span>
              </div>
              <div class="power-spec">33% Pulso</div>
            </div>
          </div>
        </div>

        <!-- TARJETA 3 (DERECHA): TIEMPOS DE PROCESO & RETENCIÓN DE CALOR -->
        <div class="white-card">
          <div>
            <div class="white-card-header">
              <div>
                <div class="white-card-title">Tiempos & Retención Térmica</div>
                <div class="white-card-subtitle">
                  Supervisión de tiempos de inyección de calor, enfriamiento y aislamiento de la lonchera.
                </div>
              </div>
            </div>

            <!-- Tiempos numéricos destacados -->
            <div class="inflammation-stats-row" style="margin-top: 10px;">
              <div class="timer-stat-box">
                <div class="title">Tiempo Calentado</div>
                <div class="val" style="color: #f97316;" id="timeHeatVal">0m 00s</div>
              </div>
              <div style="width: 1px; height: 35px; background: #e2e8f0;"></div>
              <div class="timer-stat-box">
                <div class="title">Tiempo Enfriado</div>
                <div class="val" style="color: #0284c7;" id="timeCoolVal">0m 00s</div>
              </div>
              <div style="width: 1px; height: 35px; background: #e2e8f0;"></div>
              <div class="timer-stat-box">
                <div class="title">Tiempo Estable</div>
                <div class="val" style="color: #10b981;" id="timeStableVal">0m 00s</div>
              </div>
            </div>

            <!-- Caja de explicación con borde azul cian -->
            <div class="explanation-callout">
              El tiempo de enfriado evalúa cuántos minutos la lonchera metálica retiene el calor antes de que la secadora deba activarse nuevamente.
            </div>
          </div>

          <div>
            <div class="change-over-time-row">
              <span>Tasa Térmica en Vivo</span>
              <span class="change-badge-up" id="rateBadgeText">▲ +1.5 °C/min</span>
            </div>

            <div class="bullet-diagnostics" style="margin-top: 8px;">
              <div><span>▸</span> <span>TRIAC controlado con cruce por cero (MOC3041)</span></div>
              <div><span>▸</span> <span id="diagLunchboxText">Lonchera metálica absorbiendo calor eficientemente</span></div>
            </div>
          </div>
        </div>

      </section>

    </main>

    <!-- ======================================================== -->
    <!-- VISTA 2: HISTORIAL COMPLETO EN OTRA PÁGINA               -->
    <!-- ======================================================== -->
    <main class="view-container" id="viewHistoryPage">
      <section class="history-widescreen-panel">

        <div class="history-header-bar">
          <div>
            <h2 style="font-size: 1.6rem; font-weight:800; color:#ffffff;">Historial Térmico & Modelado Teórico</h2>
            <p style="font-size: 0.85rem; color: #94a3b8; margin-top:2px;">
              Supervisión experimental, modelo matemático de transferencia de calor y picos de modulación de potencia.
            </p>
          </div>
          <button class="btn-back-dashboard" onclick="navigateToDashboard()">
            <span>← Volver al Dashboard</span>
          </button>
        </div>

        <!-- BARRA DE PESTAÑAS Y EXPORTACIÓN DEL HISTORIAL -->
        <div class="history-tabs-row">
          <div class="hist-tab-group">
            <button class="btn-hist-subtab active" id="btnHistTabCurves" onclick="switchHistSubtab('curves')">
              <span>Curva Real vs Teórica</span>
            </button>
            <button class="btn-hist-subtab" id="btnHistTabPulses" onclick="switchHistSubtab('pulses')">
              <span>Picos de Potencia & Pulsos TRIAC</span>
            </button>
            <button class="btn-hist-subtab" id="btnHistTabEssays" onclick="switchHistSubtab('essays')">
              <span>Ensayos Guardados (<span id="countEnsayos">0</span>)</span>
            </button>
          </div>
          <button class="btn-export-csv" onclick="exportarHistorialCSV()">
            <span>Exportar CSV (Excel / Lab)</span>
          </button>
        </div>

        <!-- 4 Tarjetas de Estadísticas Widescreen -->
        <div class="history-metrics-four">
          <div class="metric-glass-box">
            <div class="metric-glass-label">Temperatura Máxima (Pico)</div>
            <div class="metric-glass-val" style="color: #f87171;" id="histMaxTemp">--.-°C</div>
          </div>
          <div class="metric-glass-box">
            <div class="metric-glass-label">Temperatura Mínima</div>
            <div class="metric-glass-val" style="color: #38bdf8;" id="histMinTemp">--.-°C</div>
          </div>
          <div class="metric-glass-box">
            <div class="metric-glass-label">Temperatura Promedio</div>
            <div class="metric-glass-val" style="color: #a78bfa;" id="histAvgTemp">--.-°C</div>
          </div>
          <div class="metric-glass-box">
            <div class="metric-glass-label">Total Ciclos TRIAC</div>
            <div class="metric-glass-val" style="color: #fb923c;" id="histTriacCycles">0</div>
          </div>
        </div>

        <!-- CAJA DE MODELADO TEÓRICO (FUNDAMENTO ACADÉMICO PARA EL PROFESOR) -->
        <div class="theory-box" id="theoryBoxWrap">
          <div class="theory-box-header">
            <span>MODELO MATEMÁTICO TÉRMICO (RESPUESTA AL ESCALÓN)</span>
            <div class="theory-formula-chip">
              T(t) = Tamb + (Tobj - Tamb)·(1 - e<sup>-t/τ</sup>)
            </div>
          </div>
          <div class="theory-params-grid">
            <div class="theory-param-item">
              <span>Constante de Tiempo (τ)</span>
              <strong id="theoryTauVal">62.5 s</strong>
            </div>
            <div class="theory-param-item">
              <span>Potencia Secadora (P)</span>
              <strong>850 W (Nominal)</strong>
            </div>
            <div class="theory-param-item">
              <span>Duty Cycle Teórico Mantenimiento</span>
              <strong style="color: #38bdf8;" id="theoryDutyVal">41.8%</strong>
            </div>
            <div class="theory-param-item">
              <span>Picos de Pulsos Registrados</span>
              <strong style="color: #f59e0b;" id="theoryPulsosVal">0 pulsos</strong>
            </div>
          </div>
        </div>

        <!-- SECCIÓN 1: GRÁFICO PRINCIPAL DE TEMPERATURA REAL VS TEÓRICA -->
        <div id="sectionHistCurves" style="display: flex; flex-direction: column; gap: 10px;">
          <div style="display: flex; justify-content: space-between; align-items: center; font-size: 0.76rem; color: #94a3b8;">
            <div style="display: flex; gap: 14px;">
              <span><strong style="color: #f97316;">Línea Naranja:</strong> Temp Real Medida (LM35)</span>
              <span><strong style="color: #38bdf8;">Línea Cian Punteada:</strong> Curva Teórica Ideal (1er Orden)</span>
              <span><strong style="color: #10b981;">Línea Verde:</strong> Setpoint (<span id="histSetpointLegend">40.0°C</span>)</span>
            </div>
            <span id="histPuntosTotal">0 muestras</span>
          </div>

          <div class="history-chart-large-wrap">
            <canvas id="largeHistoryChart"></canvas>
          </div>
        </div>

        <!-- SECCIÓN 2: GRÁFICO DE PICOS DE POTENCIA Y PULSOS DEL TRIAC -->
        <div id="sectionHistPulses" style="display: none; flex-direction: column; gap: 10px;">
          <div style="display: flex; justify-content: space-between; align-items: center; font-size: 0.76rem; color: #94a3b8;">
            <span><strong style="color: #f59e0b;">Tren de Pulsos de Potencia:</strong> Modulación de ráfagas en cruce por cero para mantener la estabilidad</span>
            <span id="pulsesStatSummary">0 W promedio</span>
          </div>
          <div class="pulses-chart-wrap" style="height: 280px;">
            <canvas id="pulsesChart"></canvas>
          </div>
        </div>

        <!-- SECCIÓN 3: ENSAYOS GUARDADOS AUTOMÁTICAMENTE AL CAMBIAR SETPOINT -->
        <div id="sectionHistEssays" style="display: none; flex-direction: column; gap: 10px;">
          <div style="display: flex; justify-content: space-between; align-items: center;">
            <span style="font-size: 0.85rem; font-weight: 700; color: #ffffff;">
              Historial de Ensayos Térmicos (Guardados al cambiar temperatura)
            </span>
            <span style="font-size: 0.75rem; color: #94a3b8;">Haz clic en un ensayo para cargarlo en la gráfica</span>
          </div>
          <div class="saved-essays-grid" id="savedEssaysGrid">
            <div style="font-size: 0.8rem; color: #64748b; padding: 10px;">No hay ensayos guardados aún. Al cambiar la temperatura objetivo se registrará automáticamente.</div>
          </div>
        </div>

        <!-- Bitácora de Eventos -->
        <div>
          <div style="font-size: 0.85rem; font-weight:700; color:#ffffff; margin-bottom: 8px;">
            Bitácora de Eventos de la Lonchera
          </div>
          <div class="history-log-table" id="historyLogContainer">
            <div class="log-row-item">
              <span class="log-row-time">[00:00:00]</span>
              <span class="log-row-msg">Iniciando supervisión del sistema</span>
            </div>
          </div>
        </div>

      </section>
    </main>

  </div>

  <script>
    // ESTADO DEL SISTEMA
    let simMode = false;
    let actualTemp = 24.0;
    let targetTemp = 40.0;
    let toleranceVal = 2.0;
    let powerLevel = 'high'; // 'low', 'med', 'high'
    let triacActive = false;
    let currentModo = 'auto'; // 'auto', 'manual', 'off'
    let manualState = false;  // true = ON, false = OFF en modo manual
    let faseESP = '';         // 'NORMAL', 'RAFAGA' o 'ESPERA' (modo automatico del ESP32)

    // TIEMPOS (en segundos)
    let secondsHeat = 0;
    let secondsCool = 0;
    let secondsStable = 0;
    let triacCycles = 0;
    let triacPrev = false;

    // HISTORIAL Y ENLACE ACADÉMICO / TEÓRICO
    const MAX_POINTS = 80;
    let tempHistory = [];
    let powerPulsesHistory = []; // Almacena el tren de pulsos del TRIAC
    let savedEssays = [];        // Ensayos guardados automáticamente por setpoint
    let picoMax = null;
    let picoMin = null;
    let lastTempForRate = 24.0;
    let histSubtab = 'curves';
    let stepStartTime = Date.now();
    let initialTempOfStep = 24.0;
    let pulsesCountInStep = 0;

    // CANVAS ELEMENTS
    const radarCanvas = document.getElementById('radarCanvas');
    const previewCanvas = document.getElementById('previewCanvas');
    const largeCanvas = document.getElementById('largeHistoryChart');
    const pulsesCanvas = document.getElementById('pulsesChart');

    // NAVEGACIÓN DE SUB-PESTAÑAS EN EL HISTORIAL (CURVAS / PULSOS / ENSAYOS)
    function switchHistSubtab(sub) {
      histSubtab = sub;
      const bCurves = document.getElementById('btnHistTabCurves');
      const bPulses = document.getElementById('btnHistTabPulses');
      const bEssays = document.getElementById('btnHistTabEssays');

      const secCurves = document.getElementById('sectionHistCurves');
      const secPulses = document.getElementById('sectionHistPulses');
      const secEssays = document.getElementById('sectionHistEssays');

      bCurves.className = 'btn-hist-subtab' + (sub === 'curves' ? ' active' : '');
      bPulses.className = 'btn-hist-subtab' + (sub === 'pulses' ? ' active' : '');
      bEssays.className = 'btn-hist-subtab' + (sub === 'essays' ? ' active' : '');

      secCurves.style.display = (sub === 'curves') ? 'flex' : 'none';
      secPulses.style.display = (sub === 'pulses') ? 'flex' : 'none';
      secEssays.style.display = (sub === 'essays') ? 'flex' : 'none';

      if (sub === 'curves') {
        setTimeout(resizeLargeCanvas, 40);
      } else if (sub === 'pulses') {
        setTimeout(resizePulsesCanvas, 40);
      } else if (sub === 'essays') {
        renderSavedEssaysUI();
      }
    }

    // GESTIÓN DE ENSAYOS GUARDADOS AUTOMÁTICAMENTE
    function guardarEnsayoActual(setpointPrevio) {
      if (tempHistory.length < 5) return;
      const now = new Date();
      const horaStr = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
      const duracionSec = Math.round((Date.now() - stepStartTime) / 1000);

      const nuevoEnsayo = {
        id: Date.now(),
        numero: savedEssays.length + 1,
        hora: horaStr,
        setpoint: setpointPrevio,
        tempInicial: initialTempOfStep.toFixed(1),
        tempFinal: actualTemp.toFixed(1),
        duracionSec: duracionSec,
        pulsosTotales: pulsesCountInStep,
        datosTemp: [...tempHistory],
        datosPower: [...powerPulsesHistory]
      };

      savedEssays.unshift(nuevoEnsayo);
      if (savedEssays.length > 25) savedEssays.pop();
      document.getElementById('countEnsayos').textContent = savedEssays.length;
      addLogEntry(`Ensayo #${nuevoEnsayo.numero} GUARDADO al cambiar setpoint a ${setpointPrevio.toFixed(1)}°C`);
      renderSavedEssaysUI();
    }

    function renderSavedEssaysUI() {
      const grid = document.getElementById('savedEssaysGrid');
      if (savedEssays.length === 0) {
        grid.innerHTML = '<div style="font-size: 0.8rem; color: #64748b; padding: 10px;">No hay ensayos guardados aún. Cada vez que cambies la temperatura objetivo con los botones se guardará un ensayo aquí con su gráfica.</div>';
        return;
      }
      grid.innerHTML = '';
      savedEssays.forEach((e) => {
        const item = document.createElement('div');
        item.className = 'essay-card-item';
        item.onclick = () => cargarEnsayo(e.id);
        item.innerHTML = `
          <div class="essay-top">
            <span class="essay-target">${e.setpoint.toFixed(1)}°C <span style="font-size:0.75rem; color:#38bdf8;">(Ensayo #${e.numero})</span></span>
            <span class="essay-time-tag">${e.hora}</span>
          </div>
          <div class="essay-meta-row">
            <span>Rango: ${e.tempInicial}°C ➔ ${e.tempFinal}°C</span>
            <span>${Math.floor(e.duracionSec/60)}m ${e.duracionSec%60}s</span>
          </div>
          <div class="essay-meta-row" style="color:#f59e0b;">
            <span>${e.pulsosTotales} pulsos de TRIAC</span>
            <span style="color:#10b981; font-weight:700;">Ver Gráfica ▸</span>
          </div>
        `;
        grid.appendChild(item);
      });
    }

    function cargarEnsayo(id) {
      const e = savedEssays.find(x => x.id === id);
      if (!e) return;
      tempHistory = [...e.datosTemp];
      powerPulsesHistory = [...e.datosPower];
      targetTemp = e.setpoint;
      switchHistSubtab('curves');
      updateUI();
      addLogEntry(`Cargando curva y datos del Ensayo #${e.numero}`);
    }

    // EXPORTACIÓN A CSV PARA EL PROFESOR / LABORATORIO
    function exportarHistorialCSV() {
      let csv = 'Muestra,Tiempo_s,Temp_Real_C,Temp_Teorica_C,Setpoint_C,TRIAC_Estado,Potencia_W\n';
      const TAU = 62.5;
      const T_AMB = 22.0;

      for (let i = 0; i < tempHistory.length; i++) {
        const tSec = i;
        const real = tempHistory[i].toFixed(2);
        const teorica = (T_AMB + (targetTemp - T_AMB) * (1.0 - Math.exp(-tSec / TAU))).toFixed(2);
        const pItem = powerPulsesHistory[i] || { active: false, power: 0 };
        const triac = pItem.active ? 'ENCENDIDO' : 'APAGADO';
        const watts = pItem.power || (pItem.active ? 850 : 0);
        csv += `${i + 1},${tSec},${real},${teorica},${targetTemp.toFixed(1)},${triac},${watts}\n`;
      }

      const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.setAttribute('href', url);
      link.setAttribute('download', `Horno_Termico_Ensayo_${targetTemp.toFixed(0)}C.csv`);
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      addLogEntry('Archivo CSV exportado exitosamente para análisis en Excel/MATLAB');
    }

    // CONTROL DE MODO DE OPERACIÓN (AUTOMÁTICO VS MANUAL DE PRUEBA)
    function cambiarModoOperacion(modo) {
      currentModo = modo;
      const bAuto = document.getElementById('btnModeAuto');
      const bManual = document.getElementById('btnModeManual');
      const bOff = document.getElementById('btnModeOff');
      const cardManual = document.getElementById('manualControlCard');
      const badgeInfo = document.getElementById('badgeModoActivo');

      bAuto.className = 'btn-mode-select' + (modo === 'auto' ? ' active auto' : '');
      bManual.className = 'btn-mode-select' + (modo === 'manual' ? ' active manual' : '');
      bOff.className = 'btn-mode-select' + (modo === 'off' ? ' active off' : '');

      if (modo === 'manual') {
        cardManual.style.display = 'flex';
        badgeInfo.innerHTML = '<span style="color:#f59e0b;">● Modo Manual de Pruebas (Tú controlas la secadora)</span>';
        addLogEntry('Modo de operación cambiado a: MANUAL (Prueba directa)');
        enviarModoESP32('manual_off');
        manualState = false;
        actualizarBotonManualUI(false);
      } else if (modo === 'auto') {
        cardManual.style.display = 'none';
        badgeInfo.innerHTML = '<span style="color:#38bdf8;">● Modo Automático Activo (ESP32 regulando solo)</span>';
        addLogEntry('Modo de operación cambiado a: AUTOMÁTICO (Autorregulación)');
        enviarModoESP32('auto');
      } else {
        cardManual.style.display = 'none';
        badgeInfo.innerHTML = '<span style="color:#ef4444;">● Sistema Apagado (Corte de Secadora)</span>';
        addLogEntry('Modo de operación cambiado a: APAGADO');
        enviarModoESP32('off');
        triacActive = false;
        manualState = false;
      }
    }

    function enviarAccionManual(encender) {
      manualState = encender;
      actualizarBotonManualUI(encender);
      if (encender) {
        enviarModoESP32('manual_on');
        addLogEntry('Prueba manual: Secadora ENCENDIDA (TRIAC ON)');
      } else {
        enviarModoESP32('manual_off');
        addLogEntry('Prueba manual: Secadora APAGADA (TRIAC OFF)');
      }
    }

    function actualizarBotonManualUI(encendido) {
      const txt = document.getElementById('manualTriacText');
      if (encendido) {
        txt.textContent = 'ENCENDIDO (Soplando calor)';
        txt.style.background = '#15803d';
        txt.style.color = '#ffffff';
        txt.style.boxShadow = '0 0 12px rgba(34, 197, 94, 0.5)';
      } else {
        txt.textContent = 'APAGADO';
        txt.style.background = '#334155';
        txt.style.color = '#94a3b8';
        txt.style.boxShadow = 'none';
      }
    }

    // NAVEGACIÓN ENTRE PÁGINAS (DASHBOARD vs HISTORIAL)
    function navigateToHistory() {
      document.getElementById('viewMainDashboard').classList.remove('active');
      document.getElementById('viewHistoryPage').classList.add('active');
      setTimeout(resizeLargeCanvas, 60);
    }

    function navigateToDashboard() {
      document.getElementById('viewHistoryPage').classList.remove('active');
      document.getElementById('viewMainDashboard').classList.add('active');
      setTimeout(() => { resizeRadar(); resizePreview(); }, 60);
    }

    // ICONOS SVG DEL BOTÓN DE MODO
    const ICON_SIM = '<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><rect x="3" y="6" width="18" height="12" rx="4"/><path d="M8 10v4M6 12h4"/><circle cx="16" cy="12" r="1"/></svg>';
    const ICON_ESP = '<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><path d="M5 12a10 10 0 0 1 14 0M8.5 15.5a5 5 0 0 1 7 0"/><circle cx="12" cy="19" r="1"/></svg>';

    // TOGGLE MODO PILOTO (SIMULACIÓN MANUAL)
    function toggleSimMode() {
      simMode = !simMode;
      const btn = document.getElementById('btnModeSim');
      const label = document.getElementById('simLabel');
      const icon = document.getElementById('simIcon');
      const pillMsg = document.getElementById('pillStatusMsg');

      if (simMode) {
        btn.classList.add('pilot-active');
        label.textContent = 'Simulación Activa';
        icon.innerHTML = ICON_SIM;
        pillMsg.innerHTML = '<span style="color:#c7d2fe;">Simulación activa (Física de Lonchera)</span>';
        addLogEntry('Modo Simulación ACTIVADO');
        updateLoop();
      } else {
        btn.classList.remove('pilot-active');
        label.textContent = 'Modo ESP32';
        icon.innerHTML = ICON_ESP;
        addLogEntry('Modo Simulación DETENIDO (Esperando ESP32)');
        setDisconnectedState();
      }
    }

    // ESTADO DESCONECTADO (Sin simulación y sin ESP32 detectado)
    function setDisconnectedState() {
      document.getElementById('pillTempActual').textContent = '--.-°C';
      document.getElementById('centerTempVal').textContent = '--.-';
      document.getElementById('actualDisplaySub').textContent = '--.-';
      document.getElementById('pillStatusMsg').innerHTML = '<span style="color:#ef4444;">● Sin conexión al ESP32</span> (Activa "Modo Piloto" si deseas simular)';

      const pulseBadge = document.getElementById('rangePulseBadge');
      pulseBadge.className = 'range-pulse-indicator';
      pulseBadge.innerHTML = '<span style="color:#ef4444;">●</span> <span>Esperando conexión ESP32...</span>';

      const heroBadge = document.getElementById('heroBadgeStatus');
      heroBadge.className = 'badge-suboptimal';
      heroBadge.textContent = 'Desconectado';

      const rangeBadge = document.getElementById('rangeBadgeText');
      rangeBadge.textContent = 'En Espera';
      rangeBadge.style.background = '#f1f5f9';
      rangeBadge.style.color = '#64748b';

      document.getElementById('arcFillDynamic').style.strokeDashoffset = 580;
      const pinDesc = document.getElementById('gradientSliderPin');
      if (pinDesc) pinDesc.style.left = '0%';
    }

    let ultimoAjusteTiempo = 0;
    let timerDebounceEnsayo = null;

    // AJUSTES DE OBJETIVO Y TOLERANCIA
    let sliderBeingDragged = false;

    function onTargetSliderInput(val) {
      sliderBeingDragged = true;
      targetTemp = Math.min(62.0, Math.max(20.0, parseFloat(val)));
      ultimoAjusteTiempo = Date.now();
      const echo = document.getElementById('sliderTargetEcho');
      if (echo) echo.textContent = targetTemp.toFixed(1) + '°C';
      updateUI();
    }

    function onTargetSliderChange(val) {
      sliderBeingDragged = false;
      targetTemp = Math.min(62.0, Math.max(20.0, parseFloat(val)));
      ultimoAjusteTiempo = Date.now();
      updateUI();
      syncWithESP32();
      if (timerDebounceEnsayo) clearTimeout(timerDebounceEnsayo);
      timerDebounceEnsayo = setTimeout(() => {
        guardarEnsayoActual(targetTemp);
        initialTempOfStep = actualTemp;
        stepStartTime = Date.now();
        pulsesCountInStep = 0;
      }, 2500);
    }

    function setTargetExact(val) {
      const nuevo = Math.min(62.0, Math.max(20.0, val));
      const previo = targetTemp;
      targetTemp = nuevo;
      ultimoAjusteTiempo = Date.now();
      updateUI();
      syncWithESP32();

      if (timerDebounceEnsayo) clearTimeout(timerDebounceEnsayo);
      timerDebounceEnsayo = setTimeout(() => {
        guardarEnsayoActual(previo);
        initialTempOfStep = actualTemp;
        stepStartTime = Date.now();
        pulsesCountInStep = 0;
      }, 2500);
    }

    function changeTarget(delta) {
      const nuevo = Math.min(62.0, Math.max(20.0, targetTemp + delta));
      if (nuevo !== targetTemp) {
        const previo = targetTemp;
        targetTemp = nuevo;
        ultimoAjusteTiempo = Date.now();
        updateUI();
        syncWithESP32();

        // Guardar ensayo solo cuando el usuario termine de hacer clics (2.5 segundos después)
        if (timerDebounceEnsayo) clearTimeout(timerDebounceEnsayo);
        timerDebounceEnsayo = setTimeout(() => {
          guardarEnsayoActual(previo);
          initialTempOfStep = actualTemp;
          stepStartTime = Date.now();
          pulsesCountInStep = 0;
        }, 2500);
      }
    }

    function changeTolerance(delta) {
      toleranceVal = Math.min(5.0, Math.max(0.5, toleranceVal + delta));
      ultimoAjusteTiempo = Date.now();
      updateUI();
      syncWithESP32();
    }

    // REGULADOR DE SECADORA: ALTO, MEDIO, BAJO
    function selectPowerLevel(lvl) {
      powerLevel = lvl;
      document.getElementById('powerItemHigh').className = 'power-level-card-item' + (lvl === 'high' ? ' selected' : '');
      document.getElementById('powerItemMed').className = 'power-level-card-item' + (lvl === 'med' ? ' selected' : '');
      document.getElementById('powerItemLow').className = 'power-level-card-item' + (lvl === 'low' ? ' selected' : '');

      const topVal = document.getElementById('statPowerTopVal');
      const topTag = document.getElementById('statPowerTag');
      const percBig = document.getElementById('powerPercBig');
      const barFill = document.getElementById('statPowerBarFill');

      if (lvl === 'high') {
        topVal.textContent = '100%';
        topTag.textContent = 'ALTO';
        topTag.style.background = '#ef4444';
        percBig.textContent = '100%';
        barFill.style.width = '100%';
        addLogEntry('Regulador secadora establecido en ALTO (100% continuo)');
      } else if (lvl === 'med') {
        topVal.textContent = '66%';
        topTag.textContent = 'MEDIO';
        topTag.style.background = '#f59e0b';
        percBig.textContent = '66%';
        barFill.style.width = '66%';
        addLogEntry('Regulador secadora establecido en MEDIO (66% pulsos)');
      } else {
        topVal.textContent = '33%';
        topTag.textContent = 'BAJO';
        topTag.style.background = '#38bdf8';
        percBig.textContent = '33%';
        barFill.style.width = '33%';
        addLogEntry('Regulador secadora establecido en BAJO (33% mantenimiento)');
      }

      syncWithESP32();
    }

    // DETECCIÓN DE RUTA DE CONEXIÓN AL ESP32
    // Si la web se abre desde el ESP32 usa '', si se abre como archivo local file:/// usa http://192.168.4.1
    const API_BASE = (window.location.protocol === 'http:' || window.location.protocol === 'https:') ? '' : 'http://192.168.4.1';

    async function syncWithESP32() {
      if (simMode) return;
      try {
        await fetch(`${API_BASE}/set?t=${targetTemp.toFixed(1)}&tol=${toleranceVal.toFixed(1)}&pot=${powerLevel}`);
      } catch (err) {}
    }

    async function enviarModoESP32(modoStr) {
      if (simMode) return;
      try {
        await fetch(`${API_BASE}/modo?m=${modoStr}`);
      } catch (err) {}
    }

    // SIMULACIÓN FÍSICA PARA MODO PILOTO EN COMPUTADORA
    let simStep = 0;
    function simulateThermalPhysics() {
      simStep++;
      const minR = targetTemp - toleranceVal;
      let heatFactor = 0.55;
      let dutyOn = true;

      if (powerLevel === 'low') {
        heatFactor = 0.22;
        dutyOn = (simStep % 3 === 0);
      } else if (powerLevel === 'med') {
        heatFactor = 0.38;
        dutyOn = (simStep % 3 !== 0);
      }

      if (currentModo === 'off') {
        triacActive = false;
      } else if (currentModo === 'manual') {
        triacActive = manualState;
      } else {
        // AUTOMÁTICO: Regula solo
        if (actualTemp < minR) {
          triacActive = dutyOn;
        } else if (actualTemp >= targetTemp) {
          triacActive = false;
        }
      }

      const coolingDissipation = (actualTemp - 22.0) * 0.024;
      if (triacActive) {
        actualTemp += heatFactor + (Math.random() * 0.06 - 0.03);
      } else {
        actualTemp -= coolingDissipation + (Math.random() * 0.02 - 0.01);
      }

      return {
        valido: true,
        temperatura: actualTemp,
        objetivo: targetTemp,
        tolerancia: toleranceVal,
        potencia: powerLevel,
        calefaccion: triacActive
      };
    }

    // BUCLE DE ACTUALIZACIÓN (SIN SOLAPAMIENTO DE PETICIONES)
    let isUpdating = false;
    let consecutiveErrors = 0;

    async function updateLoop() {
      if (isUpdating) return;
      isUpdating = true;
      try {
        await updateLoopInner();
      } catch (e) {
        console.error('Error actualizando la UI:', e);
      } finally {
        isUpdating = false;
      }
    }

    async function updateLoopInner() {
      let data = null;

      if (simMode) {
        data = simulateThermalPhysics();
      } else {
        try {
          const controller = new AbortController();
          const timeoutId = setTimeout(() => controller.abort(), 2600);
          const res = await fetch(API_BASE + '/estado', { signal: controller.signal });
          clearTimeout(timeoutId);
          data = await res.json();
        } catch (e) {
          data = null;
        }
      }

      if (!data) {
        if (!simMode) {
          consecutiveErrors++;
          if (consecutiveErrors < 3) {
            // Mantiene los datos en pantalla y muestra aviso de reconexión sutil
            const pText = document.getElementById('rangePulseText');
            if (pText) pText.innerHTML = `<span style="color:#f59e0b;">Reintentando conexión con ESP32 (${consecutiveErrors}/3)...</span>`;
          } else {
            setDisconnectedState();
          }
        }
        isUpdating = false;
        return;
      }
      consecutiveErrors = 0;

      if (data.temperatura !== null && data.temperatura !== undefined) {
        actualTemp = data.temperatura;
      }
      if (data.calefaccion !== undefined) {
        triacActive = data.calefaccion;
      }
      faseESP = (!simMode && data.fase) ? data.fase : '';

      if (!data.valido) {
        const pillMsg = document.getElementById('pillStatusMsg');
        if (pillMsg) pillMsg.innerHTML = '<span style="color:#f59e0b;">Lectura analógica baja en GPIO 34. Verificando sensor...</span>';
      }

      if (true) {

        if (!simMode) {
          // Solo sincronizar el objetivo si el usuario no ha tocado los botones en los últimos 3.5 segundos
          if (Date.now() - ultimoAjusteTiempo > 3500) {
            if (data.objetivo !== undefined) targetTemp = data.objetivo;
            if (data.tolerancia !== undefined) toleranceVal = data.tolerancia;
          }
          if (data.modo) {
            if (data.modo === 'MANUAL' && currentModo !== 'manual') {
              cambiarModoOperacion('manual');
            } else if (data.modo === 'AUTOMATICO' && currentModo !== 'auto') {
              cambiarModoOperacion('auto');
            } else if (data.modo === 'APAGADO' && currentModo !== 'off') {
              cambiarModoOperacion('off');
            }
          }
          if (currentModo === 'manual') {
            actualizarBotonManualUI(data.calefaccion);
          }
        }

        // Historial
        tempHistory.push(actualTemp);
        if (tempHistory.length > MAX_POINTS) tempHistory.shift();

        if (picoMax === null || actualTemp > picoMax) picoMax = actualTemp;
        if (picoMin === null || actualTemp < picoMin) picoMin = actualTemp;

        computeTimers(triacActive);
        updateUI();
        drawRadarChart();
        drawPreviewChart();
        if (document.getElementById('viewHistoryPage').classList.contains('active')) {
          drawLargeHistoryChart();
        }
      }
      isUpdating = false;
    }

    // COMPUTAR TIEMPOS Y PULSOS DE POTENCIA
    function computeTimers(isHeating) {
      const minR = targetTemp - toleranceVal;
      const maxR = targetTemp + toleranceVal;
      const inRange = (actualTemp >= minR && actualTemp <= maxR);

      if (isHeating) secondsHeat++;
      else secondsCool++;

      if (inRange) secondsStable++;

      // Registrar pulso de potencia en Watts
      const watts = isHeating ? (powerLevel === 'high' ? 850 : (powerLevel === 'med' ? 560 : 280)) : 0;
      powerPulsesHistory.push({ active: isHeating, power: watts });
      if (powerPulsesHistory.length > MAX_POINTS) powerPulsesHistory.shift();

      if (isHeating && !triacPrev) {
        triacCycles++;
        pulsesCountInStep++;
        const pEl = document.getElementById('theoryPulsosVal');
        if (pEl) pEl.textContent = `${triacCycles} pulsos`;
        addLogEntry(`TRIAC disparado con cruce por cero (Pulso #${triacCycles})`);
      } else if (!isHeating && triacPrev) {
        addLogEntry(`TRIAC apagado - Iniciando enfriamiento`);
      }
      triacPrev = isHeating;

      document.getElementById('timeHeatVal').textContent = formatSeconds(secondsHeat);
      document.getElementById('timeCoolVal').textContent = formatSeconds(secondsCool);
      document.getElementById('timeStableVal').textContent = formatSeconds(secondsStable);
      document.getElementById('histTriacCycles').textContent = triacCycles;
    }

    function formatSeconds(sec) {
      const m = Math.floor(sec / 60);
      const s = sec % 60;
      return `${m}m ${s < 10 ? '0' : ''}${s}s`;
    }

    // ACTUALIZAR ELEMENTOS VISUALES DEL DASHBOARD
    function updateUI() {
      const minR = targetTemp - toleranceVal;
      const maxR = targetTemp + toleranceVal;

      // Header y textos
      document.getElementById('pillTempActual').textContent = actualTemp.toFixed(1) + '°C';
      document.getElementById('pillTempObjetivo').textContent = targetTemp.toFixed(1) + '°C';
      document.getElementById('centerTempVal').textContent = actualTemp.toFixed(1);

      document.getElementById('targetDisplayBig').textContent = targetTemp.toFixed(1);
      document.getElementById('actualDisplaySub').textContent = actualTemp.toFixed(1);
      document.getElementById('stepperTargetVal').textContent = targetTemp.toFixed(1);
      document.getElementById('stepperTolVal').textContent = toleranceVal.toFixed(1);

      const sl = document.getElementById('targetRangeSlider');
      if (sl && !sliderBeingDragged) sl.value = targetTemp.toFixed(1);
      const echo = document.getElementById('sliderTargetEcho');
      if (echo) echo.textContent = targetTemp.toFixed(1) + '°C';

      // Resaltar preset activo
      [30, 35, 40, 45, 50].forEach(deg => {
        const btn = document.getElementById(`btnPreset${deg}`);
        if (btn) btn.classList.toggle('active', Math.abs(targetTemp - deg) < 0.25);
      });

      document.getElementById('orbitRangeMin').textContent = minR.toFixed(1) + '°C';
      document.getElementById('orbitRangeMax').textContent = maxR.toFixed(1) + '°C';
      document.getElementById('orbitSetpoint').textContent = targetTemp.toFixed(1) + '°C';

      // 1. Arco SVG Principal
      const TEMP_MIN = 20.0;
      const TEMP_MAX = 65.0;
      const TOTAL_DASH = 580;

      const toRatio = (t) => Math.max(0, Math.min(1, (t - TEMP_MIN) / (TEMP_MAX - TEMP_MIN)));
      const curRatio = toRatio(actualTemp);
      const offset = TOTAL_DASH * (1 - curRatio);
      document.getElementById('arcFillDynamic').style.strokeDashoffset = offset;

      // 2. Animación Central de Mantenimiento en Rango
      const pulseBadge = document.getElementById('rangePulseBadge');
      const pulseText = document.getElementById('rangePulseText');
      const heroBadge = document.getElementById('heroBadgeStatus');
      const rangeBadge = document.getElementById('rangeBadgeText');
      const pillMsg = document.getElementById('pillStatusMsg');

      if (actualTemp >= minR && actualTemp <= maxR) {
        // EN RANGO: ESTABILIDAD ÓPTIMA
        pulseBadge.className = 'range-pulse-indicator active-pulse';
        pulseText.textContent = (faseESP === 'RAFAGA')
          ? `MANTENIENDO EN RÁFAGA (±${toleranceVal.toFixed(1)}°C)`
          : `MANTENIENDO EN RANGO (±${toleranceVal.toFixed(1)}°C)`;
        heroBadge.className = 'badge-suboptimal stable';
        heroBadge.textContent = 'En Rango Estable';
        rangeBadge.textContent = 'Estable';
        rangeBadge.style.background = '#dcfce7';
        rangeBadge.style.color = '#15803d';
        pillMsg.innerHTML = (faseESP === 'RAFAGA')
          ? 'Manteniendo temperatura con <strong>ráfagas de la secadora</strong>'
          : 'Temperatura interior <strong>óptima y estable</strong>';
      } else if (actualTemp < minR) {
        // POR DEBAJO DEL OBJETIVO: CALENTANDO O DEMANDANDO CALOR
        pulseBadge.className = 'range-pulse-indicator heating-pulse';
        pulseText.textContent = (faseESP === 'NORMAL')
          ? `SUBIENDO EN POTENCIA NORMAL (HACIA ${targetTemp.toFixed(1)}°C)`
          : triacActive
          ? `CALENTANDO LONCHERA (${powerLevel.toUpperCase()})` 
          : `DEMANDANDO CALOR (HACIA ${targetTemp.toFixed(1)}°C)`;
        heroBadge.className = 'badge-suboptimal heating';
        heroBadge.textContent = 'Calentando';
        rangeBadge.textContent = 'Calentando';
        rangeBadge.style.background = '#ffedd5';
        rangeBadge.style.color = '#c2410c';
        pillMsg.innerHTML = (faseESP === 'NORMAL')
          ? 'Subiendo temperatura con <strong>Secadora (NORMAL)</strong>'
          : triacActive
          ? `Calentando interior con <strong>Secadora (${powerLevel.toUpperCase()})</strong>` 
          : `Por debajo del rango. <strong>Inyectando calor a la lonchera...</strong>`;
      } else {
        // POR ENCIMA DEL OBJETIVO: ENFRIANDO NATURALMENTE
        pulseBadge.className = 'range-pulse-indicator';
        pulseText.textContent = `ENFRIANDO HACIA EL RANGO`;
        heroBadge.className = 'badge-suboptimal cooling';
        heroBadge.textContent = 'Enfriando';
        rangeBadge.textContent = 'Enfriando';
        rangeBadge.style.background = '#e0f2fe';
        rangeBadge.style.color = '#0369a1';
        pillMsg.innerHTML = 'Disipando calor naturalmente por <strong>paredes metálicas</strong>';
      }

      // 3. Pin deslizante en la barra horizontal con degradado
      const sliderPin = document.getElementById('gradientSliderPin');
      if (sliderPin) sliderPin.style.left = `${curRatio * 100}%`;

      // 4. Estadísticas del Historial
      if (picoMax !== null) document.getElementById('histMaxTemp').textContent = picoMax.toFixed(1) + '°C';
      if (picoMin !== null) document.getElementById('histMinTemp').textContent = picoMin.toFixed(1) + '°C';
      if (tempHistory.length > 0) {
        const sum = tempHistory.reduce((a, b) => a + b, 0);
        document.getElementById('histAvgTemp').textContent = (sum / tempHistory.length).toFixed(1) + '°C';
      }

      // 5. Cálculos Teóricos para el Profesor
      const dutyTeorico = Math.max(0, Math.min(100, ((targetTemp - 22.0) / (65.0 - 22.0)) * 100));
      const elDuty = document.getElementById('theoryDutyVal');
      if (elDuty) elDuty.textContent = `${dutyTeorico.toFixed(1)}%`;
      const elPuntos = document.getElementById('histPuntosTotal');
      if (elPuntos) elPuntos.textContent = `${tempHistory.length} muestras`;
      const elLeg = document.getElementById('histSetpointLegend');
      if (elLeg) elLeg.textContent = `${targetTemp.toFixed(1)}°C`;
    }

    // DIBUJO DEL RADAR / PENTÁGONO TÉRMICO (EXACTO AL DE LA IMAGEN)
    function resizeRadar() {
      if (!radarCanvas.parentElement) return;
      radarCanvas.width = radarCanvas.parentElement.clientWidth || 240;
      radarCanvas.height = radarCanvas.parentElement.clientHeight || 180;
      drawRadarChart();
    }

    function drawRadarChart() {
      const ctx = radarCanvas.getContext('2d');
      const w = radarCanvas.width;
      const h = radarCanvas.height;
      ctx.clearRect(0, 0, w, h);

      const centerX = w / 2;
      const centerY = h / 2 + 5;
      const radius = Math.min(centerX, centerY) - 26;

      const labels = ['Calor', 'Inercia', 'Flujo', 'Estabilidad', 'Aislamiento'];
      const numSides = 5;
      const angleStep = (Math.PI * 2) / numSides;

      // Dibujar anillos concéntricos
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
      ctx.lineWidth = 1;
      for (let r = 0.25; r <= 1.0; r += 0.25) {
        ctx.beginPath();
        for (let i = 0; i < numSides; i++) {
          const angle = i * angleStep - Math.PI / 2;
          const x = centerX + Math.cos(angle) * (radius * r);
          const y = centerY + Math.sin(angle) * (radius * r);
          if (i === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        }
        ctx.closePath();
        ctx.stroke();
      }

      // Ejes y Etiquetas
      ctx.fillStyle = '#94a3b8';
      ctx.font = '10px sans-serif';
      ctx.textAlign = 'center';
      for (let i = 0; i < numSides; i++) {
        const angle = i * angleStep - Math.PI / 2;
        const x = centerX + Math.cos(angle) * radius;
        const y = centerY + Math.sin(angle) * radius;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(x, y);
        ctx.stroke();

        const labelX = centerX + Math.cos(angle) * (radius + 14);
        const labelY = centerY + Math.sin(angle) * (radius + 14) + 3;
        ctx.fillText(labels[i], labelX, labelY);
      }

      // Polígono de datos en vivo
      const calorScore = triacActive ? 0.9 : 0.3;
      const inerciaScore = 0.85; // Alta inercia metálica
      const flujoScore = (powerLevel === 'high' ? 0.95 : (powerLevel === 'med' ? 0.65 : 0.4));
      const minR = targetTemp - toleranceVal;
      const maxR = targetTemp + toleranceVal;
      const estabScore = (actualTemp >= minR && actualTemp <= maxR) ? 0.9 : 0.45;
      const aislamScore = 0.7;

      const scores = [calorScore, inerciaScore, flujoScore, estabScore, aislamScore];

      ctx.beginPath();
      ctx.fillStyle = 'rgba(56, 189, 248, 0.3)';
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 2;

      for (let i = 0; i < numSides; i++) {
        const angle = i * angleStep - Math.PI / 2;
        const dist = radius * scores[i];
        const x = centerX + Math.cos(angle) * dist;
        const y = centerY + Math.sin(angle) * dist;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
    }

    // ========================================================
    // HOVER TOOLTIP Y EVENTOS DE INTERACCIÓN EN CANVAS
    // ========================================================
    let hoverIndexLarge = null;
    let hoverIndexPulses = null;

    largeCanvas.addEventListener('mousemove', (e) => {
      const rect = largeCanvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const padLeft = 68;
      const padRight = 88;
      const gw = largeCanvas.width - padLeft - padRight;
      if (mouseX >= padLeft && mouseX <= largeCanvas.width - padRight && tempHistory.length > 1) {
        const stepX = gw / (MAX_POINTS - 1);
        const offset = MAX_POINTS - pts.length;
        const idx = Math.round((mouseX - padLeft) / stepX) - offset;
        if (idx >= 0 && idx < tempHistory.length) {
          hoverIndexLarge = idx;
          drawLargeHistoryChart();
          return;
        }
      }
      if (hoverIndexLarge !== null) {
        hoverIndexLarge = null;
        drawLargeHistoryChart();
      }
    });

    largeCanvas.addEventListener('mouseleave', () => {
      if (hoverIndexLarge !== null) {
        hoverIndexLarge = null;
        drawLargeHistoryChart();
      }
    });

    pulsesCanvas.addEventListener('mousemove', (e) => {
      const rect = pulsesCanvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const padLeft = 72;
      const padRight = 50;
      const gw = pulsesCanvas.width - padLeft - padRight;
      if (mouseX >= padLeft && mouseX <= pulsesCanvas.width - padRight && powerPulsesHistory.length > 1) {
        const stepX = gw / (MAX_POINTS - 1);
        const offset = MAX_POINTS - pulses.length;
        const idx = Math.round((mouseX - padLeft) / stepX) - offset;
        if (idx >= 0 && idx < powerPulsesHistory.length) {
          hoverIndexPulses = idx;
          drawPulsesChart();
          return;
        }
      }
      if (hoverIndexPulses !== null) {
        hoverIndexPulses = null;
        drawPulsesChart();
      }
    });

    pulsesCanvas.addEventListener('mouseleave', () => {
      if (hoverIndexPulses !== null) {
        hoverIndexPulses = null;
        drawPulsesChart();
      }
    });

    // ========================================================
    // DIBUJO DEL MINI GRÁFICO PREVIEW (HERO CARD)
    // Con cotas numéricas, setpoint y lectura actual destacada
    // ========================================================
    function resizePreview() {
      if (!previewCanvas.parentElement) return;
      previewCanvas.width = previewCanvas.parentElement.clientWidth || 220;
      previewCanvas.height = previewCanvas.parentElement.clientHeight || 80;
      drawPreviewChart();
    }

    function drawPreviewChart() {
      const ctx = previewCanvas.getContext('2d');
      const w = previewCanvas.width;
      const h = previewCanvas.height;
      ctx.clearRect(0, 0, w, h);

      const padL = 36;
      const padR = 48;
      const padT = 8;
      const padB = 16;
      const gh = h - padT - padB;
      const gw = w - padL - padR;

      const pts = tempHistory.length > 0 ? tempHistory : [actualTemp, actualTemp];
      const minT = Math.min(...pts, targetTemp - 3);
      const maxT = Math.max(...pts, targetTemp + 3);
      const range = (maxT - minT) || 1;
      const toY = (v) => padT + gh * (1 - (v - minT) / range);
      const stepX = gw / (MAX_POINTS - 1);
      const offset = MAX_POINTS - pts.length;

      // Líneas guía y números mín/máx
      ctx.fillStyle = '#64748b';
      ctx.font = '9px monospace';
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      ctx.fillText(`${maxT.toFixed(0)}°`, padL - 4, padT + 2);
      ctx.fillText(`${minT.toFixed(0)}°`, padL - 4, padT + gh);

      // Línea punteada Setpoint
      const yObj = toY(targetTemp);
      ctx.strokeStyle = 'rgba(16, 185, 129, 0.45)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(padL, yObj);
      ctx.lineTo(padL + gw, yObj);
      ctx.stroke();
      ctx.setLineDash([]);

      // Curva histórica
      ctx.beginPath();
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 2.2;
      ctx.lineJoin = 'round';

      for (let i = 0; i < pts.length; i++) {
        const x = padL + (i + offset) * stepX;
        const y = toY(pts[i]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();

      // Punto actual con número
      const xLast = padL + (pts.length - 1 + offset) * stepX;
      const yLast = toY(actualTemp);

      ctx.fillStyle = '#38bdf8';
      ctx.beginPath();
      ctx.arc(xLast, yLast, 4, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 9px sans-serif';
      ctx.textAlign = 'left';
      ctx.fillText(`${actualTemp.toFixed(1)}°C`, xLast + 6, yLast);
      return;
    }

    // ========================================================
    // DIBUJO DEL GRÁFICO GRANDE DE HISTORIAL (TEÓRICO VS REAL)
    // Con Ejes Numéricos, Cuadrícula, Recta de Números y Datos
    // ========================================================
    function resizeLargeCanvas() {
      if (!largeCanvas.parentElement) return;
      largeCanvas.width = largeCanvas.parentElement.clientWidth;
      largeCanvas.height = largeCanvas.parentElement.clientHeight;
      drawLargeHistoryChart();
    }

    function drawLargeHistoryChart() {
      const ctx = largeCanvas.getContext('2d');
      const w = largeCanvas.width;
      const h = largeCanvas.height;
      ctx.clearRect(0, 0, w, h);

      const pts = tempHistory.length > 0 ? tempHistory : [actualTemp];

      // MÁRGENES PROFESIONALES PARA EJES Y NÚMEROS
      const padLeft = 68;
      const padRight = 92;
      const padTop = 32;
      const padBottom = 44;
      const gw = w - padLeft - padRight;
      const gh = h - padTop - padBottom;

      // RANGO DINÁMICO GRADUADO
      const observedMin = Math.min(...pts, targetTemp - toleranceVal - 3, 20.0);
      const observedMax = Math.max(...pts, targetTemp + toleranceVal + 3, 45.0);
      let minT = Math.max(15, Math.floor((observedMin - 1) / 5) * 5);
      let maxT = Math.min(70, Math.ceil((observedMax + 1) / 5) * 5);
      if (maxT - minT < 20) maxT = minT + 20;

      const toY = (v) => padTop + gh * (1 - (v - minT) / (maxT - minT));
      const stepX = gw / (MAX_POINTS - 1);
      const offset = MAX_POINTS - pts.length;
      const toX = (i) => padLeft + (i + offset) * stepX;

      // ----------------------------------------------------
      // 1. CUADRÍCULA HORIZONTAL Y ESCALA NUMÉRICA DEL EJE Y
      // ----------------------------------------------------
      ctx.font = '11px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace';
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';

      for (let tVal = minT; tVal <= maxT; tVal += 5) {
        const yPos = toY(tVal);

        // Línea horizontal de cuadrícula
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(padLeft, yPos);
        ctx.lineTo(padLeft + gw, yPos);
        ctx.stroke();
        ctx.setLineDash([]);

        // Muesca en el eje Y
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
        ctx.beginPath();
        ctx.moveTo(padLeft - 6, yPos);
        ctx.lineTo(padLeft, yPos);
        ctx.stroke();

        // Número en el eje Y
        ctx.fillStyle = '#94a3b8';
        ctx.fillText(`${tVal.toFixed(0)}°C`, padLeft - 9, yPos);
      }

      // Recta sólida vertical del Eje Y
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(padLeft, padTop - 8);
      ctx.lineTo(padLeft, padTop + gh);
      ctx.stroke();

      // Título del Eje Y
      ctx.save();
      ctx.translate(18, padTop + gh / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.fillStyle = '#cbd5e1';
      ctx.font = 'bold 11px sans-serif';
      ctx.fillText('Temperatura (°C)', 0, 0);
      ctx.restore();

      // ----------------------------------------------------
      // 2. CUADRÍCULA VERTICAL Y GRADUACIÓN DE TIEMPO DEL EJE X
      // ----------------------------------------------------
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(padLeft, padTop + gh);
      ctx.lineTo(padLeft + gw + 10, padTop + gh);
      ctx.stroke();

      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';

      const totalSecs = (MAX_POINTS - 1) * 1.5;
      const intervalSec = 15;

      for (let s = 0; s <= totalSecs; s += intervalSec) {
        const sampleIdx = Math.round(s / 1.5);
        const xPos = padLeft + sampleIdx * stepX;
        if (xPos >= padLeft && xPos <= padLeft + gw) {
          // Muesca
          ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
          ctx.beginPath();
          ctx.moveTo(xPos, padTop + gh);
          ctx.lineTo(xPos, padTop + gh + 6);
          ctx.stroke();

          // Línea vertical sutil
          ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
          ctx.setLineDash([2, 4]);
          ctx.beginPath();
          ctx.moveTo(xPos, padTop);
          ctx.lineTo(xPos, padTop + gh);
          ctx.stroke();
          ctx.setLineDash([]);

          // Número en el eje X
          ctx.fillStyle = '#94a3b8';
          ctx.fillText(`${s}s`, xPos, padTop + gh + 8);
        }
      }

      // Título del Eje X
      ctx.fillStyle = '#cbd5e1';
      ctx.font = 'bold 11px sans-serif';
      ctx.fillText('Tiempo transcurrido (segundos)', padLeft + gw / 2, padTop + gh + 26);

      // ----------------------------------------------------
      // 3. FRANJA DE TOLERANCIA Y RECTA DEL SETPOINT CON NÚMEROS
      // ----------------------------------------------------
      const minRango = targetTemp - toleranceVal;
      const maxRango = targetTemp + toleranceVal;
      const yTolTop = toY(maxRango);
      const yTolBot = toY(minRango);

      // Sombreado de tolerancia
      ctx.fillStyle = 'rgba(16, 185, 129, 0.12)';
      ctx.fillRect(padLeft, yTolTop, gw, yTolBot - yTolTop);

      // Líneas de margen superior e inferior
      ctx.strokeStyle = 'rgba(16, 185, 129, 0.6)';
      ctx.lineWidth = 1.2;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(padLeft, yTolTop);
      ctx.lineTo(padLeft + gw, yTolTop);
      ctx.moveTo(padLeft, yTolBot);
      ctx.lineTo(padLeft + gw, yTolBot);
      ctx.stroke();
      ctx.setLineDash([]);

      // Números de límites en el margen derecho
      ctx.textAlign = 'left';
      ctx.textBaseline = 'middle';
      ctx.font = '10px monospace';
      ctx.fillStyle = '#34d399';
      ctx.fillText(`+${toleranceVal.toFixed(1)}° (${maxRango.toFixed(1)}°C)`, padLeft + gw + 5, yTolTop);
      ctx.fillText(`-${toleranceVal.toFixed(1)}° (${minRango.toFixed(1)}°C)`, padLeft + gw + 5, yTolBot);

      // Recta verde continua del Setpoint
      const yObj = toY(targetTemp);
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 2.2;
      ctx.beginPath();
      ctx.moveTo(padLeft, yObj);
      ctx.lineTo(padLeft + gw, yObj);
      ctx.stroke();

      // Placa / Badge con el número exacto del Setpoint
      ctx.fillStyle = '#10b981';
      ctx.beginPath();
      ctx.roundRect(padLeft + gw + 4, yObj - 10, 84, 20, 5);
      ctx.fill();
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 10px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`${targetTemp.toFixed(1)}°C`, padLeft + gw + 46, yObj);

      // ----------------------------------------------------
      // 4. CURVA TEÓRICA IDEAL (1er ORDEN: T(t) = Tamb + (Tobj - Tamb)(1 - e^-t/tau))
      // ----------------------------------------------------
      const TAU = 62.5;
      const T_AMB = 22.0;
      ctx.beginPath();
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 2.4;
      ctx.setLineDash([5, 5]);

      const numCurvesPts = Math.max(pts.length, 30);
      for (let i = 0; i < numCurvesPts; i++) {
        const x = padLeft + i * stepX;
        const tSec = i * 1.5;
        const tempTeorica = T_AMB + (targetTemp - T_AMB) * (1.0 - Math.exp(-tSec / TAU));
        const y = toY(tempTeorica);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);

      // ----------------------------------------------------
      // 5. CURVA REAL MEDIDA (LM35 - Naranja sólida con degradado)
      // ----------------------------------------------------
      ctx.beginPath();
      ctx.strokeStyle = '#f97316';
      ctx.lineWidth = 3.4;
      ctx.lineJoin = 'round';

      for (let i = 0; i < pts.length; i++) {
        const x = toX(i);
        const y = toY(pts[i]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();

      // Degradado bajo la curva real
      ctx.lineTo(toX(pts.length - 1), padTop + gh);
      ctx.lineTo(toX(0), padTop + gh);
      ctx.closePath();
      const grad = ctx.createLinearGradient(0, yObj, 0, padTop + gh);
      grad.addColorStop(0, 'rgba(249, 115, 22, 0.28)');
      grad.addColorStop(1, 'rgba(249, 115, 22, 0.0)');
      ctx.fillStyle = grad;
      ctx.fill();

      // ----------------------------------------------------
      // 6. VALORES Y COTAS NUMÉRICAS DIRECTAS SOBRE LA CURVA
      // ----------------------------------------------------
      // Pico Máximo
      if (picoMax !== null) {
        const maxIdx = tempHistory.lastIndexOf(picoMax);
        if (maxIdx >= 0) {
          const xM = toX(maxIdx);
          const yM = toY(picoMax);
          ctx.fillStyle = '#ef4444';
          ctx.beginPath();
          ctx.arc(xM, yM, 4, 0, Math.PI * 2);
          ctx.fill();

          ctx.fillStyle = 'rgba(239, 68, 68, 0.9)';
          ctx.beginPath();
          ctx.roundRect(xM - 32, yM - 22, 64, 16, 4);
          ctx.fill();
          ctx.fillStyle = '#ffffff';
          ctx.font = 'bold 9px sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText(`▲ MÁX ${picoMax.toFixed(1)}°`, xM, yM - 10);
        }
      }

      // Último punto (Lectura actual)
      const lastIdx = pts.length - 1;
      const xLast = toX(lastIdx);
      const yLast = toY(actualTemp);

      // Anillo de pulso
      ctx.fillStyle = 'rgba(249, 115, 22, 0.35)';
      ctx.beginPath();
      ctx.arc(xLast, yLast, 8, 0, Math.PI * 2);
      ctx.fill();

      // Punto sólido central
      ctx.fillStyle = '#f97316';
      ctx.beginPath();
      ctx.arc(xLast, yLast, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Placa flotante con el número actual
      ctx.fillStyle = '#ea580c';
      ctx.beginPath();
      ctx.roundRect(xLast - 38, yLast - 26, 76, 20, 5);
      ctx.fill();
      ctx.fillStyle = '#ffffff';
      ctx.font = 'bold 10px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`● ${actualTemp.toFixed(1)}°C Actual`, xLast, yLast - 12);

      // ----------------------------------------------------
      // 7. HOVER TOOLTIP INTERACTIVO (DATOS AL PASAR EL RATÓN)
      // ----------------------------------------------------
      if (hoverIndexLarge !== null && hoverIndexLarge >= 0 && hoverIndexLarge < tempHistory.length) {
        const hX = toX(hoverIndexLarge);
        const hY = toY(tempHistory[hoverIndexLarge]);

        // Línea vertical guía
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.45)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(hX, padTop);
        ctx.lineTo(hX, padTop + gh);
        ctx.stroke();
        ctx.setLineDash([]);

        // Punto de intersección
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(hX, hY, 5, 0, Math.PI * 2);
        ctx.fill();

        // Tarjeta flotante con datos
        const tipW = 145;
        const tipH = 62;
        let tipX = hX + 12;
        if (tipX + tipW > padLeft + gw) tipX = hX - tipW - 12;
        let tipY = hY - tipH / 2;
        if (tipY < padTop) tipY = padTop + 4;
        if (tipY + tipH > padTop + gh) tipY = padTop + gh - tipH - 4;

        ctx.fillStyle = 'rgba(15, 23, 42, 0.95)';
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.roundRect(tipX, tipY, tipW, tipH, 8);
        ctx.fill();
        ctx.stroke();

        const tSec = (hoverIndexLarge * 1.5).toFixed(1);
        const tReal = tempHistory[hoverIndexLarge].toFixed(2);
        const tTeo = (T_AMB + (targetTemp - T_AMB) * (1.0 - Math.exp(-(hoverIndexLarge * 1.5) / TAU))).toFixed(2);

        ctx.textAlign = 'left';
        ctx.font = 'bold 10px sans-serif';
        ctx.fillStyle = '#94a3b8';
        ctx.fillText(`Tiempo: ${tSec} s`, tipX + 10, tipY + 16);

        ctx.fillStyle = '#fb923c';
        ctx.fillText(`Temp Real: ${tReal} °C`, tipX + 10, tipY + 34);

        ctx.fillStyle = '#38bdf8';
        ctx.fillText(`Teórica: ${tTeo} °C`, tipX + 10, tipY + 52);
      }
    }

    // ========================================================
    // DIBUJO DEL GRÁFICO DE PICOS DE POTENCIA Y PULSOS TRIAC
    // Con Eje Y en Watts graduado, Eje X en Tiempo y Ciclos
    // ========================================================
    function resizePulsesCanvas() {
      if (!pulsesCanvas.parentElement) return;
      pulsesCanvas.width = pulsesCanvas.parentElement.clientWidth;
      pulsesCanvas.height = pulsesCanvas.parentElement.clientHeight;
      drawPulsesChart();
    }

    function drawPulsesChart() {
      const ctx = pulsesCanvas.getContext('2d');
      const w = pulsesCanvas.width;
      const h = pulsesCanvas.height;
      ctx.clearRect(0, 0, w, h);

      const pulses = powerPulsesHistory.length > 0 ? powerPulsesHistory : [{ active: triacActive, power: triacActive ? 850 : 0 }];

      const padLeft = 72;
      const padRight = 50;
      const padTop = 30;
      const padBottom = 44;
      const gw = w - padLeft - padRight;
      const gh = h - padTop - padBottom;

      const stepX = gw / (MAX_POINTS - 1);
      const offset = MAX_POINTS - pulses.length;
      const toX = (i) => padLeft + (i + offset) * stepX;

      const y0 = padTop + gh;      // 0 Watts
      const yMax = padTop + 10;    // 850 Watts (Pico Máx)
      const yMed = padTop + gh * (1 - 560 / 850);
      const yLow = padTop + gh * (1 - 280 / 850);

      // ----------------------------------------------------
      // 1. LÍNEAS DE GUÍA Y ESCALA GRADUADA EN WATTS
      // ----------------------------------------------------
      const levels = [
        { y: yMax, label: '850 W (Pico)', sub: 'Máx' },
        { y: yMed, label: '560 W', sub: 'Medio' },
        { y: yLow, label: '280 W', sub: 'Bajo' },
        { y: y0,   label: '0 W (Corte)', sub: 'Apagado' }
      ];

      ctx.font = '10px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace';
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';

      levels.forEach(lvl => {
        // Línea horizontal
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 4]);
        ctx.beginPath();
        ctx.moveTo(padLeft, lvl.y);
        ctx.lineTo(padLeft + gw, lvl.y);
        ctx.stroke();
        ctx.setLineDash([]);

        // Muesca
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
        ctx.beginPath();
        ctx.moveTo(padLeft - 6, lvl.y);
        ctx.lineTo(padLeft, lvl.y);
        ctx.stroke();

        // Número en Watts
        ctx.fillStyle = '#94a3b8';
        ctx.fillText(lvl.label, padLeft - 9, lvl.y);
      });

      // Ejes sólidos
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(padLeft, padTop);
      ctx.lineTo(padLeft, padTop + gh);
      ctx.lineTo(padLeft + gw + 10, padTop + gh);
      ctx.stroke();

      // Título Eje Y
      ctx.save();
      ctx.translate(16, padTop + gh / 2);
      ctx.rotate(-Math.PI / 2);
      ctx.textAlign = 'center';
      ctx.fillStyle = '#cbd5e1';
      ctx.font = 'bold 11px sans-serif';
      ctx.fillText('Potencia Eléctrica (Watts)', 0, 0);
      ctx.restore();

      // Graduaciones en Eje X
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      const totalSecs = (MAX_POINTS - 1) * 1.5;
      for (let s = 0; s <= totalSecs; s += 15) {
        const sampleIdx = Math.round(s / 1.5);
        const xPos = padLeft + sampleIdx * stepX;
        if (xPos >= padLeft && xPos <= padLeft + gw) {
          ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
          ctx.beginPath();
          ctx.moveTo(xPos, padTop + gh);
          ctx.lineTo(xPos, padTop + gh + 6);
          ctx.stroke();

          ctx.fillStyle = '#94a3b8';
          ctx.fillText(`${s}s`, xPos, padTop + gh + 8);
        }
      }

      ctx.fillStyle = '#cbd5e1';
      ctx.font = 'bold 11px sans-serif';
      ctx.fillText('Tiempo transcurrido (segundos)', padLeft + gw / 2, padTop + gh + 26);

      // ----------------------------------------------------
      // 2. FORMA DE ONDA DE PULSOS (ONDA CUADRADA DEL TRIAC)
      // ----------------------------------------------------
      ctx.beginPath();
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2.8;

      let sumWatts = 0;
      let activeCount = 0;

      for (let i = 0; i < pulses.length; i++) {
        const x = toX(i);
        const item = pulses[i];
        const isON = item.active;
        if (isON) activeCount++;
        const targetWatts = item.power || (isON ? 850 : 0);
        sumWatts += targetWatts;

        const y = isON ? (padTop + gh * (1 - targetWatts / 850)) : y0;

        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          const prevItem = powerPulsesHistory[i - 1];
          const prevON = prevItem.active;
          const prevWatts = prevItem.power || (prevON ? 850 : 0);
          const prevY = prevON ? (padTop + gh * (1 - prevWatts / 850)) : y0;

          if (prevY !== y) {
            ctx.lineTo(x, prevY); // Flanco vertical de conmutación
          }
          ctx.lineTo(x, y);
        }
      }
      ctx.stroke();

      // Relleno bajo los pulsos
      ctx.lineTo(toX(pulses.length - 1), y0);
      ctx.lineTo(toX(0), y0);
      ctx.closePath();
      ctx.fillStyle = 'rgba(245, 158, 11, 0.16)';
      ctx.fill();

      // ----------------------------------------------------
      // 3. ANOTACIONES Y MÉTRICAS
      // ----------------------------------------------------
      const avgWatts = Math.round(sumWatts / pulses.length);
      const duty = ((activeCount / pulses.length) * 100).toFixed(1);
      const sumEl = document.getElementById('pulsesStatSummary');
      if (sumEl) sumEl.textContent = `${avgWatts} W promedio consumido | Duty: ${duty}%`;

      // ----------------------------------------------------
      // 4. HOVER TOOLTIP EN PULSOS
      // ----------------------------------------------------
      if (hoverIndexPulses !== null && hoverIndexPulses >= 0 && hoverIndexPulses < powerPulsesHistory.length) {
        const hX = toX(hoverIndexPulses);
        const item = powerPulsesHistory[hoverIndexPulses];
        const isON = item.active;
        const curWatts = item.power || (isON ? 850 : 0);
        const hY = isON ? (padTop + gh * (1 - curWatts / 850)) : y0;

        ctx.strokeStyle = 'rgba(255, 255, 255, 0.45)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(hX, padTop);
        ctx.lineTo(hX, padTop + gh);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(hX, hY, 5, 0, Math.PI * 2);
        ctx.fill();

        // Tooltip
        const tipW = 140;
        const tipH = 54;
        let tipX = hX + 12;
        if (tipX + tipW > padLeft + gw) tipX = hX - tipW - 12;
        let tipY = hY - tipH / 2;
        if (tipY < padTop) tipY = padTop + 4;
        if (tipY + tipH > padTop + gh) tipY = padTop + gh - tipH - 4;

        ctx.fillStyle = 'rgba(15, 23, 42, 0.95)';
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.roundRect(tipX, tipY, tipW, tipH, 8);
        ctx.fill();
        ctx.stroke();

        const tSec = (hoverIndexPulses * 1.5).toFixed(1);
        ctx.textAlign = 'left';
        ctx.font = 'bold 10px sans-serif';
        ctx.fillStyle = '#94a3b8';
        ctx.fillText(`Tiempo: ${tSec} s`, tipX + 10, tipY + 16);

        ctx.fillStyle = isON ? '#f59e0b' : '#38bdf8';
        ctx.fillText(isON ? `TRIAC: ON (${curWatts} W)` : 'TRIAC: OFF (0 W)', tipX + 10, tipY + 36);
      }
    }
    // LOG DE EVENTOS
    function addLogEntry(msg) {
      const box = document.getElementById('historyLogContainer');
      const now = new Date();
      const timeStr = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
      const item = document.createElement('div');
      item.className = 'log-row-item';
      item.innerHTML = `<span class="log-row-time">[${timeStr}]</span> <span class="log-row-msg">${msg}</span>`;
      box.insertBefore(item, box.firstChild);
    }

    window.addEventListener('resize', () => {
      resizeRadar();
      resizePreview();
      resizeLargeCanvas();
      resizePulsesCanvas();
    });

    // INICIALIZACIÓN INMEDIATA
    try { updateUI(); } catch (e) { console.error('Error en updateUI inicial:', e); }
    resizeRadar();
    resizePreview();
    drawRadarChart();
    drawPreviewChart();
    updateLoop();
    setInterval(updateLoop, 1200);
  </script>
</body>
</html>


)rawliteral";

// =======================================
// CABECERAS CORS
// =======================================
void enviarCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
}

// =======================================
// RUTAS DEL SERVIDOR
// =======================================
void configurarWeb() {

  // PREFLIGHT CORS
  server.on("/estado", HTTP_OPTIONS, []() {
    enviarCORS();
    server.send(204);
  });
  server.on("/set", HTTP_OPTIONS, []() {
    enviarCORS();
    server.send(204);
  });
  server.on("/modo", HTTP_OPTIONS, []() {
    enviarCORS();
    server.send(204);
  });

  // PAGINA PRINCIPAL
  server.on("/", []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  // DATOS JSON
  server.on("/estado", HTTP_GET, []() {
    enviarCORS();
    String json = "{";
    json += "\"valido\":true";
    json += ",\"temperatura\":" + String(temperatura, 1);
    json += ",\"objetivo\":" + String(objetivo, 1);
    json += ",\"tolerancia\":" + String(tolerancia, 1);
    json += ",\"potencia\":\"" + nivelPotencia + "\"";
    json += ",\"calefaccion\":" + String(calefaccion ? "true" : "false");
    json += ",\"modo\":\"" + nombreModo() + "\"";
    json += ",\"manualEncendido\":" + String(manualEncendido ? "true" : "false");
    json += ",\"fase\":\"" + String(modo == AUTOMATICO ? faseAuto : "") + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });

  // MODO
  server.on("/modo", HTTP_GET, []() {
    enviarCORS();
    String m = server.arg("m");
    if (m == "off") {
      modo = APAGADO;
      manualEncendido = false;
      actualizarSalida(false);
    } 
    else if (m == "auto") {
      modo = AUTOMATICO;
      manualEncendido = false;
      actualizarControl();
    } 
    else if (m == "manual_on") {
      modo = MANUAL;
      manualEncendido = true;
      actualizarSalida(true);
    } 
    else if (m == "manual_off") {
      modo = MANUAL;
      manualEncendido = false;
      actualizarSalida(false);
    }
    server.send(200, "text/plain", "OK");
  });

  // SETPOINT
  server.on("/set", HTTP_GET, []() {
    enviarCORS();
    if (server.hasArg("t")) {
      float nuevo = server.arg("t").toFloat();
      if (nuevo >= 20.0 && nuevo <= 65.0) {
        objetivo = nuevo;
        Serial.printf("[OBJETIVO ACTUALIZADO] -> %.1f C\n", objetivo);
      }
    }
    if (server.hasArg("tol")) {
      float tol = server.arg("tol").toFloat();
      if (tol >= 0.5 && tol <= 5.0) {
        tolerancia = tol;
      }
    }
    if (server.hasArg("pot")) {
      String p = server.arg("pot");
      if (p == "low" || p == "med" || p == "high") {
        nivelPotencia = p;
      }
    }
    actualizarControl();
    server.send(200, "text/plain", "OK");
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
  analogSetPinAttenuation(PIN_LM35, ADC_11db);

  Wire.begin(PIN_SDA, PIN_SCL);
  oledDisponible = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Lectura inicial del sensor
  temperatura = leerLM35();
  if (!isfinite(temperatura) || temperatura < 2.0f) {
    temperatura = 24.0f;
  }

  actualizarControl();

  WiFi.mode(WIFI_AP);
  bool wifiOK = WiFi.softAP(ssid, password);

  if (wifiOK) {
    configurarWeb();
    Serial.println("\n==================================");
    Serial.println("  HORNO 4.0 GG - LISTO Y CONECTADO");
    Serial.println("==================================");
    Serial.print("Red Wi-Fi: "); Serial.println(ssid);
    Serial.print("Clave:     "); Serial.println(password);
    Serial.print("Web:       http://"); Serial.println(WiFi.softAPIP());
    Serial.println("==================================");
  }

  actualizarOLED();
}

// =======================================
// LOOP PRINCIPAL
// =======================================
void loop() {
  server.handleClient();

  unsigned long ahora = millis();

  // Control térmico cada 100 ms
  if (ahora - ultimaLectura >= INTERVALO) {
    ultimaLectura = ahora;
    actualizarControl();
  }

  // Refresco cada 500 ms
  if (ahora - ultimoRefrescoOLED >= INTERVALO_OLED) {
    ultimoRefrescoOLED = ahora;
    actualizarOLED();

    Serial.printf("[HORNO GG] Sensor: %.1f C | Obj: %.1f C | Modo: %s | Fase: %s | Secadora: %s\n",
                  temperatura, objetivo, nombreModo().c_str(),
                  modo == AUTOMATICO ? faseAuto.c_str() : "-",
                  calefaccion ? "ENCENDIDA" : "APAGADA");
  }
}
