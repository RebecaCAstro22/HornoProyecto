# Horno Térmico 4.0 — Lonchera

Control de temperatura de una lonchera térmica con ESP32, TRIAC y secadora.

## Archivos

- `HornoProyecto.ino` — firmware del ESP32 (sensor LM35 con detección de desconexión, control del TRIAC, pantalla OLED y comunicación por USB a 115200 baudios).
- `index.html` — dashboard web conectado por cable USB (Web Serial en Chrome/Edge) con temperatura en vivo, modos automático/manual, potencia, historial y lonchera 3D.

## Cómo usarlo

1. Carga `HornoProyecto.ino` en el ESP32 con el Arduino IDE (cierra el Monitor Serial después de subirlo).
2. Abre `index.html` en **Google Chrome** o **Microsoft Edge**.
3. Haz clic en **Conectar USB** y selecciona el puerto del ESP32.
4. Sin ESP32, usa el botón **Modo Piloto** para simular.
