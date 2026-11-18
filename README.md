# Horno Térmico 4.0 — Lonchera

Control de temperatura de una lonchera térmica con ESP32, TRIAC y secadora.

## Archivos

- `HornoProyecto.ino` — firmware del ESP32 (sensor LM35, control del TRIAC, servidor web y punto de acceso Wi-Fi).
- `index.html` — dashboard web (temperatura en vivo, modos automático/manual, potencia e historial).

## Cómo usarlo

1. Carga `HornoProyecto.ino` en el ESP32 con el Arduino IDE.
2. Conéctate a la red Wi-Fi `Horno4_0` que crea el ESP32.
3. Abre `index.html` en el navegador (o la IP `192.168.4.1`).
4. Sin ESP32, usa el botón **Modo Piloto** para simular.
