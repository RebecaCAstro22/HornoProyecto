# Horno Térmico 4.0 — Lonchera

Control de temperatura de una lonchera térmica con ESP32, TRIAC y secadora.

## Archivos

- `HornoProyecto.ino` — firmware del ESP32: sensor LM35 con detección de desconexión, control del TRIAC (ráfagas fijas, PID y autoajuste del PID), seguridad por sobrecalentamiento, pantalla OLED, conexión por USB (115200 baudios) y por Wi-Fi.
- `index.html` — dashboard: temperatura en vivo, modos automático/manual, PID, calibración, alertas, historial y lonchera 3D. Se conecta por USB o por Wi-Fi.

## Cómo usarlo

1. Carga `HornoProyecto.ino` en el ESP32 con el Arduino IDE (cierra el Monitor Serial después de subirlo).
2. Abre `index.html` en **Google Chrome** o **Microsoft Edge** y conéctate de una de estas dos formas:
   - **USB:** clic en **Conectar USB** y elige el puerto del ESP32.
   - **Wi-Fi:** sin hacer nada, el dashboard busca el horno en `horno.local` (Wi-Fi de la casa) o en `192.168.4.1` (red propia `Horno4_0`, clave `Horno2026`). Para meterlo al Wi-Fi de la casa: conéctate a `Horno4_0` y abre `http://192.168.4.1/wifi`.
3. Sin ESP32, usa el botón **Modo Piloto** para simular.

## Seguridad

- Aviso de temperatura alta a **60 °C** (banner rojo + sonido).
- A **65 °C reales** el ESP32 apaga la secadora en cualquier modo y no la vuelve a encender hasta bajar de 60 °C.
- Sensor desconectado → secadora apagada en modo automático.
- Botón **Simular sobrecalentamiento (65 °C · 6 s)** para la presentación: muestra las alertas sin calentar (la secadora queda apagada mientras dura).

## Prueba completa (ráfagas vs PID)

Botón **Prueba completa (3 en 1)** o `http://horno.local/prueba`: enfría, prueba ráfagas fijas, hace el autoajuste, enfría y prueba el PID. Al final deja el horno apagado y muestra la tabla, la gráfica y el CSV.

## Comandos (USB o `/cmd?c=` por Wi-Fi)

```
MODO auto | MODO off | MODO manual_on | MODO manual_off
SET t=40 tol=2 pot=high ctl=rafaga|pid|autotune kp=0.2 ki=0.01 kd=1 banda=5 ref=45.3
PRUEBA iniciar 15 | PRUEBA cancelar
DEMO sobrecalentamiento
```
