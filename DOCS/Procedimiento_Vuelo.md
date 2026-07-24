# Procedimiento de vuelo — Daltonics_Vuelo (despegue/aterrizaje por tecla)

Firmware Aire: `Firmware/Daltonics_Vuelo/Daltonics_Vuelo.ino` (ESP32-C3 SuperMini)
Firmware Tierra: `Firmware/Daltonics_Tierra/Daltonics_Tierra.ino` (con telemetría al operador)
Mando: `CONTROL/Control_Vuelo_Daltonics.m` (MATLAB)

**Telemetría**: el dron reporta a MATLAB a 10 Hz su roll/pitch, gas, modo y alertas
(emergencia, IMU caída, aterrizado) — se ve en la ventana del mando como "DRON >>".
El viejo `ESP32_PRUEBATIERRA` sigue sirviendo como respaldo, pero sin telemetría.

## Máquina de estados (campo A de la trama)

| A | Estado | Motores | LED |
|---|--------|---------|-----|
| 0 | DESARMADO | Apagados | Verde tenue |
| 1 | IDLE | Girando suave (1120 µs), sin despegar | Verde brillante |
| 2 | VUELO | Rampa 8 %/s hasta gas de hover + PID de actitud | Azul |
| 3 | ATERRIZAJE | Rampa automática -3.5 %/s hasta idle | Amarillo → celeste |
| — | EMERGENCIA (>45° de inclinación) | Apagados hasta desarmar (Q) y rearmar | Rojo |
| — | FAILSAFE (>500 ms sin señal) | Apagados | Naranja |

## Teclas (MATLAB)

`E` armar/idle · `T` despegue · `L` aterrizar · `Q` corte total ·
`W/S` trim de gas ±0.5 % · flechas roll/pitch · `A/D` yaw · `ESPACIO` nivelar

## Secuencia de esta noche

1. **Alimentación**: usar la **LiPo real**. La fuente de banco se cae de amperaje y
   puede resetear el ESP32 **con los motores armados**. Si igual usás la fuente,
   solo pruebas SIN hélices.
2. Encender el dron **quieto y nivelado**: al arrancar hace 3 s de calibración del
   giroscopio (LED violeta). No moverlo hasta que termine.
3. Espera los beeps de los ESC (LED ámbar, pulso mínimo 3 s).
4. **Prueba SIN hélices** (obligatoria la primera vez):
   - `E` → los 4 motores giran parejos en idle. Si uno no arranca, recalibrar ESCs.
   - `T` → inclinar el dron a mano y ver por Serial que el motor del lado que baja
     **sube** de PWM. Si es al revés, corregir signos antes de poner hélices.
   - `L` y `Q` → verificar rampa de bajada y corte.
5. **Primera prueba con hélices**: dron sujetado (banco o correas), a baja altura,
   sin gente cerca del plano de las hélices.
6. En hover, ajustar el gas con `W/S`: el 45 % inicial se midió con la fuente
   dudosa, así que el valor real con LiPo puede ser distinto.

## Ajuste del PID (en orden, sin hélices primero)

1. Subir `Kp_ang` hasta que empuje de regreso con fuerza al inclinar, sin oscilar.
2. Si tiembla al soltar, subir `Kd_ang`.
3. Solo al final, si queda "recostado", subir `Ki_ang` de a poco.

## Sin altímetro

El hover mantiene el dron **nivelado**, pero la altura es lazo abierto: deriva con
la batería descargándose. Compensar con `W/S`. Siguiente mejora sugerida: montar el
BME680 o un VL53L0X para lazo cerrado de altura.
