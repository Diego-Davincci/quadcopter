# Fase 0 — Entorno y plataforma base

**Objetivo de ingeniería:** validar la cadena de trabajo completa
(_editar → compilar → flashear → monitorear → versionar_) sobre una base
de código portable, **antes** de añadir cualquier hardware nuevo. Aislamos
una sola variable: la herramienta. Si algo falla más adelante, ya sabremos
que NO es el entorno.

> **Criterio rector de la fase:** no se mide "que funcione un LED", se mide
> que tu flujo de trabajo sea _repetible y portable_. El LED es solo el
> testigo visible de que toda la cadena está sana.

> **🔄 Replanteo (ESP32 disponible):** el ESP32 ya está en mano, así que es
> la **plataforma principal** desde la Fase 0. Consecuencias: (1) desaparece
> la "migración" de la Fase 4 como evento —arrancamos en el target final;
> (2) la Fase 1 (IMU) es directa, sin adaptar niveles, porque IMU y ESP32
> son ambos de 3.3 V; (3) la FPU está disponible desde ya. El Uno queda como
> **entorno opcional** (respaldo y comparación), ya no condiciona el plan.
> El entorno por defecto en `platformio.ini` es ahora `esp32`.

---

## 1. El Arduino Uno / ATmega328P como dispositivo

### Qué es

El "cerebro" del Uno es el **ATmega328P**, un microcontrolador AVR de
**8 bits** con arquitectura **Harvard** (buses separados para programa y
datos) y conjunto de instrucciones RISC. No es una computadora con sistema
operativo: es un chip que ejecuta _un solo programa_ en bucle infinito,
directamente sobre el metal.

### Recursos (y sus límites)

| Recurso          | Valor                 | Por qué importa                                                                                               |
| ---------------- | --------------------- | ------------------------------------------------------------------------------------------------------------- |
| Frecuencia       | **16 MHz**            | Velocidad de cómputo. Suficiente para parpadear; ajustado para fusión de sensores + PID a buena tasa.         |
| Flash (programa) | 32 KB                 | Donde vive tu firmware. Por eso usamos `F()` para no malgastarla…                                             |
| **SRAM (datos)** | **2 KB**              | …y por eso la SRAM es el cuello de botella real. Variables, buffers y pila viven aquí. 2 KB se agotan rápido. |
| EEPROM           | 1 KB                  | Memoria no volátil para calibraciones (la usaremos para offsets de la IMU).                                   |
| Lógica           | **5 V**, ~20 mA/pin   | Define compatibilidad eléctrica. **Ojo:** la MPU-6050 es de 3.3 V (tema de la Fase 1).                        |
| **FPU**          | **No tiene**          | Crítico. No hay coma flotante por hardware.                                                                   |
| UART hardware    | 1                     | El puerto serie/USB. Uno solo.                                                                                |
| I²C (TWI) / SPI  | 1 / 1                 | I²C para la IMU; ambos se comparten con pines fijos.                                                          |
| Timers           | 3 (2×8 bit, 1×16 bit) | Generan el PWM. Aquí está el verdadero límite para 4 motores.                                                 |
| ADC              | 6 canales, 10 bits    | Lectura analógica (medir voltaje de batería, etc.).                                                           |

### El detalle que define todo: no hay FPU

El ATmega328P **no tiene unidad de coma flotante**. Cada operación con
`float` (sumar, multiplicar, y peor: dividir, `sqrt`, `atan2`) se _emula
por software_ con decenas o cientos de ciclos. Un PID en cascada con fusión
de sensores hace cientos de operaciones flotantes por iteración, y un lazo
de vuelo decente corre a 200–500 Hz. El Uno se ahoga ahí.

→ **Por eso el Uno es solo provisional.** Sirve perfecto para aprender el
entorno y para las fases de sensor y de un motor (Fases 0–3), donde la
carga de cómputo es trivial. Pero el control "fuerte" (Fases 5+) exige el
ESP32.

### Por qué arrancar en el Uno y no directo en el ESP32

1. **Aísla la herramienta del hardware objetivo.** Validamos PlatformIO sin
   arriesgar el ESP32.
2. **Es eléctricamente robusto** (5 V, tolerante, alimentado por USB):
   difícil de dañar mientras aprendes el flujo.
3. **Te obliga a escribir código portable** desde el día 1. Si funciona en
   ambas placas sin tocar `main.cpp`, tu arquitectura es sana.

---

## 2. El ESP32 WROOM-32 como objetivo final

| Aspecto      | Arduino Uno                    | ESP32 WROOM-32                                        |
| ------------ | ------------------------------ | ----------------------------------------------------- |
| Núcleos      | 1                              | **2** (Xtensa LX6)                                    |
| Reloj        | 16 MHz                         | **240 MHz** (~15×)                                    |
| FPU          | No                             | **Sí (hardware)** ← decisivo para PID                 |
| SRAM         | 2 KB                           | ~520 KB (~260×)                                       |
| Lógica       | 5 V                            | **3.3 V** (nativa para la MPU-6050)                   |
| Conectividad | Ninguna                        | **WiFi + Bluetooth** (telemetría/tuning inalámbrico)  |
| Canales PWM  | Limitados (timers compartidos) | 16 canales LEDC independientes → 4 motores sin pelear |

El ESP32 resuelve exactamente los cuatro cuellos de botella del Uno: FPU,
SRAM, voltaje (coincide con la IMU sin adaptación) y PWM (4 canales limpios
para los motores). Como ya está disponible, lo adoptamos desde la Fase 0;
no hay "migración" pendiente.

### 2.1 Reglas de pines del ESP32 (esto el Uno no lo exigía)

A diferencia del Uno, en el ESP32 **no todos los pines sirven para todo**.
Sembramos el mapa ahora para no equivocarnos en las Fases 1–3:

| Pines                   | Estado           | Uso                                      |
| ----------------------- | ---------------- | ---------------------------------------- |
| GPIO 6–11               | **Prohibidos**   | Conectados a la flash SPI interna.       |
| GPIO 34, 35, 36, 39     | **Solo entrada** | Sin salida ni PWM → NO para motores.     |
| GPIO 0, 2, 12, 15       | _Strapping_      | Afectan el arranque; usar con cuidado.   |
| GPIO 21 / 22            | I²C por defecto  | **SDA / SCL** para la MPU-6050 (Fase 1). |
| GPIO 25, 26, 27, 32, 33 | Libres y seguros | **PWM de los 4 motores** (Fases 2–3).    |

Reglas eléctricas: lógica **3.3 V**, los GPIO **no toleran 5 V** (una señal
de 5 V puede dañar el pin). El ESP32 es tu controlador final e insustituible
a corto plazo: trátalo con más respeto que al Uno.

---

## 3. Explicación del programa de validación (`src/main.cpp`)

| Decisión                        | Qué hace                                              | Porqué                                                                                                            |
| ------------------------------- | ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `millis()` no bloqueante        | Conmuta el LED comparando tiempos en vez de `delay()` | `delay()` congela el micro. Un lazo de control jamás puede congelarse. Practicamos ya el patrón temporal del PID. |
| `LED_BUILTIN`                   | Pin del LED de placa                                  | El core lo define solo: 13 en Uno, GPIO2 en ESP32. Cero cambios al migrar.                                        |
| `F("...")`                      | Cadenas en flash, no en SRAM                          | Con solo 2 KB de SRAM en el Uno, mover texto a la flash (32 KB) evita corrupción silenciosa de memoria.           |
| `#if defined(ARDUINO_ARCH_...)` | El firmware se autoidentifica al arrancar             | Confirma visualmente _para qué placa_ compilaste. Evita el clásico "flasheé el target equivocado".                |
| Contador de latidos             | Imprime `#0, #1, #2…` con timestamp                   | Prueba que el lazo corre de verdad y que el reloj (`millis`) avanza coherente.                                    |

---

## 4. ¿Diagrama de circuito?

**No aplica en la Fase 0.** El LED testigo ya está cableado _dentro_ de la
placa (GPIO2 → LED en el ESP32; pin 13 → LED en el Uno) y la alimentación
es por USB. No se conecta nada externo. El primer circuito real (motor +
MOSFET + diodo flyback + LiPo) aparece en la **Fase 2**, y allí sí incluirá
su diagrama.

```
   PC ──USB──> [ ESP32 WROOM-32 ]
                    │ (interno)
                    └── GPIO2 ──► LED de placa
   Alimentación y datos: ambos por el mismo cable USB.
```

---

## 5. Pruebas de verificación

Ejecuta en orden. Cada prueba aísla un eslabón de la cadena.

| #   | Prueba                   | Comando / acción                              | Resultado esperado                                                                                               |
| --- | ------------------------ | --------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| P1  | Compilación ESP32        | `pio run -e esp32`                            | `SUCCESS`, sin errores ni warnings graves.                                                                       |
| P2  | Flasheo ESP32            | `pio run -e esp32 -t upload`                  | Sube el binario (si se cuelga en `Connecting...`, mantén **BOOT**). El LED empieza a parpadear.                  |
| P3  | LED (testigo físico)     | Observar la placa                             | Parpadeo regular ~1 Hz (0.5 s ON / 0.5 s OFF) en GPIO2.                                                          |
| P4  | Serial + info de chip    | `pio device monitor`                          | Tras el log del bootloader, ves el banner con CPU (≈240 MHz), flash y SRAM, y `latido #N` incrementando ~500 ms. |
| P5  | Ciclo editar→flashear    | Cambia `INTERVALO_MS` a 150, reflashea        | El parpadeo se acelera visiblemente. Controlas el ciclo completo.                                                |
| P6  | Portabilidad (opcional)  | `pio run -e uno`                              | `SUCCESS`. Demuestra que el mismo código compila para el Uno sin tocarlo.                                        |
| P7  | Reconstrucción repetible | `pio run -t clean` y luego `pio run -e esp32` | Compila desde cero sin errores. Build reproducible.                                                              |
| P8  | Versionado limpio        | `git status` tras `git add . && git commit`   | Árbol limpio y `.pio/` **no** aparece como cambio (lo ignora `.gitignore`).                                      |

---

## 6. Criterio de aceptación de la Fase 0

> La Fase 0 se considera **superada** cuando, de forma simultánea:
>
> 1. El entorno `esp32` **compila** sin errores (P1).
> 2. El ESP32 **flashea** y el LED **late a ~1 Hz** en GPIO2 (P2, P3).
> 3. El monitor serie muestra el **banner con info del chip** y **latidos
>    coherentes** con `millis()`, incrementando con timestamps consistentes (P4).
> 4. Demuestras dominio del **ciclo editar→compilar→flashear→observar** (P5).
> 5. El firmware se **reconstruye de cero** de forma repetible (P7) y el
>    **repo git está limpio** con `.pio/` ignorado (P8).
>
> _Opcional pero recomendado:_ el código también compila para el Uno (P6),
> confirmando que la portabilidad sigue intacta.

Cumplido esto, el entorno deja de ser una variable: cualquier fallo futuro
será del subsistema nuevo, no de la herramienta. Recién ahí pasamos a la
Fase 1 (IMU MPU-6050).

---

## 7. Errores frecuentes (y su causa)

- **El flasheo se cuelga en `Connecting........_____`** → mantén presionado
  el botón **BOOT** del ESP32 al iniciar la subida; suéltalo cuando empiece
  a escribir. Algunas DevKit no auto-entran en modo flasheo.
- **El SO no ve el puerto del ESP32** → falta el driver USB-serie. Mira si tu
  placa trae **CP2102** (Silicon Labs) o **CH340** e instala el suyo.
- **"A fatal error occurred: Failed to connect"** → cable USB solo-carga (sin
  datos), puerto ocupado por el monitor, o velocidad alta: descomenta
  `upload_speed = 115200` en `platformio.ini`.
- **Veo `rst:0x1 (POWERON_RESET)`, `boot:0x...` al abrir el monitor** → es
  el log normal del bootloader del ESP32, no es un error.
- **Texto basura en el serial** → la velocidad del monitor no coincide con
  `Serial.begin()`. Ambos deben ser 115200.
- **El LED no parpadea pero compila** → flasheaste el target equivocado;
  el banner del serial te confirma para qué placa es el binario.
- **`.pio/` aparece en cada commit** → falta o está mal el `.gitignore`.
- **(Uno) "avrdude: ser_open() can't open device"** → puerto equivocado u
  ocupado por el monitor; cierra el monitor antes de flashear.
