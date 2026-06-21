# Microcuadricóptero brushed — Proyecto de Fundamentos de Control

Banco de pruebas y plataforma de vuelo para aplicar teoría de control
(fusión de sensores + PID en cascada) sobre una planta física real:
un micro-cuadricóptero de motores _brushed_ 8520.

> **Filosofía:** un subsistema a la vez; cada fase tiene un criterio de
> aceptación medible; no se avanza hasta demostrar que la fase pasó.

---

## Hardware

| Rol                              | Componente                                         |
| -------------------------------- | -------------------------------------------------- |
| Controlador de vuelo (principal) | ESP32 WROOM-32, 30 pines (240 MHz, **3.3 V**)      |
| Banco opcional / respaldo        | Arduino Uno (ATmega328P, 16 MHz, **5 V**)          |
| IMU                              | MPU-6050 / GY-521 (giroscopio + acelerómetro, I²C) |
| Actuadores                       | 4× motor brushed 8520 (2 CW + 2 CCW)               |
| Hélices                          | 46 mm tipo AB, eje 1.0 mm                          |
| Drivers                          | 4× MOSFET IRLZ44N (canal N, lógico)                |
| Protección                       | Diodos Schottky 1N5819 (flyback)                   |
| Energía                          | LiPo 1S 1000 mAh 25C + cargador JST-PH 2.0         |

---

## Plan por fases

| #   | Fase                                            | Estado     |
| --- | ----------------------------------------------- | ---------- |
| 0   | Entorno y plataforma base                       | **← AQUÍ** |
| 1   | Sensor: IMU MPU-6050 (Uno/USB)                  | pendiente  |
| 2   | Actuador: un motor + MOSFET (entra la LiPo)     | pendiente  |
| 3   | Cuatro motores + mezcla (mixing)                | pendiente  |
| 4   | Integración en banco (lazo abierto completo)    | pendiente  |
| 5   | Modelado y plataforma de 1 GDL (control fuerte) | pendiente  |
| 6   | Plataforma de 2 GDL                             | pendiente  |
| 7   | Vuelo libre                                     | pendiente  |

Cada fase se documenta en `docs/NN-nombre.md` siguiendo
`docs/plantilla-fase.md`.

---

## Cómo compilar, flashear y monitorear

Requisitos: VS Code + extensión **PlatformIO IDE**.

```bash
# Compilar (sin flashear) para cada target
pio run -e uno
pio run -e esp32

# Flashear la placa conectada
pio run -e uno   -t upload
pio run -e esp32 -t upload

# Abrir el monitor serie (115200 baudios)
pio device monitor

# Limpiar artefactos de compilación (reconstrucción desde cero)
pio run -t clean
```

En la interfaz de VS Code estos comandos también están como botones
(✓ compilar, → flashear, 🔌 monitor) en la barra inferior de PlatformIO.

---

## Estructura del repo

```
microquad/
├── platformio.ini      # doble entorno uno/esp32 (código portable)
├── README.md           # este archivo
├── .gitignore
├── src/
│   └── main.cpp        # firmware (Fase 0: blink no bloqueante + serial)
├── include/            # headers compartidos (vacío en Fase 0)
├── lib/                # librerías propias (vacío en Fase 0)
├── test/               # pruebas unitarias (opcional)
└── docs/
    ├── 00-entorno.md   # documento técnico de la Fase 0
    ├── plantilla-fase.md
    └── bitacora.md     # registro cronológico de aprendizajes
```

---

## Convención de control de versiones

- Un commit por hito demostrable (no por "guardar").
- Mensajes: `fase0: ...`, `fase1: ...` para rastrear el avance por fase.
- La carpeta `.pio/` **no** se versiona (está en `.gitignore`): es
  artefacto reproducible, no fuente.
