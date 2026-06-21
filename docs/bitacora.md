### Qué es PWM

Modulación por Ancho de Pulso, sirve para simular voltajes analógicos utilizando señales digitales

### Extensión .cp

Extensión para código fuente de C++

### Qué son los baudios?

El ESP32 cuenta con una velocidad de mensajes a 115200 baudios. Los baudios representan la velocidad de transmisión por segundo(bps) en una comunicación serie(UART).

# Fase 0 - Blinquear LED

Se necesita primer configurar platformio.ini para que pueda entender nuestra placa principal (esp32) y poder correr los programas en vscode.
En el archivo src/main.cpp, se corre el código principal utilizando c++ y la estructura misma de arduino. El archivo contiene 2 funciones, "setup" en donde levantamos nuestra configuración inicial y "loop" el cuál será nuestro programa que corre en ciclos continuamente de la misma manera que lo hace arduino
