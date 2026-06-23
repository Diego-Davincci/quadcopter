### Qué es PWM

Modulación por Ancho de Pulso, sirve para simular voltajes analógicos utilizando señales digitales

### Extensión .cp

Extensión para código fuente de C++

### Qué son los baudios?

El ESP32 cuenta con una velocidad de mensajes a 115200 baudios. Los baudios representan la velocidad de transmisión por segundo(bps) en una comunicación serie(UART).

# Fase 0 - Blinquear LED

Se necesita primer configurar platformio.ini para que pueda entender nuestra placa principal (esp32) y poder correr los programas en vscode.
En el archivo src/main.cpp, se corre el código principal utilizando c++ y la estructura misma de arduino. El archivo contiene 2 funciones, "setup" en donde levantamos nuestra configuración inicial y "loop" el cuál será nuestro programa que corre en ciclos continuamente de la misma manera que lo hace arduino

# Faseo 1 - roll & pitch para el MPU6050

Necesitamos verificar roll y pitch limpios a tasa fija, ese estimador es la entrada fija a nuestro proceso de medición de nuestro proceso de control.

La MPU6050: es una IMU(unidad de medición de inercial) con 6 grados de libertad. Combina un acelerómetro de 3 ejes y un giroscopio de 3 ejes un un mismo circuito encapsulado en ambos MEMS(estructuras micromecánicas grabadas en silicio). El acelerómetro en reposo mide el vector gravedad que apunta "hacia abajo". El giroscopio mide la velocidad angular.

La GY-521: placa de ruptura ("breakout") que lleva la MPU6050, le añade un regulador de 3.3V, resistencias de pull-up para el bus I2C. Hablamos con ella mediante I2C, un bus de dos hilos(SDA=Datos, SCL=reloj) donde el ESP32 es el maestro y la IMU es el esclavo con dirección propia.

La comunicación I2C: Creada por Philips, permite que multiples dispositivos o "chips" se comuniquen entre si a corta distancia. Hay dos lineas o cables principales para transferir información. SDA(Línea de Datos): Por donde se envia y reciben datos entre los dispositivos. SCL(Línea de Reloj): La señal de sincronización generada por el controlador para marcar el ritmo al que se transmiten los datos. Maestro: Dispositivo(algún tipo de microcontrolador como Arduino o ESP32), que inicia la comunicación, genera la señal de reloj y controla el flujo de datos. Esclavo: Son los periféricos(sensores, módulos de pantalla, memorias) que simplemente responden cuando el maestro los llama o solicitan enviar datos.

ACK del esclavo: Acknowledge o acuse de recibo, es una señal de confirmación que emite un dispositivo esclavo en protocolos de comunicación serie (como I2C) para indicarle al maestro que ha recibido exitosamente un byte de datos o que esta listo para la siguiente operación.
