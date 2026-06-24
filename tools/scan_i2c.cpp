/*
  microquad - Fase 1 / Etapa 1: Escaner I2C + WHO_AM_I
  Plataforma: ESP32 WROOM-32

  PROPOSITO
  - Confirmar que la GY-521 esta presente en el bus (prueba T1).
  - Confirmar su identidad leyendo el registro WHO_AM_I (prueba T2).

  COMO USARLO
  Este archivo vive en tools/ para que PlatformIO NO lo compile junto al
  estimador. Para la Etapa 1, copia su contenido a src/main.cpp, flashea,
  verifica T1/T2 en el monitor serie (115200), y luego restaura el
  src/main.cpp del estimador para la Etapa 2.
*/

#include <Arduino.h>  // Trae entorno arduino (tipo, funciones de tiempo, Serial).
#include <Wire.h>  // Wire.h es la libreria de arduino que permite la comunicación I2C, genera las señales de reloj en SCL, maneja el bit de start/stop y detecta el ACK del esclavo

static const uint8_t MPU_ADDR = 0x68;  // ADO -> GND. Dirección I2C del MPU6050
static const uint8_t REG_WHO_AM_I =
    0x75;  // Registro interno del chip donde vive el identificador WHO_AM_I

void setup() {
    Serial.begin(115200);  // Abrimos puerto a 115200 baudios
    delay(300);  // El convertidor USB-serial (CH340) necesita alrededor de 200ms para estabilizarse
    Wire.begin(21, 22);  // SDA = GPI021, SCL = GPI022. Le dice a la libreria wire que utilice GIO21
                         // como SDA, y GIO22 como SCL lo que activa la comunicación del ESP32 para
                         // I2C mediante esos pines en modo open-drain con pull-ups
    Wire.setClock(400000);  // I2C fast mode (400kHz). Fijamos la frecuencia de SCL a 400kHz
    Serial.println("\n[Fase 1 / Etapa 1] Escaner I2C");
}

void loop() {
    Serial.println("Escaneando bus I2C...");
    uint8_t encontrados = 0;
    /*
        I2C tiene direcciones de 7bits = 128 valores posibles (0x00, 0x7F). 0x00 es la dirección del
        broadcast en general y los valores 0x78-0x7F estan reservados por estándar, el rango util
        del bucle es 1-126. Llamamos a cada puerta y vemos quien contesta
    */
    for (uint8_t addr = 1; addr < 127; addr++) {
        /*
            1. START Condition: el ESP32 jala del SDA hacía abajo mientras SCL esta en alto. todos
           los esclavos del bus la detectan y se ponen en modo escucha.
            2. Dirección + bit de escritura: el ESP32 transmite 7bits de addr seguidor de un 0.
            3. Bit 9 - ACK: el ESP32 suelta el SDA, si existe un esclavo con esa dirección el jala
            SDA a bajo en ese ciclo de reloj. Eso es ACK, si nadie jala, la línea queda en alto =
            NACK (No-Acknowledge).
           4. STOP Contidion: el ESP32 jala SDA hacia abajo, luego libera SCL, luego libera SDA. Esa
           secuencia cierra la transacción.
        */
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {  // 0 = ACK del esclavo
            Serial.printf("Dispositivo en 0x%02X\n", addr);
            encontrados++;
        }
    }

    if (encontrados == 0) {
        Serial.println("No se ha detectado el MPU6050, revisa cableado y GND");
    } else {
        // T2: lectura de WHO_AM_I en la direccion esperada
        // Si encontramos el dispositivo, hacemos una segunda verificación, debe contestar nuestra
        // MPU6050
        Wire.beginTransmission(MPU_ADDR);
        /*
            ponemos 0x75 en el buffer de transmisión. Al llegar endTransmission(false), el ESP32
           envia al esclavo:
            - START + dirección 0x68 + bit escritura = ACK
            - byte 0x75 (número de registro que queremos leer) = ACK
            El argumento false le dice que no envie STOP al final, esto genera un 'repeated START',
           el bus queda ocupado con 0x68, le decimos que va a continuar con una lectura sin soltar
           el bus.

            Wire.requestFrom((int)MPU_ADDR, 1), Aquí el ESP32 hace una transacción de lectura:
            - Repeated START + dirección 0x68 + bit de lectura (1) = ACK del esclavo
            - El esclavo manda 1 byte (contenido del registro 0x75)
            - El ESP32 responde con NACK (señal de que ya no quiere más bytes) + STOP
        */
        Wire.write(REG_WHO_AM_I);
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)MPU_ADDR, 1) == 1) {
            // El registro WHO_AM_I de la MPU6050 siempre contiene 0x68, el el número de
            // identificación del chip
            uint8_t who = Wire.read();
            Serial.printf("  WHO_AM_I (0x75) en 0x68 = 0x%02X (esperado 0x68)\n", who);
            if (who == 0x98) {
                Serial.printf("Puerto registrado 0x98, El MPU6050 utilizado es una replica");
            }
        }
    }

    Serial.println("---");
    delay(2000);
}
