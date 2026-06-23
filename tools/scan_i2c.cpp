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

#include <Arduino.h>
#include <Wire.h>

static const uint8_t MPU_ADDR = 0x68;  // ADO -> GND
static const uint8_t REG_WHO_AM_I = 0x75;

void setup() {
    Serial.begin(115200);
    delay(300);
    Wire.begin(21, 22);     // SDA = GPI021, SCL = GPI022
    Wire.setClock(400000);  // I2C fast mode (400kHz)
    Serial.println("\n[Fase 1 / Etapa 1] Escaner I2C");
}

void loop() {
    Serial.println("Escaneando bus I2C...");
    uint8_t encontrados = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
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
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_WHO_AM_I);
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)MPU_ADDR, 1) == 1) {
            uint8_t who = Wire.read();
            Serial.printf("  WHO_AM_I (0x75) en 0x68 = 0x%02X (esperado 0x68)\n", who);
        }
    }

    Serial.println("---");
    delay(2000);
}
