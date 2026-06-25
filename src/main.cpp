/*
  microquad - Fase 1 / Etapa 2: Estimador de actitud (roll y pitch)
  Plataforma: ESP32 WROOM-32  (entorno esp32, por defecto)
  Sensor:     MPU-6050 / GY-521 por I2C (SDA=GPIO21, SCL=GPIO22, AD0->GND => 0x68)

  QUE HACE
    1. Lee los 6 ejes crudos y los convierte a unidades fisicas (g y deg/s).
    2. Calibra el bias del giroscopio (sensor inmovil) y lo resta.
    3. Calcula angulo por acelerometro (gravedad) y por giroscopio (integracion).
    4. Fusiona ambos con un filtro complementario.
    5. Corre a dt fijo (200 Hz) con micros(); mide el dt real.

  La telemetria por serial imprime, en este orden, las variables utiles para
  las pruebas T3-T8 (ver docs/01-imu.md).

  NOTA DE SIGNOS: si al inclinar fisicamente el angulo fusionado se va al
  lado contrario o se dispara, invierte el signo del termino de giroscopio
  del eje afectado (gxd o gyd). Es el ajuste tipico de convencion de ejes.
*/

#include <Arduino.h>  // Trae entorno arduino (tipo, funciones de tiempo, Serial).
#include <Wire.h>  // Wire.h es la libreria de arduino que permite la comunicación I2C, genera las señales de reloj en SCL, maneja el bit de start/stop y detecta el ACK del esclavo

// Registro interno del chip donde vive el identificador WHO_AM_I
static const uint8_t MPU_ADDR = 0x68; /* AD0 - GND. Direcció del chip en el bus I2C, es el "número
                                         de casa" de la IMU en el bus compartido*/
static const uint8_t REG_WHO_AM_I =
    0X75; /* Casillero de lectura, contiene el número de identidad del chip */
static const uint8_t REG_PWR_MGMT_1 =
    0x6B; /* "Power Management 1". Controla la energía, escribir aquí despierta al sensor para que
             pueda entregar información */
static const uint8_t REG_CONFIG =
    0x1A; /* Configuración general. En el código lo usamos para el DLPF (Digital Low-Pass Filter),
             filtro digital interno que suaviza el ruido mecánico antes de que el dato salga */
static const uint8_t REG_GYRO_CONFIG =
    0x1B; /* Configuración del Giroscopio. principalmente su fondo de escala (±250, ±500, ±1000 o
             ±2000 °/s), 0x00 para dejar ±250 (velocidad angular máxima que el sensor medirá), fija
             la sensibilidad 131 LSB/°·s */
static const uint8_t REG_ACCEL_CONFIG =
    0x1C; /* Configura el Acelerómetro. fondo de escala (±2, ±4, ±8 o ±16 g), en 0x00 fijamos ±2g
             (máximo rango de aceleración a medir, 1g ≈ 9.81 m/s²) */
static const uint8_t REG_ACCEL_XOUT_H =
    0x3B; /* A partir de aquí, los 14 casilleros consecutivos contienen en orden accel X/Y/Z (6
             bytes), temperatura (2 bytes), gyro X/Y/Z (6 bytes) */

// Sensibilidades de los fondos de escala por defecto:
//   accel +/-2 g     -> 16384 LSB/g
//   gyro  +/-250 deg -> 131   LSB/(deg/s)
/*
   El sensor mide con un ADC de 16 bits con signo: cada eje te da un número entero entre -32768 y
   32768. Estos números son cuentas, no son unidades físicas. LSB significa "least significant bit",
   el escalon más pequeño que el ADC puede distinguir. ¿Cuántas cuentas equivalen a 1g? Depende del
   fondo de escala. El acelerometro está configurado en ±2g, el rango de 16bits va desde −2g a +2g.
   32768 cuentas / 2g = 16384 cuentas/g.

   Lo mismo para el giroscopio, a fondo de escala ±250 °/s, 32768 / 250 = 131 cuentas por cada °/s
   */
static const float ACC_LSB_PER_G = 16384.0f;
static const float GYR_LSB_PER_DPS = 131.0f;

// ---------- Lazo a dt fijo ----------
static const uint32_t LOOP_HZ = 200;                  // Tasa objetivo
static const uint32_t LOOP_US = 1000000UL / LOOP_HZ;  // Periodo 5000us

// ---------- Filtro complementario ----------
// alpha alto  -> mas peso al giroscopio (corto plazo, suave/rapido)
// 1 - alpha   -> peso al acelerometro   (largo plazo, corrige deriva)
// constante de tiempo:  tau = alpha*dt/(1-alpha)
static const float ALPHA = 0.98f;

// ---------- Estado ----------
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasY = 0;  // Bias del gyro (deg/s)
float rollFused = 0, pitchFused = 0;                // Angulos fusionados (deg)
float rollGyro = 0, pitchGyro = 0;                  // Solo gyro (evidencia T5)
uint32_t tPrev = 0;

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
