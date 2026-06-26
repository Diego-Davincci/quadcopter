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
/*
    "LOOP_HZ" es cuántas veces por segundo queremos correr el ciclo: 200. De ahí calculamos el
   periodo en microsegundos. Un segundo tiene 1.000.000us , divido entre 200 da 5000us entre
   iteración e iteración.
*/
static const uint32_t LOOP_HZ = 200;                  // Tasa objetivo
static const uint32_t LOOP_US = 1000000UL / LOOP_HZ;  // Periodo 5000us

// ---------- Filtro complementario ----------
// alpha alto  -> mas peso al giroscopio (corto plazo, suave/rapido)
// 1 - alpha   -> peso al acelerometro   (largo plazo, corrige deriva)
// constante de tiempo:  tau = alpha*dt/(1-alpha)
/*
    Es el peso de la fusión, significa "confia 98% en el giroscopio y 2% en el acelerómetro" en cada
   paso.
*/
static const float ALPHA = 0.98f;

// ---------- Estado ----------
/*
    Estás son las variables de estado, son globales porque deben sobrevivir a cada loop().
    gyroBias*: El sesgo del giroscopio, se mide una vez al arrancar y se resta siempre.
    rollFused, pitchFused: El ángulo fusionado, la salida real del estimador. Cada iteración lo
   actualiza partiendo de su valor previo. rollGyro, pitchGyro: El ángulo calculado solo integrando
   el gyro. tPrev: Marca del tiempo de la ultima iteración
*/
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;  // Bias del gyro (deg/s)
float rollFused = 0, pitchFused = 0;                // Angulos fusionados (deg)
float rollGyro = 0, pitchGyro = 0;                  // Solo gyro (evidencia T5)
uint32_t tPrev = 0;

// ---------- Acceso de bajo nivel al sensor ----------
/* Aquí metemos un valor en un casillero (registro) del chip; */
void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

/* Leemos un casillero */
uint8_t mpuRead8(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);  // "repeated-start" no suelta el bus
    Wire.requestFrom((int)MPU_ADDR, 1);
    return Wire.read();
}

// Lee los 14 bytes (accel, temp, gyro) en una sola rafaga.
// Importante: se leen a un buffer y luego se arman los int16, porque el
// orden de evaluacion de Wire.read() dentro de una misma expresion no esta
// garantizado en C++ y eso corromperia los bytes.
/*
   El "&" significa paso de referencia en la memoria, la función no recibe copias, sino las
   variables reales de quien las llama, y las llena directamente. Es la forma de devolver seis
   valores a la vez.
   El cuerpo le dice al chip empieza en "0x3B" y dame 14 bytes seguidos, que son accel(6) + tem(2) +
   gyro(6). Los seis ejes quedan capturados en el mismo instante.
   Luego llegamos a la recombinación, cada eje son 16 bits guardados en 2 casilleros, parte alta
   "_H" y parte baja "_L", para reconstruir el número: ax = (b[0] << 8) | b[1] b[0] << 8 corre la
   parte alta 8bits a la izquierda y | b[1] encaja la parte de debajo
 */
void mpuReadRaw(int16_t& ax, int16_t& ay, int16_t az, int16_t& gx, int16_t& gy, int16_t gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);  // Mantenemos el bus abierto
    Wire.requestFrom(int(MPU_ADDR), 14);
    uint8_t b[14];
    for (int i = 0; i < 14; i++) b[i] = Wire.read();
    ax = (int16_t)((b[0] << 8) | b[1]);
    ay = (int16_t)((b[0] << 8) | b[3]);
    az = (int16_t)((b[0] << 8) | b[5]);
    // b[6], b[7] = temperatura, no se usa en la fase 1
    gx = (int16_t)((b[0] << 8) | b[9]);
    gy = (int16_t)((b[0] << 8) | b[11]);
    gz = (int16_t)((b[0] << 8) | b[13]);
};

// Promedia n muestras en reposo: el promedio del gyro ES su bias.
/*
   Un giroscopio real, perfectamente quieto no marca cero, marca un pequeño valor constante(el
   bias). La idea para corregirlo es estadistica simple: Si el sensor esta inmovil, su velocidad
   angular real es cero. Si promediamos 2000 muestras, el ruido aleatorio se cancela. Ese promedio
   es el bias. El delay(1) espacia las muestras 1ms para que cubran un ratito de tiempo y no sean
   2000 lecturas idénticas y al final dividimos entre la sensibilidad para guardarlo en °/s;
*/
void calibrarGiroscopio(uint16_t n = 2000) {
    double sx = 0, sy = 0, sz = 0;
    int16_t ax, ay, az, gx, gy, gz;
    for (uint16_t i = 0; i < n; i++) {
        mpuReadRaw(ax, ay, az, gx, gy, gz);
        sx += gx;
        sy += gy;
        sz += gz;
        delay(1);
    }
    gyroBiasX = (float)(sx / n) / GYR_LSB_PER_DPS;
    gyroBiasY = (float)(sy / n) / GYR_LSB_PER_DPS;
    gyroBiasZ = (float)(sz / n) / GYR_LSB_PER_DPS;
};

void setup() {
    Serial.begin(115200);  // Abrimos puerto a 115200 baudios
    delay(300);  // El convertidor USB-serial (CH340) necesita alrededor de 200ms para estabilizarse
    Wire.begin(21, 22);  // SDA = GPI021, SCL = GPI022. Le dice a la libreria wire que utilice GIO21
                         // como SDA, y GIO22 como SCL lo que activa la comunicación del ESP32 para
                         // I2C mediante esos pines en modo open-drain con pull-ups
    Wire.setClock(400000);  // I2C fast mode (400kHz). Fijamos la frecuencia de SCL a 400kHz

    // T2: identidad. La MPU-6050 genuina reporta 0x68; muchos clones
    // compatibles (familia MPU-6500/9250) reportan 0x98, 0x70 o 0x71.
    // Todos comparten el mismo mapa de registros -> validos para este proyecto.
    // (Hallazgo Etapa 1: este modulo reporta 0x98.)
    uint8_t who = mpuRead8(REG_WHO_AM_I);  // La verificación de identidad tolerante a tu clon 0x98.
    Serial.printf("\n[Fase 1] WHO_AM_I = 0x%02X\n", who);
    if (who == 0x68 || who == 0x98) {
        Serial.printf("---> IMU Compatible detectada. OK");
    } else {
        Serial.printf("---> ADVERTENCIA: identidad inesperada (0x%02X).\n", who);
    }

    // Despertar: PWR_MGMT_1 = 0x01 saca del modo sleep y usa el PLL con
    // referencia del gyro X como reloj (mas estable que el oscilador interno).
    mpuWrite(REG_PWR_MGMT_1, 0x01);  // Despertamos el chip. La IMU arranca dormida; este registro
                                     // la activa y le asigna el reloj.
    delay(100);
    /* Fijamos los fondos de escala (para justificar las senbilidades previamente fijadas) y activa
     * el filtro pasa-bajos interno para evitar el ruido mecánico */
    mpuWrite(REG_ACCEL_CONFIG, 0x00);  // +/-2 g
    mpuWrite(REG_GYRO_CONFIG, 0x00);   // +/-250 °/s
    mpuWrite(REG_CONFIG, 0x03);        // DLPF ~44 Hz: filtra ruido mecanico
    delay(50);

    Serial.println("Calibrando el giroscopio... manten el sensor totalmente quieto");
    calibrarGiroscopio();  // Mide el bias
    Serial.printf("Bias gyro [°/s]: X=%.3f Y=%.3f Z=%.3f \n", gyroBiasX, gyroBiasY, gyroBiasZ);

    // Semilla: inicializa el angulo fusionado con el del acelerometro para
    // arrancar sin un salto inicial.
    /*
        Antes de entrar al lazo, inicializa el ángulo fusionado con lo que dice el acelerometro
       ahora mismo. Si arrancaramos desde cero y el sensor estuviera inclinado, veríamos el ángulo
       subir desde 0° hasta el valor real en los primeros segundos. Sembrandólo con el accel, el
       ángulo arranca correcto sin salto.
    */
    int16_t ax, ay, az, gx, gy, gz;
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    float axg = ax / ACC_LSB_PER_G, ayg = ay / ACC_LSB_PER_G, azg = az / ACC_LSB_PER_G;
    rollFused = atan2f(ayg, azg) * 180.0f / PI;
    pitchFused = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * 180.0f / PI;

    rollGyro = rollFused;
    pitchGyro = pitchFused;

    Serial.println("Melo, Telemetria abajo (separada por '|')");
    tPrev = micros();
}

void loop() {
    // --- Lazo a dt fijo, no bloqueante (semilla del tiempo de muestreo) ---
    uint32_t now = micros();
    if ((uint32_t)(now - tPrev) < LOOP_US) return;
    float dt = (now - tPrev) * 1e-6f;  // dt real medido (prueba T8)
    tPrev = now;

    int16_t ax, ay, az, gx, gy, gz;
    mpuReadRaw(ax, ay, az, gx, gy, gz);

    // Conversion a unidades fisicas
    float axg = ax / ACC_LSB_PER_G;
    float ayg = ay / ACC_LSB_PER_G;
    float azg = az / ACC_LSB_PER_G;
    float gxd = gx / GYR_LSB_PER_DPS - gyroBiasX;  // °/s, sin bias
    float gyd = gy / GYR_LSB_PER_DPS - gyroBiasY;
    // gz no se usa: el yaw no es observable sin magnetometro

    // Angulo por acelerometro (referencia absoluta, ruidosa)
    float rollAcc = atan2f(ayg, azg) * 180.0f / PI;
    float pitchAcc = atan2f(-axg, sqrtf(ayg * ayg + azg * azg)) * 180.0f / PI;

    // Angulo por giroscopio solo (integracion pura: deriva, para T5)
    rollGyro += gxd * dt;
    pitchGyro += gyd * dt;

    // Fusion: filtro complementario
    rollFused = ALPHA * (rollFused + gxd * dt);
}