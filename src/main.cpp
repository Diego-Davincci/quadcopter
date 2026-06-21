/*
 * ============================================================
 *  Fase 0 — Validación del entorno (banco de pruebas)
 * ============================================================
 *  Objetivo del programa: demostrar la cadena completa
 *  compilar -> flashear -> monitorear, en una sola base de
 *  código que funciona igual en el Arduino Uno y en el ESP32.
 *
 *  Decisiones de diseño (el "porqué"):
 *   - Parpadeo NO bloqueante con millis() en vez de delay().
 *     delay() congela el micro; un lazo de control NUNCA puede
 *     congelarse. Aquí ya practicamos el patrón temporal que
 *     usaremos para el PID en las fases de control.
 *   - LED_BUILTIN: el core de cada placa lo define solo
 *     (pin 13 en el Uno, GPIO2 en el ESP32). Código portable.
 *   - Macro F("..."): guarda las cadenas en flash (32 KB) en
 *     lugar de la SRAM (¡solo 2 KB en el Uno!). Buen hábito.
 *   - Bloques #if por arquitectura: el mismo binario fuente se
 *     autoidentifica al arrancar.
 * ============================================================
 */

#include <Arduino.h>

// ---------- Parámetros de configuración ----------
const uint32_t INTERVALO_MS = 500;  // medio periodo, parpadeo: 1hz
const uint32_t BAUDIOS = 115200;    // debe coincidir con monitor_speed

// ---------- Estado del programa ----------
uint32_t t_anterior = 0;
bool led_encendido = false;
uint32_t contador = 0;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(BAUDIOS);
    delay(200);  // breve respiro para el puerto serie

    Serial.println();
    Serial.println(F("=== Fase 0: validacion del entorno ==="));

#if defined(ARDUINO_ARCH_ESP32)
    Serial.println(F("Plataforma: ESP32 WROOM-32 (controlador final, 3.3 V)"));
    Serial.print(F("  CPU      : "));
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(F(" MHz"));
    Serial.print(F("  Flash    : "));
    Serial.print(ESP.getFlashChipSize() / (1024UL * 1024UL));
    Serial.println(F(" MB"));
    Serial.print(F("  SRAM libre: "));
    Serial.print(ESP.getFreeHeap());
    Serial.println(F(" bytes"));
#else
    Serial.println(F("Plataforma: desconocida"));
#endif

    Serial.print(F("LED_BUILTIN = "));
    Serial.println(LED_BUILTIN);
    Serial.println(F("Si ves latidos incrementando, el flujo funciona."));
    Serial.println(F("---------------------------------------------"));
}

void loop() {
    digitalWrite(LED_BUILTIN, LOW);
    // const uint32_t ahora = millis();

    // // ¿Ya pasó el intervalo? -> conmuta LED y reporta. Si no, sigue de largo.
    // if (ahora - t_anterior >= INTERVALO_MS) {
    //     t_anterior = ahora;
    //     led_encendido = !led_encendido;
    //     digitalWrite(LED_BUILTIN, led_encendido ? HIGH : LOW);

    //     Serial.print(F("t="));
    //     Serial.print(ahora);
    //     Serial.print(F(" ms | latido #"));
    //     Serial.print(contador++);
    //     Serial.print(F(" | LED="));
    //     Serial.println(led_encendido ? F("ON") : F("OFF"));
    // }
}