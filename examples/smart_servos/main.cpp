#include <Arduino.h>
#include "SmartServoBus.hpp"

using namespace lx16a;

// Funkce pro získání instance SmartServoBus
lx16a::SmartServoBus& rkSmartServoBus(uint8_t servo_count) {
    static SmartServoBus servoBus;
    static bool initialized = false;

    if (!initialized) {
        // Inicializace sběrnice pro chytrá serva
        servoBus.begin(servo_count, UART_NUM_2, GPIO_NUM_14); // UART2, GPIO14 jako pin
        initialized = true;
    }

    return servoBus;
}

void setup() {
    Serial.begin(115200);

    // Inicializace chytrých serv (např. 1 servo na sběrnici)
    auto& servoBus = rkSmartServoBus(1);

    // Nastavení pozice serva
    servoBus.set(0, Angle::deg(90)); // Servo ID 0 nastaví na 90°
    delay(1000);

    servoBus.set(0, Angle::deg(0)); // Servo ID 0 nastaví na 0°
    delay(1000);

    servoBus.set(0, Angle::deg(180)); // Servo ID 0 nastaví na 180°
    delay(1000);
}

void loop() {
    // Servo zůstává na poslední pozici
}