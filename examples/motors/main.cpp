#include "motor_commands.h"
#include "robotka.h"
void setup() {
    Serial.begin(115200);

    // Inicializace robotky
    rkConfig cfg;
    rkSetup(cfg);

    // Nastavení tlačítek
    pinMode(Bbutton1, INPUT_PULLUP);
    pinMode(Bbutton2, INPUT_PULLUP);

    Serial.println("Motor example started!");
}

void loop() {
    // Kontrola stavu tlačítek
    if (rkButtonIsPressed(BTN_ON)) {
        Serial.println("Button ON pressed");
        rkLedRed(true); // Turn on red LED
        delay(1000); // Wait for 500 milliseconds
        forward(-2000, 70); // Call a motor function
        delay(500); // Wait for the motor function to execute
        rkLedRed(false); // Turn off red LED
    } else if (rkButtonIsPressed(BTN_RIGHT)) {
        Serial.println("Button OFF pressed");
        rkLedBlue(true); // Turn on blue LED
        delay(1000); // Wait for 500 milliseconds
        forward(2000, 40); // Call a motor function
        delay(500); // Wait for the motor function to execute
        rkLedBlue(false); // Turn off blue LED
    } else if (rkButtonIsPressed(BTN_UP)) {
        Serial.println("Button UP pressed");
        rkLedGreen(true); // Turn on green LED
        delay(1000); // Wait for 500 milliseconds
        radius_r(-360, 200, 40); // Call a radius turn function
        delay(500); // Wait for the motor function to execute
        rkLedGreen(false); // Turn off green LED
    } else if (rkButtonIsPressed(BTN_DOWN)) {
        Serial.println("Button DOWN pressed");
        rkLedYellow(true); // Turn on yellow LED
        delay(1000); // Wait for 500 milliseconds
        radius_r(360, 200, 40); // Call a radius turn function
        delay(500); // Wait for the motor function to execute
        rkLedYellow(false); // Turn off yellow LED
    } else if (rkButtonIsPressed(BTN_LEFT)) {
        Serial.println("Button LEFT pressed");
        rkLedAll(true); // Turn on all LEDs
        delay(1000); // Wait for 500 milliseconds
        turn_on_spot(360); // Call a turn on spot function
        delay(500); // Wait for the motor function to execute
        rkLedAll(false); // Turn off all LEDs
    }
    delay(100); // Small delay to prevent busy waiting
}