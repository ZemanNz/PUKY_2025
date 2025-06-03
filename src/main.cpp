#include <Arduino.h>
#include "robotka.h"
#include "smart_servo_command.h"
#include "motor_commands.h"

using namespace lx16a; // aby nebylo třeba to psát všude

void setup() {
    rkConfig cfg;
    rkSetup(cfg);

    printf("Robotka started!\n");
    pinMode(Bbutton1, INPUT_PULLUP);
    pinMode(Bbutton2, INPUT_PULLUP);
    // Pripoj jedni chytre servo s id 0
    //auto &bus = rkSmartServoBus(1);
    auto &bus = rkSmartServoBus(2);
    s_s_init(bus, 1, 50, 190);
    s_s_init(bus, 0, 10, 110);
    rkLedRed(true); // Turn on red LED
    rkLedBlue(true); // Turn on blue LED

}//230- dole 0 -nahore

void loop() {
  auto &bus = rkSmartServoBus(2);
  //auto &bus = rkSmartServoBus(1);
  if (rkButtonIsPressed(BTN_UP)) {
      s_s_move(bus, 0, 100, 80.0); // nahoru
      delay(5000);
      s_s_move(bus, 0, 10, 80.0); // nahoru
      delay(5000);
      rkServosSetPosition(1, 90); // Servo 1 nastaví na 90°
      delay(5000);
      rkServosSetPosition(1, -90); // Servo 1 nastaví na 0°
  }
  if (rkButtonIsPressed(BTN_DOWN)) {
      s_s_move(bus, 1, 190, 80.0); // dolu
      delay(5000);
      s_s_move(bus, 1, 120, 80.0); // dolu
      delay(5000);
      rkServosSetPosition(1, 90); // Servo 1 nastaví na 90°
      delay(5000);
      rkServosSetPosition(1, 0); // Servo 1 nastaví na 0°
  }
  if (rkButtonIsPressed(BTN_ON)) {//nahoru -------------zavirame 0------------ otevreny na 160
    forward(1000, 70);
  }
  if (rkButtonIsPressed(BTN_OFF)) {//dolu
    back_buttons(40);
  }
  if ((digitalRead(Bbutton1) == LOW)){
            rkLedYellow(true); // Turn on red LED
            rkLedGreen(false); // Turn on red LED
        }
  if ((digitalRead(Bbutton2) == LOW)){
            rkLedGreen(true); // Turn on red LED
            rkLedYellow(false); // Turn on red LED
        }
  delay(50);
}