#include <Arduino.h>

#include "robotka.h"

using namespace lx16a; // aby nebylo třeba to psát všude

void setup() {
    rkConfig cfg;
    cfg.motor_max_power_pct = 90; // limit the power
    rkSetup(cfg);

    printf("Robotka started!\n");

    // Pripoj jedni chytre servo s id 0
    //auto &bus = rkSmartServoBus(1);
    auto &bus = rkSmartServoBus(2);
    printf("Servo 0 je na pozici %f stupnu\n", bus.pos(0).deg());
    //bus.set(0, Angle::deg(0));
    //bus.set(1, Angle::deg(0));
    printf("Servo 0 je na pozici %f stupnu\n", bus.pos(0).deg());
    printf("Servo 1 je na pozici %f stupnu\n", bus.pos(1).deg());
    delay(1000);
    rkLedRed(true); // Turn on red LED
    rkLedBlue(true); // Turn on blue LED
    bus.set(0, Angle::deg(10));
    int a=10;
    while(a<150) {
        bus.set(0, Angle::deg(a));
        a+=10;
        delay(500);
        printf("klepeta je na pozici %f stupnu\n", bus.pos(0).deg());
    }
    bus.set(0, Angle::deg(160));
    delay(1000);
    while(a>10) {
        bus.set(0, Angle::deg(a));
        a-=10;
        delay(500);
        printf("klepeta je na pozici %f stupnu\n", bus.pos(0).deg());
    }
    bus.set(0, Angle::deg(a));
    delay(1000);

}//230- dole 0 -nahore

void loop() {
  auto &bus = rkSmartServoBus(0);
  //auto &bus = rkSmartServoBus(1);
  if (rkButtonIsPressed(BTN_UP)) {
      bus.set(0, Angle::deg(0));
      bus.set(1, Angle::deg(0));
  }
  if (rkButtonIsPressed(BTN_DOWN)) {
      bus.set(0, Angle::deg(230));
      bus.set(1, Angle::deg(230));
  }
  if (rkButtonIsPressed(BTN_ON)) {//nahoru -------------zavirame 0------------ otevreny na 160
    uint8_t pos = 230;
    while(pos > 10) {
        printf("Test-point-2");
        bus.set(0, Angle::deg(pos));
        bus.set(1, Angle::deg(pos));
        pos -= 10;
        delay(500);
        printf("Servo 0 je na pozici %f stupnu\n", bus.pos(0).deg());
        printf("Servo 1 je na pozici %f stupnu\n", bus.pos(1).deg());
    }
  }
  if (rkButtonIsPressed(BTN_OFF)) {//dolu
    uint8_t pos = 0;
    while(pos < 220) {
      printf("Test-point-2");
      bus.set(0, Angle::deg(pos));
      bus.set(1, Angle::deg(pos));
      pos += 10;
      delay(500);
      printf("Servo 0 je na pozici %f stupnu\n", bus.pos(0).deg());
      printf("Servo 1 je na pozici %f stupnu\n", bus.pos(1).deg());
    }
  }
  delay(50);
}