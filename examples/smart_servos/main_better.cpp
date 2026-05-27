#include <Arduino.h>
#include "robotka.h"
#include "smart_servo_command.h"
using namespace lx16a;

void setup() {
    rkConfig cfg;
    cfg.motor_max_power_pct = 90; // limit the power
    rkSetup(cfg);
    printf("Robotka started!\n");
    
    rkLedRed(true); // Turn on red LED
    rkLedBlue(true); // Turn on blue LED
    auto &bus = rkSmartServoBus(2);
    s_s_init(bus, 1, 0, 239);
    s_s_init(bus, 0, 50, 160);
    printf("Servo 0 je na pozici %f stupnu\n", bus.pos(0).deg());
    printf("Servo 1 je na pozici %f stupnu\n", bus.pos(1).deg());
    s_s_move(bus, 1, 160, 50.0);
    delay(5000);
    s_s_move(bus, 1, 0, 50.0);
    delay(1000);
    s_s_soft_move(bus, 0, 220, 30.0);
    delay(5000);  
    s_s_soft_move(bus, 0, 70, 30.0);
}
void loop() {
}