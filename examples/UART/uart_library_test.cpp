#include <Arduino.h>
#include <stdint.h>
#include "robotka.h"
#include "smart_servo_command.h"
 
#define RX1_PIN   16
#define TX1_PIN   17  // mimochodem nepotřebujeme, ale Arduino vyžaduje parametry

// Stejná struktura (packed implicitně)
typedef struct __attribute__((packed)) {
    uint8_t  id;
    int16_t  value;
} ServoMsg;


const uint8_t SYNC0 = 0xAA;
const uint8_t SYNC1 = 0x55;

enum RxState { WAIT_SYNC0, WAIT_SYNC1, READ_PAYLOAD };
RxState state = WAIT_SYNC0;

static const size_t PAYLOAD_SIZE = sizeof(ServoMsg);
uint8_t buf[PAYLOAD_SIZE];
size_t idx = 0;

void setup() {
  rkConfig cfg;
  rkSetup(cfg);
  
  rkLedRed(true); // Turn on red LED
  rkLedBlue(true); // Turn on blue LED
  printf("Robotka started!\n");
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
  auto &bus = rkSmartServoBus(2);
  s_s_init(bus, 1, 0, 239);
  s_s_init(bus, 0, 70, 220);
  printf("Servo 0 je na pozici %f stupnu\n", bus.pos(0).deg());
  printf("Servo 1 je na pozici %f stupnu\n", bus.pos(1).deg());
  printf("done\n");
  while (true) {
    while (Serial1.available()) {
    uint8_t c = Serial1.read();
    printf("c: %02X\n", c);
    switch (state) {
      case WAIT_SYNC0:
        if (c == SYNC0) state = WAIT_SYNC1;
        break;
      case WAIT_SYNC1:
        if (c == SYNC1) {
          state = READ_PAYLOAD;
          idx = 0;
        } else {
          // možná c je už SYNC0, zkusit znovu
          state = (c == SYNC0) ? WAIT_SYNC1 : WAIT_SYNC0;
        }
        break;
      case READ_PAYLOAD:
        buf[idx++] = c;
        if (idx >= PAYLOAD_SIZE) {
          // máme celý payload
          ServoMsg m;
          memcpy(&m, buf, PAYLOAD_SIZE);
          printf("Received: ");
          printf("ID: %d, Value: %d\n", m.id, m.value);
          // zpracování zprávy
          s_s_soft_move(bus, m.id, m.value, 50.0);
          // zpracování zprávy
          state = WAIT_SYNC0;
        }
        break;
    }
  }
  }
}

void loop() {
  
}