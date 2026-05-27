
#include <Arduino.h>

const uint8_t SERVO_COUNT = 4;
const uint8_t servoPins[SERVO_COUNT] = { 13, 12, 14, 27 };

// zpracuje „1_-80“, „2_45“…
void processCommand(const String &cmd) {
  int sep = cmd.indexOf('_');
  if (sep < 1) {
    Serial.println("Chybný formát, očekávám '<id>_<úhel>'");
    return;
  }

  int id  = cmd.substring(0, sep).toInt();
  int ang = cmd.substring(sep + 1).toInt();

  if (id < 0 || id >= SERVO_COUNT) {
    Serial.println("Chyba: id mimo rozsah");
    return;
  }
  if (ang < -90 || ang > 90) {
    Serial.println("Chyba: úhel mimo rozsah");
    return;
  }
  int pwmAngle = ang + 90;
  Serial.printf("Servo %d na %d° (pwm %d)\n", id, ang, pwmAngle);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("Pošli příkaz ve tvaru: <id>_<úhel> a stiskni Enter");
}

void loop() {
  // Pokud je aspoň jeden znak k dispozici, přečteme až po '\n'
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();               // odstraní CR/LF a mezery navíc
    if (line.length() > 0) {
      Serial.print("Dostal jsem řetězec: "); 
      Serial.println(line);
      processCommand(line);
    }
  }
}

