#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "robotka.h"

// deklarace instanci senzoru
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxBottom = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  rkConfig cfg; rkSetup(cfg);
  delay(50);
  pinMode(21, PULLUP);
  pinMode(22, PULLUP);
  pinMode(14, PULLUP);
  pinMode(26, PULLUP);
  // 1) Spust obě I2C sběrnice
  Wire.begin(14, 26, 400000);
  Wire.setTimeOut(1);
  Wire1.begin(21, 22, 400000);
  Wire1.setTimeOut(1);
  // 2) Inicializuj senzory:
  rk_laser_init("front",  Wire1, loxFront,  27, 0x30);
  rk_laser_init("bottom", Wire,  loxBottom, 33, 0x31);
  Serial.println("Scanning I2C WIRE1...");
    for (byte addr = 1; addr < 127; addr++) {
      Wire1.beginTransmission(addr);
      if (Wire1.endTransmission() == 0) {
        Serial.print("  Found: 0x");
        Serial.println(addr, HEX);
      }
    }
    Serial.println("Scan complete.\n");

    Serial.println("Scanning I2C WIRE...");
    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("  Found: 0x");
        Serial.println(addr, HEX);
      }
    }
}

void loop() {
  int d1 = rk_laser_measure("front");
  int d2 = rk_laser_measure("bottom");
  
    Serial.println("Scan complete.\n");
  Serial.print("Front:  "); Serial.print(d1>=0?String(d1):"Err"); Serial.print(" mm, ");
  Serial.print("Bottom: "); Serial.print(d2>=0?String(d2):"Err"); Serial.println(" mm");

  delay(100);
}
