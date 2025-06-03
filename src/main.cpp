#include <Arduino.h>
#include "robotka.h"
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "smart_servo_command.h"
#include "motor_commands.h"
#include "esp_system.h" // pro esp_restart()

using namespace lx16a; // aby nebylo třeba to psát všude
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
float r1, g1, b1;
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
float r2, g2, b2;
void open_l_box(){
  rkServosSetPosition(1, 20); // Servo 1 nastaví pnuti po 3minutach 90°
  delay(500);
}
void open_r_box(){
  rkServosSetPosition(2, -20); // Servo 1 nastaví pnuti po 3minutach 90°
  delay(500);
}
void close_l_box(){
  rkServosSetPosition(1, -60); // Servo 1 nastaví pnuti po 3minutach 90°
  delay(500);
}
void close_r_box(){
  rkServosSetPosition(2, 70); // Servo 1 nastaví pnuti po 3minutach 90°
  delay(500);
}
enum Barva { RED, BLUE};

struct DetekceBarvy {
    bool je_tam;
    Barva barva;
};

// Funkce pro detekci barvy ze senzoru
DetekceBarvy Detekce_puku() {
    DetekceBarvy vysledek;
    if (rkColorSensorGetRGB("puky", &r1, &g1, &b1)) {
         Serial.print("R: "); Serial.print(r1, 3);
         Serial.print(" G: "); Serial.print(g1, 3);
         Serial.print(" B: "); Serial.println(b1, 3);
        if ((r1 -28) > b1 && (r1-28) > g1) {
            vysledek.barva = RED;
            vysledek.je_tam = true;
        } else if ((b1 -28) > r1 && (b1-28) > g1) {
            vysledek.barva = BLUE;
            vysledek.je_tam = true;
        }
        else{
          vysledek.je_tam = false;
        }
    } else {
      vysledek.je_tam = false;
    }
    return vysledek;
}

DetekceBarvy Kde_jsme() {
    DetekceBarvy vysledek;
    if (rkColorSensorGetRGB("zem", &r2, &g2, &b2)) {
        Serial.print("R: "); Serial.print(r2, 3);
        Serial.print(" G: "); Serial.print(g2, 3);
        Serial.print(" B: "); Serial.println(b2, 3);
        if ((r2 - 20) > b2 && (r2 - 20) > g2) {
            vysledek.barva = RED;
            vysledek.je_tam = true;
        } else if ((b2 - 20) > r2 && (b2 - 20) > g2) {
            vysledek.barva = BLUE;
            vysledek.je_tam = true;
        } else {
            vysledek.je_tam = false;
        }
    } else {
        vysledek.je_tam = false;
    }
    return vysledek;
}
void puk_do_l_boxu(){
  auto &bus = rkSmartServoBus(1);
  s_s_move(bus, 0, 150, 80.0);
}
void puk_do_r_boxu(){
  auto &bus = rkSmartServoBus(1);
  s_s_move(bus, 0, 90, 80.0);
}
void zavreni_dvirek(){
  auto &bus = rkSmartServoBus(1);
  s_s_move(bus, 0, 120, 80.0); // nahoru
}
void StopTask(void *pvParameters) {
    vTaskDelay(2.8 * 60 * 1000 / portTICK_PERIOD_MS); // 3 minuty
    Serial.println("Uplynuly 3 minuty, restartuji program...");
    esp_restart(); // Restartuje ESP32
    vTaskDelete(NULL);
}
void chytej_puky(){
  auto now = Detekce_puku();
     if(now.je_tam){
      if (now.barva == RED) {
            Serial.println("Detekován červený puk!");
            puk_do_l_boxu();
            delay(1500);
            zavreni_dvirek();
        } else if (now.barva == BLUE) {
            Serial.println("Detekován modrý puk!");
            puk_do_r_boxu();
            delay(1500);
            zavreni_dvirek();// mala mezera, puk neprojde... 
                }
    } else {
       Serial.println("Puk nenalezen.");
    }
}
void setup() {
  Serial.begin(115200);
    rkConfig cfg;
    rkSetup(cfg);

    printf("Robotka started!\n");
    pinMode(Bbutton1, INPUT_PULLUP);
    pinMode(Bbutton2, INPUT_PULLUP);
    // Pripoj jedni chytre servo s id 0
    //auto &bus = rkSmartServoBus(1);
    auto &bus = rkSmartServoBus(1);
    s_s_init(bus, 0, 0, 240);
    rkLedRed(true); // Turn on red LED
    rkLedBlue(true); // Turn on blue LED
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
    // Initialize the color sensor with a unique name and the I2C bus
    rkColorSensorInit("puky", Wire1, tcs1);
    rkColorSensorInit("zem", Wire, tcs2);

}//230- dole 0 -nahore

void loop() {
  auto &bus = rkSmartServoBus(1);
  if (rkButtonIsPressed(BTN_UP)) {
      delay(3000);
      int max_distance_enc = 20000;
      xTaskCreate(StopTask, "StopTask", 2048, NULL, 1, NULL); // Spustí task pnuti po 3minutach pozadí pnuti po 3minutach
      setMotorsPower(20000, 20000); // Oba motory vpřed
      int distance1 = rkUltraMeasure(1);
      int distance2 = rkUltraMeasure(2);
      Serial.printf("Vzdálenosti: 1=%d mm, 2=%d mm\n", distance1, distance2);
      while(((distance1 > 150) || (distance1 <20)) && ((distance2 > 150) || (distance2<20))) {
          if(encodery() > max_distance_enc){
            break;
          }
          chytej_puky();
          delay(100);
          distance1 = rkUltraMeasure(1);
          distance2 = rkUltraMeasure(2);
          Serial.printf("Vzdálenosti: 1=%d mm, 2=%d mm\n", distance1, distance2);
      }
      Serial.println("Cesta dokončena, zastavuji motory.");
      setMotorsPower(0, 0); // Zastaví motory
  }

  // if (rkButtonIsPressed(BTN_UP)) {
  //     s_s_move(bus, 0, 150, 80.0); // nahoru
  //     delay(5000);
  //     s_s_move(bus, 0, 90, 80.0); // nahoru
  //     delay(5000);
  //     s_s_move(bus, 0, 120, 80.0); // nahoru
  // }
  // if (rkButtonIsPressed(BTN_DOWN)) {
  //     // for(int i=25; i > -70; i-=5) {
  //     //     rkServosSetPosition(1, i); // Servo 1 nastaví pnuti po 3minutach 90°
  //     //     delay(500);
  //     // }
  //     open_l_box();
  //     delay(5000);
  //     close_l_box();
  // }
  // if (rkButtonIsPressed(BTN_ON)) {//nahoru -------------zavirame 0------------ otevreny pnuti po 3minutach 160
  //   // for(int i=-25; i < 80; i+=5) {
  //   //     rkServosSetPosition(2, i); // Servo 1 nastaví pnuti po 3minutach 90°
  //   //     delay(500);
  //   // }
  //   open_r_box();
  //   delay(5000);
  //   close_r_box();
  // }
  // if (rkButtonIsPressed(BTN_OFF)) {//dolu
  //   int stop_time= 1000000;
  //   for(int i=0; i < stop_time; i+=300) {
  //       int distance1 = rkUltraMeasure(1);
  //       int distance2 = rkUltraMeasure(2);
  //       int distance3 = rkUltraMeasure(3);
  //       int distance4 = rkUltraMeasure(4);
  //       printf("Vzdálenosti: 1=%d mm, 2=%d mm, 3=%d mm, 4=%d mm\n", distance1, distance2, distance3, distance4);
  //       delay(300);
  //   }
  // }
  // if ((digitalRead(Bbutton1) == LOW)){
  //           rkLedYellow(true); // Turn on red LED
  //           rkLedGreen(false); // Turn on red LED
  //           while(true){
  //               if (rkColorSensorGetRGB("puky", &r1, &g1, &b1)) {
  //                 Serial.print("R: "); Serial.print(r1, 3);
  //                 Serial.print(" G: "); Serial.print(g1, 3);
  //                 Serial.print(" B: "); Serial.println(b1, 3);
  //               } else {
  //                 Serial.println("Sensor 'puky' not found.");
  //               }
  //               delay(1000); // Wait for a second before the next reading
  //           }
  //       }
  // if ((digitalRead(Bbutton2) == LOW)){
  //           rkLedGreen(true); // Turn on red LED
  //           rkLedYellow(false); // Turn on red LED
  //           while(true){
  //               if (rkColorSensorGetRGB("zem", &r2, &g2, &b2)) {
  //                   Serial.print("R: "); Serial.print(r2, 3);
  //                   Serial.print(" G: "); Serial.print(g2, 3);
  //                   Serial.print(" B: "); Serial.println(b2, 3);
  //               } else {
  //                   Serial.println("Sensor 'zem' not found.");
  //               }
  //               delay(1000); // Wait for a second before the next reading
  //           }
  //       }
  delay(50);
}