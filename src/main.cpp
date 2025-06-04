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

//////////////
int max_puck_per_box = 8;
int puck_in_l_box = 0;
int puck_in_r_box = 0;
struct Pozice {
    int x;
    int y;
};
int stred_od_zadu = 250;
int stred_od_predu = 140;
int stred_od_boku = 150;

// Globální proměnná pro aktuální pozici
Pozice aktualni_pozice = {350 , 350};

///////////

void open_l_box(){
  rkServosSetPosition(1, 28); // Servo 1 nastaví pnuti po 3minutach 90°
  delay(500);
}
void open_r_box(){
  rkServosSetPosition(2, -28); // Servo 1 nastaví pnuti po 3minutach 90°
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
TaskHandle_t chytejPukyHandle = NULL; // Globální handle pro task

enum Barva { RED, BLUE};

struct DetekceBarvy {
    bool je_tam;
    Barva barva;
};

// Funkce pro detekci barvy ze senzoru
DetekceBarvy Detekce_puku() {
    DetekceBarvy vysledek;
    rkColorSensorGetRGB("puky", &r1, &g1, &b1);
    delay(20);
    rkColorSensorGetRGB("puky", &r1, &g1, &b1);
    delay(20);
    rkColorSensorGetRGB("puky", &r1, &g1, &b1);
    delay(20);
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
    rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    delay(20);
    rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    delay(20);

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
  s_s_move(bus, 0, 170, 180.0);
}
void puk_do_r_boxu(){
  auto &bus = rkSmartServoBus(1);
  s_s_move(bus, 0, 70, 180.0);
}
void zavreni_dvirek(){
  auto &bus = rkSmartServoBus(1);
  s_s_move(bus, 0, 125, 130.0); // nahoru
  delay(200);
}
void StopTask(void *pvParameters) {
    vTaskDelay(2.8 * 60 * 1000 / portTICK_PERIOD_MS); // 3 minuty
    Serial.println("Uplynuly 3 minuty, restartuji program...");
    esp_restart(); // Restartuje ESP32
    vTaskDelete(NULL);
}


void ChytejPukyTask(void *pvParameters) {
    while (true) {
        auto now = Detekce_puku();
        if (now.je_tam) {
            if (now.barva == RED) {
              if( puck_in_l_box < max_puck_per_box) {
                Serial.println("Detekován červený puk!");
                puk_do_l_boxu();
                setMotorsPower(0, 0); // zastaví motory
                delay(300);
                rkMotorsDrive(60, 60, 100, 100);
                zavreni_dvirek();
                delay(200);
                setMotorsPower(20000, 20000); // zastaví motory
                puck_in_l_box++;
                Serial.printf("Puky v levém boxu: %d\n", puck_in_l_box);
                }
            } else if (now.barva == BLUE) {
                if( puck_in_r_box < max_puck_per_box) {
                  Serial.println("Detekován modrý puk!");
                  puk_do_r_boxu();
                  setMotorsPower(0, 0); // zastaví motory
                  delay(300);
                  rkMotorsDrive(60, 60, 100, 100);
                  zavreni_dvirek();
                  delay(200);
                  setMotorsPower(20000, 20000); // zastaví motory
                  puck_in_r_box++;
                }
            }
        } else {
            Serial.println("Puk nenalezen.");
        }
        vTaskDelay(200 / portTICK_PERIOD_MS); // malá pauza, aby task nebyl příliš rychlý
    }
    vTaskDelete(NULL); // kdyby někdy skončil
}
float M_wheel_circumference = 127.0f * PI; // Průměr kola v mm * PI
int32_t MmToTicks(float mm){
    return (mm / M_wheel_circumference) * 38.55937f * 48.f;
}

float TicksToMm(int32_t ticks) {
    return float(ticks) / 38.55937f / 48.f * M_wheel_circumference;
}

int Odchylka = 0, Integral = 0, Last_odchylka = 0;
void zkontroluj_pid(int power, int M1_pos, int M4_pos){
        int Max_integral = 1000;
        Odchylka = M1_pos - M4_pos;
        Integral += Odchylka;
        if (Integral >  Max_integral) Integral =  Max_integral;
        if (Integral < -Max_integral) Integral = -Max_integral;
        int rawPower = power + Odchylka * Kp + Integral * (Ki+2) + (Odchylka - Last_odchylka) * Kd;
        // saturace
        const int maxPower = 32000;
        if      (rawPower >  maxPower) rawPower =  maxPower;
        else if (rawPower < -maxPower) rawPower = -maxPower;
        auto& man = rb::Manager::get();
        Odchylka = M1_pos - M4_pos;
        Integral += Odchylka;
        man.motor(rb::MotorId::M1).power(-power * 0.9185f);//
        man.motor(rb::MotorId::M4).power(rawPower);
        Last_odchylka = Odchylka;
}




void modra(){
  Serial.println("Modra barva");
  auto& man = rb::Manager::get(); // vytvoří referenci na man class
  man.motor(rb::MotorId::M1).setCurrentPosition(0);
  man.motor(rb::MotorId::M4).setCurrentPosition(0);
  int M4_pos = 0;
  int M1_pos = 0;
  man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
          M4_pos = info.position();
      });
  man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
         M1_pos = -info.position();
      });
  xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);
  int distance1 = 1600;
  Serial.printf("musi_jet::: M1_pos: %d, M4_pos: %d\n", MmToTicks(distance1), MmToTicks(distance1));
  while(M4_pos < MmToTicks(distance1) && M1_pos < MmToTicks(distance1)) {
          man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
              M4_pos = info.position();
          });
          man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
             M1_pos = -info.position();
          });
          Serial.printf("[ENCODERY] M4_pos: %d\n", M4_pos);
          if(M1_pos > 200 && M4_pos > 200) {
            zkontroluj_pid(20000, M1_pos, M4_pos);
          }
          if((rkUltraMeasure(1) < 150 && (rkUltraMeasure(1) > 30)) || ((rkUltraMeasure(2) < 150) && (rkUltraMeasure(2) > 30))) {
            Serial.println("Zastavuji, narazil na soupere!");
            setMotorsPower(0, 0); // Zastaví motory
            delay(1000);
            if((rkUltraMeasure(1) < 150 && (rkUltraMeasure(1) < 30)) || ((rkUltraMeasure(2) < 150) && (rkUltraMeasure(2) < 30))){
                
                rkBuzzerSet(true);
                delay(500);
                rkBuzzerSet(false);
                break;
            }
          }
          delay(10);
    }
      aktualni_pozice.y = TicksToMm(M1_pos); // Nastaví aktuální pozici na střed
      vTaskDelete(chytejPukyHandle);
      chytejPukyHandle = NULL; // Uvolníme task, pokud běžel
      setMotorsPower(0, 0); // Zastaví motory
      delay(100);
      turn_on_spot(90); // otočí se o 90 stupňů
      man.motor(rb::MotorId::M1).setCurrentPosition(0);
      man.motor(rb::MotorId::M4).setCurrentPosition(0);
      M1_pos = 0;
      M4_pos = 0;
      int distance2 = 1000;
      Serial.printf("musi_jet::: M1_pos: %d, M4_pos: %d\n", MmToTicks(distance2), MmToTicks(distance2));
      xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);
      setMotorsPower(20000, 20000); // Nastaví motory na 20000
      while(M4_pos < MmToTicks(distance2) && M1_pos < MmToTicks(distance2)) {
          man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
              M4_pos = info.position();
          });
          man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
             M1_pos = -info.position();
          });
          Serial.printf("[ENCODERY] M4_pos: %d\n", M4_pos);
          if(M1_pos > 200 && M4_pos > 200) {
            zkontroluj_pid(20000, M1_pos, M4_pos);
          }
          if((rkUltraMeasure(1) < 150 && (rkUltraMeasure(1) > 30)) || ((rkUltraMeasure(2) < 150) && (rkUltraMeasure(2) > 30))) {
            Serial.println("Zastavuji, narazil na soupere!");
            setMotorsPower(0, 0); // Zastaví motory
            delay(1000);
            if((rkUltraMeasure(1) < 150 && (rkUltraMeasure(1) < 30)) || ((rkUltraMeasure(2) < 150) && (rkUltraMeasure(2) < 30))){
                rkBuzzerSet(true);
                delay(500);
                rkBuzzerSet(false);
                break;
            }
          }
          delay(10);
      }
      aktualni_pozice.x = (TicksToMm(M1_pos) + 350); // Nastaví aktuální pozici na střed
      Serial.printf("Aktualni pozice: x: %d, y: %d\n", aktualni_pozice.x, aktualni_pozice.y);
      vTaskDelete(chytejPukyHandle);
      chytejPukyHandle = NULL; // Uvolníme task, pokud běžel
      setMotorsPower(0, 0); // Zastaví motory
      turn_on_spot(90); // otočí se
      back_buttons(40); // otočí se o 180 stupňů
      man.motor(rb::MotorId::M1).setCurrentPosition(0);
      man.motor(rb::MotorId::M4).setCurrentPosition(0);
      M1_pos = 0;
      M4_pos = 0;
      aktualni_pozice.y = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
      Serial.printf("Aktualni pozice: x: %d, y: %d\n", aktualni_pozice.x, aktualni_pozice.y);
      xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);
      int distance3 = aktualni_pozice.y - stred_od_predu ; // Vzdálenost do levé krabice
      setMotorsPower(20000, 20000); // Nastaví motory na 20000
      Serial.printf("musi_jet::: M1_pos: %d, M4_pos: %d\n", MmToTicks(distance1), MmToTicks(distance1));
      while(M4_pos < MmToTicks(distance1) && M1_pos < MmToTicks(distance1)) {
              man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                  M4_pos = info.position();
              });
              man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
                M1_pos = -info.position();
              });
              Serial.printf("[ENCODERY] M4_pos: %d\n", M4_pos);
              if(M1_pos > 200 && M4_pos > 200) {
                zkontroluj_pid(20000, M1_pos, M4_pos);
              }
              if((rkUltraMeasure(1) < 150 && (rkUltraMeasure(1) > 30)) || ((rkUltraMeasure(2) < 150) && (rkUltraMeasure(2) > 30))) {
                Serial.println("Zastavuji, narazil na soupere!");
                setMotorsPower(0, 0); // Zastaví motory
                delay(1000);
                if((rkUltraMeasure(1) < 150 && (rkUltraMeasure(1) < 30)) || ((rkUltraMeasure(2) < 150) && (rkUltraMeasure(2) < 30))){
                    rkBuzzerSet(true);
                    delay(500);
                    rkBuzzerSet(false);
                    break;
                }
              }
              delay(10);
          }
      vTaskDelete(chytejPukyHandle);
      chytejPukyHandle = NULL; // Uvolníme task, pokud běžel
      setMotorsPower(0, 0); // Zastaví motory
      M1_pos = 0;
      M4_pos = 0;
      aktualni_pozice.y = aktualni_pozice.y - TicksToMm(M1_pos);
      Serial.printf("Aktualni pozice: x: %d, y: %d\n", aktualni_pozice.x, aktualni_pozice.y);
      turn_on_spot(90); // otočí se o 90 stupňů
      open_l_box(); // Otevře levý box --- cervene -- souperovi puky
      forward(aktualni_pozice.x - 250, 70);
      close_l_box(); // Zavře levý box
      turn_on_spot(90); // otočí se o 90 stupňů
      back_buttons(40); // otočí se o 180 stup
      forward(60, 70); // Přejede do středu
      open_r_box(); // Otevře pravý box --- modre -- svoje puky
      forward(1000, 70); // Přejede do středu
}

// void modra(){
//   Serial.println("Modra barva");
//   auto& man = rb::Manager::get(); // vytvoří referenci na man class
//   man.motor(rb::MotorId::M1).setCurrentPosition(0);
//   man.motor(rb::MotorId::M4).setCurrentPosition(0);
//   int M4_pos = 0;
//   int M1_pos = 0;
//   man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
//           M4_pos = info.position();
//       });
//   man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
//          M1_pos = -info.position();
//       });
//   xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);
//   int distance1 = 1800;
//   Serial.printf("musi_jet::: M1_pos: %d, M4_pos: %d\n", MmToTicks(distance1), MmToTicks(distance1));
//   while(M4_pos < MmToTicks(distance1) || M1_pos < MmToTicks(distance1)) {
//           man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
//               M4_pos = info.position();
//           });
//           man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
//              M1_pos = -info.position();
//           });
//           Serial.printf("[ENCODERY] M4_pos: %d\n", M4_pos);
//           if(M1_pos > 200 && M4_pos > 200) {
//             zkontroluj_pid(20000, M1_pos, M4_pos);
//           }
//           if((rkUltraMeasure(1) < 250 && (rkUltraMeasure(1) > 30)) || ((rkUltraMeasure(2) < 250) && (rkUltraMeasure(2) > 30))) {
//             Serial.println("Zastavuji, narazil na soupere!");
//             setMotorsPower(0, 0); // Zastaví motory
//             delay(5000);
//             if((rkUltraMeasure(1) < 250 && (rkUltraMeasure(1) < 30)) || ((rkUltraMeasure(2) < 250) && (rkUltraMeasure(2) < 30))){
//                 break;
//             }
//           }
//           delay(10);
//       }
//       vTaskDelete(chytejPukyHandle);
//       chytejPukyHandle = NULL; // Uvolníme task, pokud běžel
//       setMotorsPower(0, 0); // Zastaví motory
//       turn_on_spot(180); // otočí se o 90 stupňů
//       delay(100);
//       forward(800, 60);
//       open_l_box();
//       forward(800, 60);
//       close_l_box();
//       open_r_box();
//       forward(500, 60);
//       close_r_box();
// }




void cervena(){
  Serial.println("Cervena barva");
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
      int max_distance_enc  =  MmToTicks(3000);
      Serial.printf("Maximalni vzdalenost: %d\n", max_distance_enc);
      xTaskCreate(StopTask, "StopTask", 2048, NULL, 1, NULL); // Spustí task pnuti po 3minutach pozadí pnuti po 3minutach
      setMotorsPower(20000, 20000); // Oba motory vpřed
      int distance1 = rkUltraMeasure(1);
      int distance2 = rkUltraMeasure(2);
      auto& man = rb::Manager::get(); // vytvoří referenci na man class
      man.motor(rb::MotorId::M1).setCurrentPosition(0);
      man.motor(rb::MotorId::M4).setCurrentPosition(0);
      Serial.printf("Vzdálenosti: 1=%d mm, 2=%d mm\n", distance1, distance2);
      int M4_pos = 0;
      int M1_pos = 0;
      man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
              M4_pos = info.position();
          });
      man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
             M1_pos = -info.position();
          });
      Serial.printf("[ENCODERY] M1_pos: %d\n", M1_pos);
      Serial.printf("[ENCODERY] M4_pos: %d\n", M4_pos);
        /////////////////////////
      while(((distance1 > 150) || (distance1 <20)) && ((distance2 > 150) || (distance2<20))) {
          man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
              M4_pos = info.position();
          });
          man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
             M1_pos = -info.position();
          });
          Serial.printf("[ENCODERY] M4_pos: %d\n", M4_pos);
          if(M1_pos > 200 && M4_pos > 200) {
            zkontroluj_pid(20000, M1_pos, M4_pos);
          }
          if(M4_pos > max_distance_enc){
            break;
          }
          delay(10);
          distance1 = rkUltraMeasure(1);
          distance2 = rkUltraMeasure(2);
          Serial.printf("Vzdálenosti: 1=%d mm, 2=%d mm\n", distance1, distance2);
      }
      Serial.println("Cesta dokončena, zastavuji motory.");
      setMotorsPower(0, 0); // Zastaví motory
  }
  else if(rkButtonIsPressed(BTN_DOWN)) {
    delay(3000);
    back_buttons(70);
  }
  else if(rkButtonIsPressed(BTN_LEFT)) {
    zavreni_dvirek();
    close_l_box();
    close_r_box();
    delay(3000);
    setMotorsPower(20000, 20000); // Oba motory vpřed
    modra();
  }
  else if(rkButtonIsPressed(BTN_ON)){
    zavreni_dvirek();
    delay(3000);
    setMotorsPower(20000, 20000); // Oba motory vpřed
    xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);
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
  //   void rkServosSetPosition(uint8_t id, float angleDegrees) {
  //     id -= 1;
  //     if (id >= rb::StupidServosCount) {
  //         ESP_LOGE(TAG, "%s: invalid id %d, must be <= %d!", __func__, id, rb::StupidServosCount);
  //         return;
  //     }
  //     gCtx.stupidServoSet(id, angleDegrees);
  // }

  // float rkServosGetPosition(uint8_t id) {
  //     id -= 1;
  //     if (id >= rb::StupidServosCount) {
  //         ESP_LOGE(TAG, "%s: invalid id %d, must be <= %d!", __func__, id, rb::StupidServosCount);
  //         return NAN;
  //     }
  //     return gCtx.stupidServoGet(id);
  // }

  // void rkServosDisable(uint8_t id) {
  //     id -= 1;
  //     if (id >= rb::StupidServosCount) {
  //         ESP_LOGE(TAG, "%s: invalid id %d, must be <= %d!", __func__, id, rb::StupidServosCount);
  //         return;
  //     }
  //     rb::Manager::get().stupidServo(id).disable();
  // }
  //   // }
  //   open_r_box();
  //   delay(5000);
  //   close_r_box();
  // }
    if (rkButtonIsPressed(BTN_OFF)) {//dolu
     int stop_time= 1000000;
     for(int i=0; i < stop_time; i+=300) {
         int distance1 = rkUltraMeasure(1);
        int distance2 = rkUltraMeasure(2);
        int distance3 = rkUltraMeasure(3);
        int distance4 = rkUltraMeasure(4);
      printf("Vzdálenosti: 1=%d mm, 2=%d mm, 3=%d mm, 4=%d mm\n", distance1, distance2, distance3, distance4);
       delay(300);
   }
 }
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