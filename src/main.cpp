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
int U1_distance = 10000;
int U2_distance = 10000;
int U3_distance = 10000;
int U4_distance = 10000;
bool je_neco_vepredu(int distance_mereni){
    U1_distance = rkUltraMeasure(1);
    U2_distance = rkUltraMeasure(2);
    U4_distance = rkUltraMeasure(4);
  if (U1_distance < 1){
      U1_distance = 10000; // pokud je vzdálenost menší než 1, nastaví se na 10000
  }
  if (U2_distance < 1){
      U2_distance = 10000; // pokud je vzdálenost menší než 1, nastaví se na 10000
  }
  if (U4_distance < 1){
      U4_distance = 10000; // pokud je vzdálenost menší než 1, nastaví se na 10000
  }
    Serial.printf("U1: %d, U2: %d, U4: %d\n", U1_distance, U2_distance, U4_distance);
    if (U1_distance < distance_mereni || U2_distance < distance_mereni || U4_distance < distance_mereni) {
        Serial.printf("Přední senzory: U1: %d, U2: %d, U4: %d\n", U1_distance, U2_distance, U4_distance);
        rkBuzzerSet(true);
        delay(100);
        rkBuzzerSet(false);
        return true;
    }
    return false;
}
bool davam_prednost_z_prava = false;
bool detekce_soupere(){
  Serial.printf("U1: %d, U2: %d, U3: %d\n, U4: %d ", rkUltraMeasure(1), rkUltraMeasure(2), rkUltraMeasure(3), rkUltraMeasure(4));
    if(je_neco_vepredu(200)) {
        setMotorsPower(-7000, -7000); // Zastaví motory
        rkBuzzerSet(true);
        delay(300);
        rkBuzzerSet(false);
        delay(900);
        setMotorsPower(0, 0); // Zastaví motory
        return true; // Detekován soupeř
    }
    if((rkUltraMeasure(3) < 200) && (rkUltraMeasure(3) > 1)&& (!davam_prednost_z_prava)) { // U3 je senzor vzadu
        davam_prednost_z_prava = true; // Dáváme přednost zprava
        for(int i = 18000; i > 0; i=- 2000) {
            setMotorsPower(i, i); // Zpomaluje motory
            delay(10);
        }
        setMotorsPower(0,0);
        int start_time = millis();
        while (millis() - start_time < 5000) { // čeká 5 sekundy
            if(je_neco_vepredu(200)){
              setMotorsPower(-7000, -7000); // Zastaví motory
              rkBuzzerSet(true);
              delay(300);
              rkBuzzerSet(false);
              delay(900);
              setMotorsPower(0, 0); // Zastaví motory
              return true; // Detekován soupeř
            }
        }
        Serial.println("Čekání na soupeře skončilo, ale nic se nenašlo.");
        Serial.println("jedu dal.");
        return false; // Čekání skončilo, ale soupeř nebyl detekován

    }
return false;
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
void otevreni_prepazky(){
  rkServosSetPosition(3, -90); // Servo 1 nastaví pnuti po 3minutach 90°
  delay(200);
}
void zavreni_prepazky(){
  rkServosSetPosition(3, 0); // Servo 1 nastaví pnuti po 3minutach 90°
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
          zavreni_prepazky();
            if (now.barva == RED) {
              if( puck_in_l_box < max_puck_per_box) {
                Serial.println("Detekován červený puk!");
                puk_do_l_boxu();
                setMotorsPower(0, 0); // zastaví motory
                delay(300);
                rkMotorsDrive(60, 60, 100, 100);
                zavreni_dvirek();
                delay(200);
                otevreni_prepazky();
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
                  otevreni_prepazky();
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
///////////////////
int M4_pos = 0;
int M1_pos = 0;

void jed_a_chytej_puky(int distance, bool x, bool kladna = true){
  auto& man = rb::Manager::get(); // vytvoří referenci na man class
  man.motor(rb::MotorId::M1).setCurrentPosition(0);
  man.motor(rb::MotorId::M4).setCurrentPosition(0);
  otevreni_prepazky(); // Otevře prepážku
  man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
          M4_pos = info.position();
      });
  man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
         M1_pos = -info.position();
      });

  xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);

  Serial.printf("musi_jet::: M1_pos: %d, M4_pos: %d\n", MmToTicks(distance), MmToTicks(distance));
  setMotorsPower(20000, 20000); // Nastaví motory na 20000

  while(M4_pos < MmToTicks(distance) && M1_pos < MmToTicks(distance)) {
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

          if(detekce_soupere()) {
           break;
          }
          delay(10);
    }
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
              M4_pos = info.position();
          });
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
             M1_pos = -info.position();
          });
    Serial.printf("[ENCODERY] M4_pos: %d, M1_pos: %d\n", M4_pos, M1_pos);
    if(kladna){
      if(x) {
        aktualni_pozice.x = aktualni_pozice.x + TicksToMm(M1_pos); // Nastaví aktuální pozici na střed
      }
      else{
        aktualni_pozice.y = aktualni_pozice.y + TicksToMm(M1_pos); // Nastaví aktuální pozici na střed
      }
    }
    else{
      if(x) {
        aktualni_pozice.x = aktualni_pozice.x - TicksToMm(M1_pos); // Nastaví aktuální pozici na střed
      }
      else{
        aktualni_pozice.y = aktualni_pozice.y - TicksToMm(M1_pos); // Nastaví aktuální pozici na střed
      }
    }
      vTaskDelete(chytejPukyHandle);
      chytejPukyHandle = NULL; // Uvolníme task, pokud běžel
      setMotorsPower(0, 0); // Zastaví motory
      delay(100);
      man.motor(rb::MotorId::M1).setCurrentPosition(0);
      man.motor(rb::MotorId::M4).setCurrentPosition(0);
      M1_pos = 0;
      M4_pos = 0;
}



void modra(){
  Serial.println("Modra barva");
  jed_a_chytej_puky(1800 - aktualni_pozice.y, false);
  turn_on_spot(90); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stupňů
  open_l_box();
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  delay(100);
  jed_a_chytej_puky(1200 - aktualni_pozice.x, true);
  close_l_box(); // Zavře levý box
  turn_on_spot(90);
  back_buttons(80);
  aktualni_pozice.y = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky((aktualni_pozice.y- stred_od_predu- 200), false, false); // jede do levé krabice
  turn_on_spot(90); // otočí se o 90 stupňů
  open_l_box(); // Otevře levý box --- cervene -- souperovi puky
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do levé krabice
  close_l_box(); // Zavře levý box
  turn_on_spot(182); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stup
  forward(70, 70); // Přejede do středu
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  delay(200);
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  open_r_box(); // Otevře pravý box --- modre -- svoje puky
  forward(100, 60);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_r_box(); // Zavře pravý box
  //////////////////////////////////////////////////
  //prvni kolo ujeto!!!
  ////////////////////////////////////////
  jed_a_chytej_puky(1500 -aktualni_pozice.y, false);
  turn_on_spot(90); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(2000 - aktualni_pozice.x, true);
  turn_on_spot(90);
  open_l_box(); // Otevře levý box --- cervene -- souperovi puky
  jed_a_chytej_puky(600, false, false);
  close_l_box(); // Zavře levý box
  turn_on_spot(90); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stupňů
  open_l_box(); // Otevře pravý box --- modre -- svoje puky
  aktualni_pozice.x = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do pravé krabice
  turn_on_spot(180); // otočí se o 90 stupňů
  close_l_box(); // Zavře levý box
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  open_r_box(); // Otevře pravý box --- modre -- svoje puky
  forward(180, 60);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_r_box(); // Zavře pravý box
  //////////////////////////////////////////////////////////
  // druhé kolo ujeto!!!
  ////////////////////////////////////////
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  turn_on_spot(90); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(2100 -aktualni_pozice.x- stred_od_predu, true); // jede do levé krabice
  open_l_box(); // Otevře levý box --- cervene -- souperovi puky
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  close_l_box(); // Zavře levý box
  back_buttons(80); // otočí se o 180 stupňů
  aktualni_pozice.y = 2500 - stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(800 - aktualni_pozice.y, false);
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  close_l_box(); // Zavře levý box
  back_buttons(80); // otočí se o 180 stupňů
  aktualni_pozice.x =2500 - stred_od_zadu; // Nastaví aktuální pozici na střed
  open_l_box(); // Otevře pravý box --- modre -- svoje puky
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu- 200, true, false); // jede do pravé krabice
  close_l_box(); 
  turn_on_spot(180);
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  open_r_box(); // Otevře levý box --- cervene -- souperovi puky
  forward(180, 60);
  close_r_box(); // Zavře levý box
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  //////////////////////////////////////////////////////////

  rkBuzzerSet(true);
  delay(500);
  rkBuzzerSet(false);
  delay(500);
  rkBuzzerSet(true);
  delay(500);
  rkBuzzerSet(false);
  /////////////////////////////////////////////////////////////
}

  




void cervena(){
  Serial.println("Cervena barva");
  jed_a_chytej_puky(1800 - aktualni_pozice.y, false);
  turn_on_spot(90); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stupňů
  open_r_box();
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  delay(100);
  jed_a_chytej_puky(1200 - aktualni_pozice.x, true);
  close_r_box(); // Zavře levý box
  turn_on_spot(90);
  back_buttons(80);
  aktualni_pozice.y = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky((aktualni_pozice.y- stred_od_predu- 200), false, false); // jede do levé krabice
  turn_on_spot(90); // otočí se o 90 stupňů
  open_r_box(); // Otevře levý box --- cervene -- souperovi puky
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do levé krabice
  close_r_box(); // Zavře levý box
  turn_on_spot(182); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stup
  forward(70, 70); // Přejede do středu
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  delay(200);
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  open_l_box(); // Otevře pravý box --- modre -- svoje puky
  forward(100, 60);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_l_box(); // Zavře pravý box
  //////////////////////////////////////////////////
  //prvni kolo ujeto!!!
  ////////////////////////////////////////
  jed_a_chytej_puky(1500 -aktualni_pozice.y, false);
  turn_on_spot(90); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(2000 - aktualni_pozice.x, true);
  turn_on_spot(90);
  open_r_box(); // Otevře levý box --- cervene -- souperovi puky
  jed_a_chytej_puky(600, false, false);
  close_r_box(); // Zavře levý box
  turn_on_spot(90); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stupňů
  open_r_box(); // Otevře pravý box --- modre -- svoje puky
  aktualni_pozice.x = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do pravé krabice
  turn_on_spot(180); // otočí se o 90 stupňů
  close_r_box(); // Zavře levý box
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  open_l_box(); // Otevře pravý box --- modre -- svoje puky
  forward(180, 60);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_l_box(); // Zavře pravý box
  //////////////////////////////////////////////////////////
  // druhé kolo ujeto!!!
  ////////////////////////////////////////
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  turn_on_spot(90); // otočí se o 90 stupňů
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(2100 -aktualni_pozice.x - stred_od_predu, true); // jede do levé krabice
  open_r_box(); // Otevře levý box --- cervene -- souperovi puky
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  close_r_box(); // Zavře levý box
  back_buttons(80); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(800 - aktualni_pozice.y, true);
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  close_r_box(); // Zavře levý box
  delay(300);
  back_buttons(80); // otočí se o 180 stupňů
  aktualni_pozice.x =2500 - stred_od_zadu; // Nastaví aktuální pozici na střed
  open_r_box(); // Otevře pravý box --- modre -- svoje puky
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu- 200, false);
  close_r_box(); 
  turn_on_spot(180);
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  turn_on_spot(-90); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(80); // otočí se o 180 stupňů
  forward(70, 70); // Přejede do středu
  open_l_box(); // Otevře levý box --- cervene -- souperovi puky
  forward(180, 60);
  close_l_box(); // Zavře levý box
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  //////////////////////////////////////////////////////////

  rkBuzzerSet(true);
  delay(500);
  rkBuzzerSet(false);
  delay(500);
  rkBuzzerSet(true);
  delay(500);
  rkBuzzerSet(false);

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
  if(rkButtonIsPressed(BTN_LEFT)) {
    zavreni_dvirek();
    close_l_box();
    close_r_box();
    delay(3000);
    modra();
  }
  else if(rkButtonIsPressed(BTN_RIGHT)) {
    zavreni_dvirek();
    close_l_box();
    close_r_box();
    delay(3000);
    cervena();
  }
  else if(rkButtonIsPressed(BTN_ON)){
    back_buttons(80); // otočí se o 180 stupňů
    forward(70, 70); // Přejede do středu
    turn_on_spot(90); // otočí se o 90 stupňů
    back_buttons(80); // otočí se o 180 stupňů
    forward(70, 70); // Přejede do středu
    aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
    aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
    jed_a_chytej_puky(2100 -aktualni_pozice.x - stred_od_predu, true); // jede do levé krabice
    open_r_box(); // Otevře levý box --- cervene -- souperovi puky
    delay(300);
    turn_on_spot(-90); // otočí se o 90 stupňů
    delay(1000);
    back_buttons(80); // otočí se o 180 stupňů
    aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
    jed_a_chytej_puky(800 - aktualni_pozice.y, true);
    turn_on_spot(-90); // otočí se o 90 stupňů
    delay(1000);
    close_r_box(); // Zavře levý box
    back_buttons(80); // otočí se o 180 stupňů
    aktualni_pozice.x =2500 - stred_od_zadu; // Nastaví aktuální pozici na střed
    open_r_box(); // Otevře pravý box --- modre -- svoje puky
    jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu- 200, false);
    close_r_box(); 
    turn_on_spot(180);
    delay(1000);
    back_buttons(80); // otočí se o 180 stupňů
    forward(70, 70); // Přejede do středu
    turn_on_spot(-90); // otočí se o 90 stupňů
    delay(1000);
    back_buttons(80); // otočí se o 180 stupňů
    forward(70, 70); // Přejede do středu
    open_l_box(); // Otevře levý box --- cervene -- souperovi puky
    forward(180, 60);
    close_l_box(); // Zavře levý box
    aktualni_pozice.y = 450; // Nastaví aktuální pozici na středů
  }
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
 else if(rkButtonIsPressed(BTN_DOWN)) {
    Serial.println("Zahajuji TEST VSECH KOMPONENT...");
    // 1. Pohyb vpřed
    setMotorsPower(15000, 15000);
    delay(1000); // Jede 1 sekundu vpřed
    setMotorsPower(0, 0);
    Serial.println("Test: Pohyb vpřed OK");

    // 2. Použití zadních tlačítek (simulace funkce back_buttons)
    Serial.println("Test: Zadní tlačítka (back_buttons)");
    back_buttons(50); // Otočí se o 180 stupňů (nebo jiná testovací akce)

    // 3. Pohyb servy
    Serial.println("Test: Serva");
    open_l_box();
    delay(500);
    close_l_box();
    delay(500);
    open_r_box();
    delay(500);
    close_r_box();
    delay(500);

        // 4. Pohyb chytrého serva (smart servo)
    Serial.println("Test: Chytré servo");
    auto &bus = rkSmartServoBus(1);
    s_s_move(bus, 0, 90, 100.0); // Nastaví servo na 90°
    delay(700);
    s_s_move(bus, 0, 150, 100.0); // Nastaví servo na 150°
    delay(700);
    s_s_move(bus, 0, 120, 100.0); // Vrátí zpět na 120°
    delay(700);

    // 4. Test ultrazvukových senzorů
    Serial.println("Test: Ultrazvukové senzory");
    int u1 = rkUltraMeasure(1);
    int u2 = rkUltraMeasure(2);
    int u3 = rkUltraMeasure(3);
    int u4 = rkUltraMeasure(4);
    Serial.printf("Ultrazvuky: 1=%d mm, 2=%d mm, 3=%d mm, 4=%d mm\n", u1, u2, u3, u4);

    // 5. Test barevných senzorů
    Serial.println("Test: Barevné senzory");
    if (rkColorSensorGetRGB("puky", &r1, &g1, &b1)) {
        Serial.printf("PUKY: R=%.1f G=%.1f B=%.1f\n", r1, g1, b1);
    } else {
        Serial.println("PUKY: Sensor nenalezen!");
    }
    if (rkColorSensorGetRGB("zem", &r2, &g2, &b2)) {
        Serial.printf("ZEM: R=%.1f G=%.1f B=%.1f\n", r2, g2, b2);
    } else {
        Serial.println("ZEM: Sensor nenalezen!");
    }

    Serial.println("Test všech komponent dokončen.");
    delay(2000); // Krátká pauza po testu
}

  delay(50);
}