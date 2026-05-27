#include <Arduino.h>
#include "robotka.h"
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "esp_system.h" // pro esp_restart()
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

volatile bool puck_handling = false;
String color_log_history = "";
int log_counter = 0;

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
        rkMotorsSetSpeed(-22, -22); // Zastaví motory
        rkBuzzerSet(true);
        delay(300);
        rkBuzzerSet(false);
        delay(900);
        rkMotorsSetSpeed(0, 0); // Zastaví motory
        return true; // Detekován soupeř
    }
    if((rkUltraMeasure(3) < 200) && (rkUltraMeasure(3) > 1)&& (!davam_prednost_z_prava)) { // U3 je senzor vzadu
        davam_prednost_z_prava = true; // Dáváme přednost zprava
        for(int i = 56; i > 0; i -= 6) {
            rkMotorsSetSpeed(i, i); // Zpomaluje motory
            delay(10);
        }
        rkMotorsSetSpeed(0, 0);
        int start_time = millis();
        while (millis() - start_time < 5000) { // čeká 5 sekundy
            if(je_neco_vepredu(200)){
              rkMotorsSetSpeed(-22, -22); // Zastaví motory
              rkBuzzerSet(true);
              delay(300);
              rkBuzzerSet(false);
              delay(900);
              rkMotorsSetSpeed(0, 0); // Zastaví motory
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
        if ((r1 -38) > b1 && (r1-38) > g1) {
            vysledek.barva = RED;
            vysledek.je_tam = true;
        } else if ((b1 > 90)&& (b1 -18) > r1 && (b1-18) > g1) {
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
    rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    delay(20);

    if (rkColorSensorGetRGB("zem", &r2, &g2, &b2)) {
        Serial.print("ZEM_R: "); Serial.print(r2, 3);
        Serial.print(" ZEM_G: "); Serial.print(g2, 3);
        Serial.print(" ZEM_B: "); Serial.println(b2, 3);
        if ((r2 -38) > b2 && (r2-38) > g2) {
            vysledek.barva = RED;
            vysledek.je_tam = true;
            Serial.println("Jsem na červené zemi.");
        } else if ((b2 - 15 > r2 )&& (b2 > 85)) {//else if ((b2 > 90)&& (b2 -18) > r2 && (b2-18) > g2) {
            vysledek.barva = BLUE;
            vysledek.je_tam = true;
            Serial.println("Jsem na modré zemi.");
        } else {
            vysledek.je_tam = false;
        }
    } else {
        vysledek.je_tam = false;
    }
    return vysledek;
}
void puk_do_l_boxu(){
  rkSmartServoSoftMove(0, 170, 180);
}
void puk_do_r_boxu(){
  rkSmartServoSoftMove(0, 70, 180);
}
void zavreni_dvirek(){
  rkSmartServoSoftMove(0, 125, 130); // nahoru
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
    vTaskDelay(4 * 60 * 1000 / portTICK_PERIOD_MS); // 3 minuty
    Serial.println("Uplynuly 3 minuty, restartuji program...");
    esp_restart(); // Restartuje ESP32
    vTaskDelete(NULL);
}


void ChytejPukyTask(void *pvParameters) {
    while (true) {
        auto now = Detekce_puku();
        if (now.je_tam) {
            puck_handling = true;
            zavreni_prepazky();
            if (now.barva == RED) {
              if( puck_in_l_box < max_puck_per_box) {
                Serial.println("Detekován červený puk!");
                puk_do_l_boxu();
                rkMotorsSetSpeed(0, 0); // zastaví motory
                delay(300);
                rkMotorsDrive(12, 12, 100, 100);
                zavreni_dvirek();
                delay(400);
                otevreni_prepazky();
                puck_in_l_box++;
                Serial.printf("Puky v levém boxu: %d\n", puck_in_l_box);
                }
            } else if (now.barva == BLUE) {
                if( puck_in_r_box < max_puck_per_box) {
                  Serial.println("Detekován modrý puk!");
                  puk_do_r_boxu();
                  rkMotorsSetSpeed(0, 0); // zastaví motory
                  delay(300);
                  rkMotorsDrive(12, 12, 100, 100);
                  zavreni_dvirek();
                  delay(400);
                  otevreni_prepazky();
                  puck_in_r_box++;
                }
            }
            puck_handling = false;
        } else {
            Serial.println("Puk nenalezen.");
        }
        vTaskDelay(200 / portTICK_PERIOD_MS); // malá pauza, aby task nebyl příliš rychlý
    }
    vTaskDelete(NULL); // kdyby někdy skončil
}
///////////////////
void jed_a_chytej_puky(int distance, bool x, bool kladna = true){
  if (distance <= 0) return;
  // Reset pozic enkodérů
  rkMotorsSetPositionLeft(0);
  rkMotorsSetPositionRight(0);
  otevreni_prepazky(); // Otevře prepážku

  puck_handling = false; // Reset to false just in case
  xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);

  Serial.printf("musi_jet::: distance: %d mm\n", distance);

  float pos_left = 0;
  float pos_right = 0;
  
  // Acceleration/deceleration parameters
  float const max_s = 15.0f; // maximum speed (slower than 20% by user request)
  float const min_s = 8.0f;  // minimum speed (so it overcomes friction but starts slow)
  float const ramp_dist = 120.0f; // distance in mm to ramp up/down
  
  float actual_ramp = ramp_dist;
  if (distance < 2.0f * ramp_dist) {
      actual_ramp = distance / 2.0f;
  }

  while(pos_left < distance && pos_right < distance) {
      pos_left = abs(rkMotorsGetPositionLeft(true));
      pos_right = abs(rkMotorsGetPositionRight(true));
      float ujeta_vzdalenost = (pos_left + pos_right) / 2.0f;

      if (!puck_handling) {
          // 1) Ramp calculation (Acceleration & Deceleration)
          float speed_base = max_s;
          if (ujeta_vzdalenost < actual_ramp) {
              // Ramp up
              float progress = ujeta_vzdalenost / actual_ramp;
              speed_base = min_s + (max_s - min_s) * progress;
          } else if (distance - ujeta_vzdalenost < actual_ramp) {
              // Ramp down
              float progress = (distance - ujeta_vzdalenost) / actual_ramp;
              speed_base = min_s + (max_s - min_s) * progress;
          }
          
          if (speed_base < min_s) speed_base = min_s;
          if (speed_base > max_s) speed_base = max_s;

          // 2) Proportional control for straight-line driving (encoder sync using raw ticks, like back_buttons)
          int left_ticks = abs(rkMotorsGetPositionLeft(false));
          int right_ticks = abs(rkMotorsGetPositionRight(false));
          int error = left_ticks - right_ticks;
          
          float m_kp = 0.23f; // Same proportional constant as back_buttons
          float m_max_correction = 5.0f; // Same max correction as back_buttons
          
          float correction = error * m_kp;
          if (correction > m_max_correction) correction = m_max_correction;
          if (correction < -m_max_correction) correction = -m_max_correction;

          float speed_left = speed_base;
          float speed_right = speed_base;

          if (error > 0) {
              // Left is ahead - slow down left
              speed_left -= correction;
          } else if (error < 0) {
              // Right is ahead - slow down right
              speed_right += correction;
          }

          // Apply bounds
          if (speed_left < min_s) speed_left = min_s;
          if (speed_left > max_s) speed_left = max_s;
          if (speed_right < min_s) speed_right = min_s;
          if (speed_right > max_s) speed_right = max_s;

          rkMotorsSetSpeed(speed_left, speed_right);
      }

      Serial.printf("[ENCODERY] left: %.1f mm, right: %.1f mm (handling puck: %s)\n", 
                    pos_left, pos_right, puck_handling ? "YES" : "NO");

      if(detekce_soupere()) {
          break;
      }
      delay(10);
  }

  // Přečtení finální pozice
  pos_left = abs(rkMotorsGetPositionLeft(true));
  pos_right = abs(rkMotorsGetPositionRight(true));
  float ujeta_vzdalenost = (pos_left + pos_right) / 2.0f;
  Serial.printf("[ENCODERY] final left: %.1f, right: %.1f, avg: %.1f mm\n", pos_left, pos_right, ujeta_vzdalenost);
  
  if(kladna){
    if(x) {
      aktualni_pozice.x = aktualni_pozice.x + (int)ujeta_vzdalenost;
    }
    else{
      aktualni_pozice.y = aktualni_pozice.y + (int)ujeta_vzdalenost;
    }
  }
  else{
    if(x) {
      aktualni_pozice.x = aktualni_pozice.x - (int)ujeta_vzdalenost;
    }
    else{
      aktualni_pozice.y = aktualni_pozice.y - (int)ujeta_vzdalenost;
    }
  }
  
  vTaskDelete(chytejPukyHandle);
  chytejPukyHandle = NULL; // Uvolníme task, pokud běžel
  rkMotorsSetSpeed(0, 0); // Zastaví motory
  delay(100);
  rkMotorsSetPositionLeft(0);
  rkMotorsSetPositionRight(0);
  zavreni_dvirek();
}
void dojed_na_svoje(bool modra){
  auto a = Kde_jsme(); // Zjistí, kde jsme
  if(modra){
    if (a.je_tam && a.barva == BLUE) {
      Serial.println("Jsem na modré barvě.");
    } else {
      Serial.println("Nejsem na modré barvě, hledaM MODROUA");
      while(true) { // dokud nejsme na modré barvě
      turn_on_spot_right(90, 10); // otočí se o 90 stupňů
      delay(1000);
      back_buttons(15); // otočí se o 180 stupňů
      jed_a_chytej_puky(2800, true, false); // jede do středu
      a = Kde_jsme(); // Zjistí, kde jsme
      if(a.je_tam && a.barva == BLUE) {
        Serial.println("Jsem na modré barvě.");
        rkBuzzerSet(true);
        delay(1000);
        rkBuzzerSet(false);
        turn_on_spot_left(180, 10); // otočí se o 90 stupňů
        back_buttons(15); // otočí se o 180 stupňů
        forward(80, 20); // Přejede do středu
        turn_on_spot_left(90, 10); // otočí se o 90 stupňů
        back_buttons(15); // otočí se o 180 stupňů
        forward(80, 20); // Přejede do středu
        turn_on_spot_right(90, 10); // otočí se o 90 stupňů
        aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
        aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
        delay(200);
        break;
      }
        delay(200);

      }
    }
  }
  else{
    if (a.je_tam && a.barva == RED) {
      Serial.println("Jsem na RED barvě.");
    } else {
      Serial.println("Nejsem na RED barvě, hledaM RED");
      while(true) { // dokud nejsme na modré barvě
      turn_on_spot_right(90, 10); // otočí se o 90 stupňů
      delay(1000);
      back_buttons(15); // otočí se o 180 stupňů
      jed_a_chytej_puky(2800, true, false); // jede do středu
      a = Kde_jsme(); // Zjistí, kde jsme
      if(a.je_tam && a.barva == RED) {
        Serial.println("reddddddddddddddddddd");
        rkBuzzerSet(true);
        delay(1000);
        rkBuzzerSet(false);
        turn_on_spot_right(182, 10); // otočí se o 90 stupňů
        back_buttons(15); // otočí se o 180 stupňů
        forward(70, 20); // Přejede do středu
        turn_on_spot_left(90, 10); // otočí se o 90 stupňů
        back_buttons(15); // otočí se o 180 stupňů
        forward(70, 20); // Přejede do středu
        turn_on_spot_right(90, 10); // otočí se o 90 stupňů
        aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
        aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
        delay(200);
        break;
      }
        delay(200);

      }
    }
  }
}


void modra(){
  Serial.println("Modra barva");
  jed_a_chytej_puky(2150 - aktualni_pozice.y, false);
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  // open_l_box(); // zakomentováno: levý box (červený) v modra()
  delay(100);
  jed_a_chytej_puky(2150 - aktualni_pozice.x, true);
  // close_l_box(); // Zavře levý box -- zakomentováno: levý box (červený) v modra()
  turn_on_spot_left(90, 10);
  back_buttons(15);
  aktualni_pozice.y = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky((aktualni_pozice.y- stred_od_predu- 200), false, false); // jede do levé krabice
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  // open_l_box(); // Otevře levý box --- cervene -- souperovi puky -- zakomentováno v modra()
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do levé krabice
  // close_l_box(); // Zavře levý box -- zakomentováno v modra()
  turn_on_spot_left(182, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stup
  forward(70, 20); // Přejede do středu
  turn_on_spot_right(90, 10); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  delay(200);
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  dojed_na_svoje(true);
  open_r_box(); // Otevře pravý box --- modre -- svoje puky
  forward(100, 15);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_r_box(); // Zavře pravý box
  //////////////////////////////////////////////////
  //prvni kolo ujeto!!!
  ////////////////////////////////////////
  jed_a_chytej_puky(1850 -aktualni_pozice.y, false);
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(2050 - aktualni_pozice.x, true);
  turn_on_spot_left(90, 10);
  // open_l_box(); // Otevře levý box --- cervene -- souperovi puky -- zakomentováno v modra()
  jed_a_chytej_puky(600, false, false);
  // close_l_box(); // Zavře levý box -- zakomentováno v modra()
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  // open_l_box(); // Otevře pravý box --- modre -- svoje puky -- zakomentováno v modra()
  aktualni_pozice.x = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do pravé krabice
  turn_on_spot_left(180, 10); // otočí se o 90 stupňů
  // close_l_box(); // Zavře levý box -- zakomentováno v modra()
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  turn_on_spot_right(90, 10); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  dojed_na_svoje(true);
  open_r_box(); // Otevře pravý box --- modre -- svoje puky
  forward(180, 15);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_r_box(); // Zavře pravý box
  //////////////////////////////////////////////////////////
  // druhé kolo ujeto!!!
  ////////////////////////////////////////
  jed_a_chytej_puky(1500 -aktualni_pozice.y, false);
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(1500 - aktualni_pozice.x, true);
  turn_on_spot_left(90, 10);
  // open_l_box(); // Otevře levý box --- cervene -- souperovi puky -- zakomentováno v modra()
  jed_a_chytej_puky(600, false, false);
  // close_l_box(); // Zavře levý box -- zakomentováno v modra()
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  // open_l_box(); // Otevře pravý box --- modre -- svoje puky -- zakomentováno v modra()
  aktualni_pozice.x = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do pravé krabice
  turn_on_spot_left(180, 10); // otočí se o 90 stupňů
  // close_l_box(); // Zavře levý box -- zakomentováno v modra()
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  turn_on_spot_right(90, 10); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  dojed_na_svoje(true);
  open_r_box(); // Otevře pravý box --- modre -- svoje puky
  forward(180, 15);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_r_box(); // Zavře pravý box

////////treti kolo ujeto//////////////////////

jed_a_chytej_puky(1000 -aktualni_pozice.y, false);
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(1000 - aktualni_pozice.x, true);
  turn_on_spot_left(90, 10);
  open_l_box(); // Otevře levý box --- cervene -- souperovi puky
  jed_a_chytej_puky(600, false, false);
  close_l_box(); // Zavře levý box
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  open_l_box(); // Otevře pravý box --- modre -- svoje puky
  aktualni_pozice.x = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do pravé krabice
  turn_on_spot_left(180, 10); // otočí se o 90 stupňů
  close_l_box(); // Zavře levý box
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  turn_on_spot_right(90, 10); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  dojed_na_svoje(true);
  open_r_box(); // Otevře pravý box --- modre -- svoje puky
  forward(180, 15);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_r_box(); // Zavře pravý box


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
  jed_a_chytej_puky(2150 - aktualni_pozice.y, false);
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  // open_r_box(); // zakomentováno: pravý box (modrý) v cervena()
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  delay(100);
  jed_a_chytej_puky(2150 - aktualni_pozice.x, true);
  // close_r_box(); // Zavře pravý box -- zakomentováno: pravý box (modrý) v cervena()
  
  turn_on_spot_left(90, 10);
  back_buttons(15);
  aktualni_pozice.y = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky((aktualni_pozice.y- stred_od_predu- 200), false, false); // jede do levé krabice
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  // open_r_box(); // Otevře pravý box --- cervene -- souperovi puky -- zakomentováno v cervena()
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do levé krabice
  // close_r_box(); // Zavře pravý box -- zakomentováno v cervena()
  turn_on_spot_left(182, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stup
  forward(70, 20); // Přejede do středu
  turn_on_spot_right(90, 10); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  delay(200);
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  dojed_na_svoje(false);
  open_l_box(); // Otevře pravý box --- modre -- svoje puky
  forward(100, 15);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_l_box(); // Zavře pravý box
  //////////////////////////////////////////////////
  //prvni kolo ujeto!!!
  ////////////////////////////////////////
  jed_a_chytej_puky(1850 -aktualni_pozice.y, false);
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(2050 - aktualni_pozice.x, true);
  turn_on_spot_left(90, 10);
  open_r_box(); // Otevře levý box --- cervene -- souperovi puky
  jed_a_chytej_puky(600, false, false);
  close_r_box(); // Zavře levý box
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  // open_r_box(); // Otevře pravý box --- modre -- svoje puky -- zakomentováno v cervena()
  aktualni_pozice.x = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do pravé krabice
  turn_on_spot_left(180, 10); // otočí se o 90 stupňů
  // close_r_box(); // Zavře pravý box -- zakomentováno v cervena()
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  turn_on_spot_right(90, 10); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  dojed_na_svoje(false);
  open_l_box(); // Otevře pravý box --- modre -- svoje puky
  forward(180, 15);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_l_box(); // Zavře pravý box
  //////////////////////////////////////////////////////////
  // druhé kolo ujeto!!!
  ////////////////////////////////////////
  jed_a_chytej_puky(1500 -aktualni_pozice.y, false);
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(1500 - aktualni_pozice.x, true);
  turn_on_spot_left(90, 10);
  // open_r_box(); // Otevře pravý box --- cervene -- souperovi puky -- zakomentováno v cervena()
  jed_a_chytej_puky(600, false, false);
  // close_r_box(); // Zavře pravý box -- zakomentováno v cervena()
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  // open_r_box(); // Otevře pravý box --- modre -- svoje puky -- zakomentováno v cervena()
  aktualni_pozice.x = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do pravé krabice
  turn_on_spot_left(180, 10); // otočí se o 90 stupňů
  // close_r_box(); // Zavře pravý box -- zakomentováno v cervena()
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  turn_on_spot_right(90, 10); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  dojed_na_svoje(false);
  open_l_box(); // Otevře pravý box --- modre -- svoje puky
  forward(180, 15);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_l_box(); // Zavře pravý box

    //////////////////////////////////////////////////////////
  // tretí kolo ujeto!!!
  ////////////////////////////////////////

    jed_a_chytej_puky(1000 -aktualni_pozice.y, false);
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  aktualni_pozice.x =stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(1000 - aktualni_pozice.x, true);
  turn_on_spot_left(90, 10);
  // open_r_box(); // Otevře pravý box --- cervene -- souperovi puky -- zakomentováno v cervena()
  jed_a_chytej_puky(600, false, false);
  // close_r_box(); // Zavře pravý box -- zakomentováno v cervena()
  turn_on_spot_left(90, 10); // otočí se o 90 stupňů
  back_buttons(15); // otočí se o 180 stupňů
  // open_r_box(); // Otevře pravý box --- modre -- svoje puky -- zakomentováno v cervena()
  aktualni_pozice.x = 2500- stred_od_zadu; // Nastaví aktuální pozici na střed
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false); // jede do pravé krabice
  turn_on_spot_left(180, 10); // otočí se o 90 stupňů
  // close_r_box(); // Zavře pravý box -- zakomentováno v cervena()
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  turn_on_spot_right(90, 10); // otočí se o 90 stupňů
  delay(1000);
  back_buttons(15); // otočí se o 180 stupňů
  forward(70, 20); // Přejede do středu
  aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
  aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
  dojed_na_svoje(false);
  open_l_box(); // Otevře pravý box --- modre -- svoje puky
  forward(180, 15);
  aktualni_pozice.y = 450; // Nastaví aktuální pozici na střed
  close_l_box(); // Zavře pravý box


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
    
    // Kompletní nastavení rkConfig přenesené z knihovny
    cfg.prevod_motoru = 919.889f; // Nastaveno na 1900 podle požadavku
    
    // Nastavení motorů
    cfg.motor_id_left = 4;
    cfg.motor_id_right = 1;
    cfg.motor_polarity_switch_left = true;
    cfg.motor_polarity_switch_right = false;
    cfg.motor_enable_failsafe = false;
    cfg.motor_max_ticks_per_second = 5200;
    cfg.motor_max_acceleration = 50000;
    cfg.motor_max_power_pct = 100;

    // Nastavení kol a rozteče
    cfg.left_wheel_diameter = 128.0;
    cfg.right_wheel_diameter = 128.0;
    cfg.motor_wheel_diameter = 128;
    cfg.roztec_kol = 267.0;

    // Korekční koeficienty
    cfg.konstanta_radius_vnejsi_kolo = 0.96f;
    cfg.konstanta_radius_vnitrni_kolo = 0.96f;
    cfg.korekce_nedotacivosti_left = 0.97f;
    cfg.korekce_nedotacivosti_right = 0.97f;

    // Nastavení tlačítek
    cfg.Button1 = 34;
    cfg.Button2 = 35;

    // Serva a WiFi
    cfg.stupid_servo_min = -1.65f;
    cfg.stupid_servo_max = 1.65f;
    cfg.pocet_chytrych_serv = 1;
    cfg.enable_wifi_log = false;
    cfg.enable_wifi_control_wasd = false;
    cfg.enable_wifi_terminal = false;
    cfg.wifi_ssid = "robot1234";
    cfg.wifi_password = "1234robot";

    rkSetup(cfg);

    printf("Robotka started!\n");
    pinMode(34, INPUT_PULLUP);
    pinMode(35, INPUT_PULLUP);
    // Inicializace chytrého serva
    rkSmartServoInit(0, 0, 240);
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
  if(rkButtonIsPressed(BTN_LEFT)) {
    zavreni_dvirek();
    close_l_box();
    close_r_box();
    xTaskCreate(StopTask, "StopTask", 1024, NULL, 1, NULL);
    delay(1000);
    modra();
  }
  else if(rkButtonIsPressed(BTN_RIGHT)) {
    zavreni_dvirek();
    close_l_box();
    close_r_box();
    xTaskCreate(StopTask, "StopTask", 1024, NULL, 1, NULL);
    delay(1000);
    cervena();
  }
  else if(rkButtonIsPressed(BTN_ON)){
    // === TEST POHYBU ===
    Serial.println("=== TEST POHYBU START ===");

    // 1) Rovně 1 metr
    Serial.println("1) Forward 1000 mm...");
    forward_acc(1500, 10);
    delay(5000);

    // 2) Otočení o 90° doprava
    Serial.println("2) Turn right 90°...");
    turn_on_spot_right(90, 8);
    delay(5000);

    // 3) Otočení o 180° doleva
    Serial.println("3) Turn left 180°...");
    turn_on_spot_left(180, 8);
    delay(5000);

    // 4) Back buttons - couvání ke zdi
    Serial.println("4) Back buttons...");
    back_buttons(8);

    Serial.println("=== TEST POHYBU KONEC ===");
    rkBuzzerSet(true);
    delay(300);
    rkBuzzerSet(false);
  }
  if(rkButtonIsPressed(BTN_UP)){
    // Precteni barevnych hodnot
    rkColorSensorGetRGB("puky", &r1, &g1, &b1);
    rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    
    log_counter++;
    char log_entry[150];
    snprintf(log_entry, sizeof(log_entry), 
             "Mereni #%d -> Barevny na puky: R=%.3f G=%.3f B=%.3f | Barevny na zem: R=%.3f G=%.3f B=%.3f\n",
             log_counter, r1, g1, b1, r2, g2, b2);
    
    color_log_history += log_entry;
    
    // Okamzity vypis pro zpetnou vazbu
    Serial.print(log_entry);
    
    // Pipnuti pro indikaci ulozeni
    rkBuzzerSet(true);
    delay(100);
    rkBuzzerSet(false);

    // Pockame na uvolneni tlacitka
    while(rkButtonIsPressed(BTN_UP)) {
      delay(20);
    }
  }
  if (rkButtonIsPressed(BTN_OFF)) {
    Serial.println("\n=== VYPIS HISTORIE BAREVNYCH SENZORU ===");
    if (color_log_history.length() == 0) {
      Serial.println("Zadne logy nebyly ulozeny.");
    } else {
      Serial.print(color_log_history);
    }
    Serial.println("========================================\n");
    
    // Dvojite pipnuti pro indikaci vypsani
    rkBuzzerSet(true);
    delay(80);
    rkBuzzerSet(false);
    delay(80);
    rkBuzzerSet(true);
    delay(80);
    rkBuzzerSet(false);

    // Pockame na uvolneni tlacitka
    while(rkButtonIsPressed(BTN_OFF)) {
      delay(20);
    }
  }
 else if(rkButtonIsPressed(BTN_DOWN)) {
    Serial.println("Zahajuji TEST VSECH KOMPONENT...");
    // 1. Pohyb vpřed
    rkMotorsSetSpeed(22, 22);
    delay(1000); // Jede 1 sekundu vpřed
    rkMotorsSetSpeed(0, 0);
    Serial.println("Test: Pohyb vpřed OK");

    // 2. Použití zadních tlačítek (simulace funkce back_buttons)
    Serial.println("Test: Zadní tlačítka (back_buttons)");
    back_buttons(25); // Otočí se o 180 stupňů (nebo jiná testovací akce)

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
    rkSmartServoSoftMove(0, 90, 100); // Nastaví servo na 90°
    delay(700);
    rkSmartServoSoftMove(0, 150, 100); // Nastaví servo na 150°
    delay(700);
    rkSmartServoSoftMove(0, 120, 100); // Vrátí zpět na 120°
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