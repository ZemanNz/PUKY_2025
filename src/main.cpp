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

// Cílové souřadnice pro jednotlivé jízdy (kola)
int jizda1_x = 2150;
int jizda1_y = 1700;

int jizda2_x = 1900;
int jizda2_y = 1500;

int jizda3_x = 1500;
int jizda3_y = 1300;

int jizda4_x = 1200;
int jizda4_y = 2050;

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
    if (U1_distance < distance_mereni || U2_distance < distance_mereni || U4_distance < distance_mereni) {
        rkBuzzerSet(true);
        delay(100);
        rkBuzzerSet(false);
        return true;
    }
    return false;
}
bool davam_prednost_z_prava = false;
bool detekce_soupere(int distance_mereni = 200){
    if(je_neco_vepredu(distance_mereni)) {
        Serial.println("Detekovan souper! Zastavuji.");
        rkMotorsSetSpeed(-22, -22); // Zastaví motory
        rkBuzzerSet(true);
        delay(300);
        rkBuzzerSet(false);
        delay(900);
        rkMotorsSetSpeed(0, 0); // Zastaví motory
        return true; // Detekován soupeř
    }
    return false;
}


TaskHandle_t chytejPukyHandle = NULL; // Globální handle pro task
TaskHandle_t mainTaskHandle = NULL;    // Globální handle pro hlavní loop task

enum Barva { RED, BLUE};

struct DetekceBarvy {
    bool je_tam;
    Barva barva;
};

// Funkce pro detekci barvy ze senzoru
DetekceBarvy Detekce_puku() {
    DetekceBarvy vysledek;
    vysledek.je_tam = false;

    // Precetni puky senzoru
    rkColorSensorGetRGB("puky", &r1, &g1, &b1);
    delay(20);
    rkColorSensorGetRGB("puky", &r1, &g1, &b1);
    delay(20);
    rkColorSensorGetRGB("puky", &r1, &g1, &b1);
    delay(20);

    // Precetni zem senzoru
    rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    delay(20);
    rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    delay(20);
    rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    delay(20);

    if (rkColorSensorGetRGB("puky", &r1, &g1, &b1) && rkColorSensorGetRGB("zem", &r2, &g2, &b2)) {
        bool is_ground_red = (r2 > 130.0f) && ((r2 - g2) > 40.0f);
        bool is_ground_blue = (b2 > 80.0f) && ((b2 - r2) > 10.0f);

        if ((r1 > 120.0f) && ((r1 - g1) > 50.0f) && ((r1 - b1) > 50.0f) && !is_ground_red) {
            vysledek.barva = RED;
            vysledek.je_tam = true;
            Serial.println(">> NALEZEN CERVENY PUK");
        } else if ((b1 > 120.0f) && ((b1 - r1) > 80.0f) && ((b1 - g1) > 40.0f) && !is_ground_blue) {
            vysledek.barva = BLUE;
            vysledek.je_tam = true;
            Serial.println(">> NALEZEN MODRY PUK");
        }
    }
    return vysledek;
}

DetekceBarvy Kde_jsme() {
    DetekceBarvy vysledek;
    vysledek.je_tam = false;

    Serial.println("[Kde_jsme] Ctu RGB zeme (pokus 1)...");
    bool ok1 = rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    Serial.printf("[Kde_jsme] Pokus 1: ok=%d, R=%.3f, G=%.3f, B=%.3f\n", ok1, r2, g2, b2);
    delay(20);

    Serial.println("[Kde_jsme] Ctu RGB zeme (pokus 2)...");
    bool ok2 = rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    Serial.printf("[Kde_jsme] Pokus 2: ok=%d, R=%.3f, G=%.3f, B=%.3f\n", ok2, r2, g2, b2);
    delay(20);

    Serial.println("[Kde_jsme] Ctu RGB zeme (pokus 3)...");
    bool ok3 = rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    Serial.printf("[Kde_jsme] Pokus 3: ok=%d, R=%.3f, G=%.3f, B=%.3f\n", ok3, r2, g2, b2);
    delay(20);

    Serial.println("[Kde_jsme] Ctu RGB zeme (hlavni cteni)...");
    bool ok4 = rkColorSensorGetRGB("zem", &r2, &g2, &b2);
    Serial.printf("[Kde_jsme] Hlavni cteni: ok=%d, R=%.3f, G=%.3f, B=%.3f\n", ok4, r2, g2, b2);

    if (ok4) {
        Serial.printf("[Kde_jsme] Kriterium RED: (R - 38 > B && R - 38 > G) -> (%.3f - 38 > %.3f && %.3f - 38 > %.3f) -> (%.3f > %.3f && %.3f > %.3f)\n",
                      r2, b2, r2, g2, r2 - 38.0f, b2, r2 - 38.0f, g2);
        Serial.printf("[Kde_jsme] Kriterium BLUE: (B - 15 > R && B > 85) -> (%.3f - 15 > %.3f && %.3f > 85) -> (%.3f > %.3f && %.3f > 85)\n",
                      b2, r2, b2, b2 - 15.0f, r2, b2);

        if ((r2 - 38) > b2 && (r2 - 38) > g2) {
            vysledek.barva = RED;
            vysledek.je_tam = true;
            Serial.println("[Kde_jsme] Detekovana cervena zeme.");
        } else if ((b2 - 15 > r2) && (b2 > 85)) {
            vysledek.barva = BLUE;
            vysledek.je_tam = true;
            Serial.println("[Kde_jsme] Detekovana modra zeme.");
        } else {
            Serial.println("[Kde_jsme] Zadne zname zbarveni zeme.");
        }
    } else {
        Serial.println("[Kde_jsme] Selhalo hlavni cteni senzoru zeme.");
    }
    return vysledek;
}
void puk_do_l_boxu(){
  rkSmartServoMove(0, 160, 180);
}
void puk_do_r_boxu(){
  rkSmartServoMove(0, 86, 180);
}
void zavreni_dvirek(){
  rkSmartServoMove(0, 125, 130); // nahoru
  delay(200);
}
void otevreni_prepazky(){
  rkServosSetPosition(3, -90); // Servo 1 nastaví pnuti po 3minutach 90°...
  delay(200);
}
void zavreni_prepazky(){
  rkServosSetPosition(3, 0); // Servo 1 nastaví pnuti po 3minutach 90°
  delay(200);
}
void StopTask(void *pvParameters) {
    vTaskDelay(3 * 60 * 1000 / portTICK_PERIOD_MS); // 3 minuty
    Serial.println("Uplynuly 3 minuty, vypinam pohyb a blikam...");
    
    // Suspendování hlavního úkolu, aby nevykonával další instrukce jízdy
    if (mainTaskHandle != NULL) {
        vTaskSuspend(mainTaskHandle);
    }
    
    // Zastavení tasku pro chytání puků
    if (chytejPukyHandle != NULL) {
        vTaskDelete(chytejPukyHandle);
        chytejPukyHandle = NULL;
    }
    
    // Zastavení motorů
    rkMotorsSetSpeed(0, 0);
    
    // Nekonečná smyčka blikání všech LEDek
    bool led_state = false;
    while (true) {
        led_state = !led_state;
        rkLedGreen(led_state);
        rkLedYellow(led_state);
        rkLedRed(led_state);
        rkLedBlue(led_state);
        rkMotorsSetSpeed(0, 0);
        delay(300);
    }
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
                rkMotorsDrive(100, 100, 25.0f, 25.0f);
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
                  rkMotorsDrive(100, 100, 25.0f, 25.0f);
                  zavreni_dvirek();
                  delay(400);
                  otevreni_prepazky();
                  puck_in_r_box++;
                  Serial.printf("Puky v pravém boxu: %d\n", puck_in_r_box);
                }
            }
            puck_handling = false;
        }
        vTaskDelay(200 / portTICK_PERIOD_MS); // malá pauza, aby task nebyl příliš rychlý
    }
    vTaskDelete(NULL); // kdyby někdy skončil
}
///////////////////
void jed_a_chytej_puky(int distance, bool x, bool kladna = true, int opponent_detection_dist = 200){
  if (distance <= 0) return;
  // Reset pozic enkodérů
  rkMotorsSetPositionLeft(0);
  rkMotorsSetPositionRight(0);
  otevreni_prepazky(); // Otevře prepážku

  puck_handling = false; // Reset to false just in case
  xTaskCreate(ChytejPukyTask, "ChytejPuky", 2048, NULL, 1, &chytejPukyHandle);

  Serial.printf("Jedu rovne (%d mm)\n", distance);

  float pos_left = 0;
  float pos_right = 0;
  
  // Acceleration/deceleration parameters
  float const max_s = 25.0f; // maximum speed (slower than 20% by user request)
  float const min_s = 18.0f;  // minimum speed (so it overcomes friction but starts slow)
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

      if(detekce_soupere(opponent_detection_dist)) {
          break;
      }
      delay(10);
  }

  // Přečtení finální pozice
  pos_left = abs(rkMotorsGetPositionLeft(true));
  pos_right = abs(rkMotorsGetPositionRight(true));
  float ujeta_vzdalenost = (pos_left + pos_right) / 2.0f;
  
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
  Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  vTaskDelete(chytejPukyHandle);
  chytejPukyHandle = NULL; // Uvolníme task, pokud běžel
  rkMotorsSetSpeed(0, 0); // Zastaví motory
  delay(100);
  rkMotorsSetPositionLeft(0);
  rkMotorsSetPositionRight(0);
  zavreni_dvirek();
}

void smart_align_wall(bool axis_x, int wall_coord, int target_reset_val, float speed) {
    int current_val = axis_x ? aktualni_pozice.x : aktualni_pozice.y;
    int dist = abs(current_val - wall_coord);
                  
    if (dist <= 600) {
        Serial.println("Srovnavam se o stenu (couvani)");
        back_buttons(speed);
        if (axis_x) {
            aktualni_pozice.x = target_reset_val;
        } else {
            aktualni_pozice.y = target_reset_val;
        }
        Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
    }
}

// Makra pro automatický výpis aktuální pozice před a po otočení
#define turn_on_spot_left(angle, speed) do { \
    Serial.printf("Otacim se vlevo (%d stupnu)\n", (int)(angle)); \
    Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y); \
    (turn_on_spot_left)(angle, speed); \
    Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y); \
} while(0)

#define turn_on_spot_right(angle, speed) do { \
    Serial.printf("Otacim se vpravo (%d stupnu)\n", (int)(angle)); \
    Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y); \
    (turn_on_spot_right)(angle, speed); \
    Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y); \
} while(0)

void dojed_na_svoje(bool modra){
  Serial.printf("[dojed_na_svoje] Spusteno s parametrem modra=%d\n", modra);
  Serial.printf("[dojed_na_svoje] Aktualni pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.println("[dojed_na_svoje] Volam Kde_jsme()...");
  auto a = Kde_jsme(); // Zjistí, kde jsme
  Serial.printf("[dojed_na_svoje] Navrat z Kde_jsme(): je_tam=%d, barva=%d\n", a.je_tam, a.barva);

  if(modra){
    if ((a.je_tam && a.barva == BLUE) || (aktualni_pozice.y <= 600 && aktualni_pozice.x <= 600)) {
      Serial.println("[dojed_na_svoje] Podminka splnena. Jsem na modre barve (nebo doma dle odometrie).");
    } else {
      Serial.println("[dojed_na_svoje] Podminka nesplnena. Zacinam hledat modrou...");
      int krok = 0;
      while(true) { // dokud nejsme na modré barvě
        krok++;
        Serial.printf("[dojed_na_svoje] Krok %d: otaceni vlevo 90...\n", krok);
        turn_on_spot_left(90, 20.0f); // otočí se o 90 stupňů vlevo (původně vpravo)
        delay(1000);
        
        Serial.printf("[dojed_na_svoje] Krok %d: couvani na zadni tlacitka...\n", krok);
        back_buttons(25.0f); // couvá na tlačítka
        delay(200);

        Serial.printf("[dojed_na_svoje] Krok %d: volam Kde_jsme() po couvani...\n", krok);
        a = Kde_jsme(); // Zjistí, kde jsme
        Serial.printf("[dojed_na_svoje] Krok %d: Kde_jsme() navratilo je_tam=%d, barva=%d\n", krok, a.je_tam, a.barva);
        
        if(a.je_tam && a.barva == BLUE) {
          Serial.println("[dojed_na_svoje] Modra nalezena po couvani!");
          rkBuzzerSet(true);
          delay(1000);
          rkBuzzerSet(false);
          turn_on_spot_left(180, 20.0f);
          back_buttons(25.0f);
          forward(80, 30.0f);
          turn_on_spot_right(90, 20.0f);
          back_buttons(25.0f);
          forward(80, 30.0f);
          aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
          aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
          Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
          delay(200);
          break;
        }
        
        Serial.printf("[dojed_na_svoje] Krok %d: jed_a_chytej_puky 2800...\n", krok);
        jed_a_chytej_puky(2800, true, false); // jede do středu
        
        Serial.printf("[dojed_na_svoje] Krok %d: volam Kde_jsme() po jizde vpred...\n", krok);
        a = Kde_jsme(); // Zjistí, kde jsme
        Serial.printf("[dojed_na_svoje] Krok %d: Kde_jsme() navratilo je_tam=%d, barva=%d\n", krok, a.je_tam, a.barva);
        
        if(a.je_tam && a.barva == BLUE) {
          Serial.println("[dojed_na_svoje] Modra nalezena pri jizde vpred!");
          rkBuzzerSet(true);
          delay(1000);
          rkBuzzerSet(false);
          turn_on_spot_left(180, 20.0f);
          back_buttons(25.0f);
          forward(80, 30.0f);
          turn_on_spot_right(90, 20.0f);
          back_buttons(25.0f);
          forward(80, 30.0f);
          aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
          aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
          Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
          delay(200);
          break;
        }
        delay(200);
      }
    }
  }
  else{
    if ((a.je_tam && a.barva == RED) || (aktualni_pozice.y <= 600 && aktualni_pozice.x <= 600)) {
      Serial.println("[dojed_na_svoje] Podminka splnena. Jsem na RED barve (nebo doma dle odometrie).");
    } else {
      Serial.println("[dojed_na_svoje] Podminka nesplnena. Zacinam hledat RED...");
      int krok = 0;
      while(true) { // dokud nejsme na modré barvě
        krok++;
        Serial.printf("[dojed_na_svoje] Krok %d: otaceni vlevo 90...\n", krok);
        turn_on_spot_left(90, 20.0f); // otočí se o 90 stupňů vlevo (původně vpravo)
        delay(1000);
        
        Serial.printf("[dojed_na_svoje] Krok %d: couvani na zadni tlacitka...\n", krok);
        back_buttons(25.0f); // couvá na tlačítka
        delay(200);

        Serial.printf("[dojed_na_svoje] Krok %d: volam Kde_jsme() po couvani...\n", krok);
        a = Kde_jsme(); // Zjistí, kde jsme
        Serial.printf("[dojed_na_svoje] Krok %d: Kde_jsme() navratilo je_tam=%d, barva=%d\n", krok, a.je_tam, a.barva);
        
        if(a.je_tam && a.barva == RED) {
          Serial.println("[dojed_na_svoje] RED nalezena po couvani!");
          rkBuzzerSet(true);
          delay(1000);
          rkBuzzerSet(false);
          turn_on_spot_right(182, 20.0f);
          back_buttons(25.0f);
          forward(70, 30.0f);
          turn_on_spot_right(90, 20.0f); // doprava o 90
          back_buttons(25.0f);
          forward(70, 30.0f);
          aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
          aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
          Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
          delay(200);
          break;
        }
        
        Serial.printf("[dojed_na_svoje] Krok %d: jed_a_chytej_puky 2800...\n", krok);
        jed_a_chytej_puky(2800, true, false); // jede do středu
        
        Serial.printf("[dojed_na_svoje] Krok %d: volam Kde_jsme() po jizde vpred...\n", krok);
        a = Kde_jsme(); // Zjistí, kde jsme
        Serial.printf("[dojed_na_svoje] Krok %d: Kde_jsme() navratilo je_tam=%d, barva=%d\n", krok, a.je_tam, a.barva);
        
        if(a.je_tam && a.barva == RED) {
          Serial.println("[dojed_na_svoje] RED nalezena pri jizde vpred!");
          rkBuzzerSet(true);
          delay(1000);
          rkBuzzerSet(false);
          turn_on_spot_right(182, 20.0f);
          back_buttons(25.0f);
          forward(70, 30.0f);
          turn_on_spot_right(90, 20.0f); // doprava o 90
          back_buttons(25.0f);
          forward(70, 30.0f);
          aktualni_pozice.x = 350; // Nastaví aktuální pozici na střed
          aktualni_pozice.y = 350; // Nastaví aktuální pozici na střed
          Serial.printf("[POZICE] X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
          delay(200);
          break;
        }
        delay(200);
      }
    }
  }
}


void smart_align_y(int current_y) {
    Serial.printf("[SmartAlign] Spusteno. Vstupni Y: %d mm\n", current_y);
    // Vždy popojedeme dopředu o 70 mm z boční stěny (z původního kódu "forward(70, 30.0f)")
    forward(70, 30.0f);
    delay(200);
    
    if (current_y > 600) {
        Serial.printf("[SmartAlign] Vzdalenost %d mm > 600 mm -> otocim a jedu k domovu popredu.\n", current_y);
        // Původně jedeme na západ. Otočením o 90° doleva budeme koukat na jih (k domovu).
        turn_on_spot_left(90, 20.0f);
        delay(500);
        
        // Jdeme dopředu k domovu (Y = 70)
        int drive_dist = current_y - 70;
        forward(drive_dist, 40.0f);
        delay(200);
        
        // Otočíse o 180°, abychom koukali na sever (+Y) jako v klasickém zarovnání
        turn_on_spot_right(180, 20.0f);
        delay(500);
    } else {
        Serial.println("[SmartAlign] Blizko steny -> klasicke zarovnani o zed.");
        turn_on_spot_right(90, 20.0f);
        delay(1000);
        back_buttons(25.0f);
        forward(70, 30.0f);
        delay(200);
    }
    Serial.println("[SmartAlign] Dokonceno.");
}



void modra(){
  Serial.println("\n=== [STRATEGIE] SPUSTENA MODRA ===\n");
  
  // ==================== KOLO 1 ====================
  Serial.printf("\n--- [KOLO 1] START --- Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.printf("[Kolo 1] Jizda na sever: jed_a_chytej_puky(%d, Y-osa)\n", jizda1_y - aktualni_pozice.y);
  jed_a_chytej_puky(jizda1_y - aktualni_pozice.y, false);
  
  Serial.println("[Kolo 1] Otoceni 90° doleva na zapad...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 1] Zarovnani o pravou (Vychodni) stenu (X=0)...");
  smart_align_wall(true, 0, stred_od_zadu, 25.0f);
  delay(100);
  open_l_box();
  delay(100);
  
  Serial.printf("[Kolo 1] Jizda na zapad k levym krabicim: jed_a_chytej_puky(%d, X-osa)\n", jizda1_x - aktualni_pozice.x);
  jed_a_chytej_puky(jizda1_x - aktualni_pozice.x, true);
  
  Serial.println("[Kolo 1] Preskakuji couvani k souperove stene Y=2500.");
  turn_on_spot_left(90, 20.0f); // pouze se otocime na jih
  delay(100);
  backward(50, 15);
  delay(200);
  close_l_box();
  delay(100);
  
  Serial.printf("[Kolo 1] Jizda na jih podel levych krabic: jed_a_chytej_puky(%d, Y-osa, zpet)\n", aktualni_pozice.y - stred_od_predu - 200);
  jed_a_chytej_puky((aktualni_pozice.y - stred_od_predu - 200), false, false);
  
  Serial.println("[Kolo 1] Otoceni 90° doleva na vychod...");
  turn_on_spot_left(90, 20.0f);

  smart_align_wall(true, 2500, 2500 - stred_od_zadu, 25.0f);
  
  Serial.printf("[Kolo 1] Jizda na vychod k leve krabici: jed_a_chytej_puky(%d, X-osa, zpet)\n", aktualni_pozice.x - stred_od_predu - 200);
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false);
  
  Serial.println("[Kolo 1] Otoceni 182° na zapad...");
  turn_on_spot_left(182, 20.0f);
  
  Serial.println("[Kolo 1] Zarovnani o stenu...");
  //smart_align_wall(true, 0, stred_od_zadu, 25.0f);// tady usime vzdy couvat....
  back_buttons(28.0f);
  delay(200);
  
  Serial.println("[Kolo 1] Spoustim smart_align_y...");

  //nahrazuji smart_align_y(aktualni_pozice.y);

  forward_acc(130, 30.0f);
  delay(200);
  
  turn_on_spot_right(90, 20.0f);
  delay(100);

  back_buttons(25.0f);
  delay(200);
  
  forward_acc(130, 30.0f);
  delay(200);
  
  aktualni_pozice.x = 350;
  aktualni_pozice.y = 350;
  Serial.printf("[Kolo 1] Reset pred dojezdem: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.println("[Kolo 1] Spoustim dojed_na_svoje...");
  dojed_na_svoje(true);
  
  Serial.println("[Kolo 1] Vykladani nasich puku (levy box)...");
  open_r_box();
  forward_acc(240, 25.0f);
  aktualni_pozice.y = 510;
  close_r_box();
  Serial.printf("[Kolo 1] UKONCENO. Aktualni pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  // ==================== KOLO 2 ====================
  Serial.printf("\n--- [KOLO 2] START --- Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.printf("[Kolo 2] Jizda na sever: jed_a_chytej_puky(%d, Y-osa)\n", jizda2_y - aktualni_pozice.y);
  jed_a_chytej_puky(jizda2_y - aktualni_pozice.y, false);
  
  Serial.println("[Kolo 2] Otoceni 90° doleva na zapad...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 2] Zarovnani o Vychodni stenu...");
  smart_align_wall(true, 0, stred_od_zadu, 25.0f);
  delay(100);
  open_l_box();
  delay(100);
  
  Serial.printf("[Kolo 2] Jizda na zapad: jed_a_chytej_puky(%d, X-osa)\n", jizda2_x - aktualni_pozice.x);
  jed_a_chytej_puky(jizda2_x - aktualni_pozice.x, true);
  
  Serial.println("[Kolo 2] Otoceni 90° doleva na jih...");
  turn_on_spot_left(90, 20.0f);
  delay(100);
  backward(50, 15);
  delay(200);
  close_l_box();
  delay(100);
  
  
  // Serial.println("[Kolo 2] Popojeti na jih: jed_a_chytej_puky(600, Y-osa, zpet)...");
  jed_a_chytej_puky((aktualni_pozice.y - stred_od_predu - 550), false, false);
  // close_l_box();
  
  Serial.println("[Kolo 2] Otoceni 90° doleva na vychod...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 2] Zarovnani o Zapadni stenu (na zacatku vlevo)...");
  smart_align_wall(true, 2500, 2500 - stred_od_zadu, 25.0f);

  open_l_box();
  
  Serial.printf("[Kolo 2] Jizda na vychod k pravym krabicim: jed_a_chytej_puky(%d, X-osa, zpet)\n", aktualni_pozice.x - stred_od_predu - 200);
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 800, true, false);
  backward(50, 15);
  delay(200);
  close_l_box();

  jed_a_chytej_puky((aktualni_pozice.x - stred_od_predu - 200), true, false);
  
  Serial.println("[Kolo 2] Otoceni 180° na zapad...");
  turn_on_spot_left(180, 20.0f);
  
  Serial.println("[Kolo 2] Zarovnani o stenu...");
  //smart_align_wall(true, 0, stred_od_zadu, 25.0f);// tady usime vzdy couvat....
  back_buttons(28.0f);
  delay(200);
    
  Serial.println("[Kolo 2] Spoustim smart_align_y...");
  forward_acc(130, 30.0f);
  delay(200);
  
  turn_on_spot_right(90, 20.0f);
  delay(100);

  back_buttons(25.0f);
  delay(200);
  
  forward_acc(130, 30.0f);
  delay(200);
  
  aktualni_pozice.x = 350;
  aktualni_pozice.y = 350;
  Serial.printf("[Kolo 2] Reset pred dojezdem: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.println("[Kolo 2] Spoustim dojed_na_svoje...");
  dojed_na_svoje(true);
  
  Serial.println("[Kolo 2] Vykladani nasich puku (levy box)...");
  open_r_box();
  forward_acc(240, 25.0f);
  aktualni_pozice.y = 510;
  close_r_box();
  Serial.printf("[Kolo 2] UKONCENO. Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  // ==================== KOLO 3 ====================
  Serial.printf("\n--- [KOLO 3] START --- Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.printf("[Kolo 3] Jizda na sever: jed_a_chytej_puky(%d, Y-osa)\n", jizda3_y - aktualni_pozice.y);
  jed_a_chytej_puky(jizda3_y - aktualni_pozice.y, false);
  
  Serial.println("[Kolo 3] Otoceni 90° doleva na zapad...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 3] Zarovnani o Vychodni stenu...");
  smart_align_wall(true, 0, stred_od_zadu, 25.0f);
  delay(100);
  open_l_box();
  delay(100);
  
  Serial.printf("[Kolo 3] Jizda na zapad: jed_a_chytej_puky(%d, X-osa)\n", jizda3_x - aktualni_pozice.x);
  jed_a_chytej_puky(jizda3_x - aktualni_pozice.x, true);
  
  Serial.println("[Kolo 3] Otoceni 90° doleva na jih...");
  turn_on_spot_left(90, 20.0f);
  delay(100);
  backward(50, 15);
  delay(200);
  close_l_box();
  delay(100);
  
  Serial.println("[Kolo 3] Popojeti na jih: jed_a_chytej_puky(600, Y-osa, zpet)...");
  jed_a_chytej_puky(600, false, false);
  
  Serial.println("[Kolo 3] Otoceni 90° doleva na vychod...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 3] Zarovnani o Zapadni stenu (na zacatku vlevo)...");
  smart_align_wall(true, 2500, 2500 - stred_od_zadu, 25.0f);
  
  Serial.printf("[Kolo 3] Jizda na vychod: jed_a_chytej_puky(%d, X-osa, zpet)\n", aktualni_pozice.x - stred_od_predu - 200);
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false);
  
  Serial.println("[Kolo 3] Otoceni 180° na zapad...");
  turn_on_spot_left(180, 20.0f);
  
  Serial.println("[Kolo 3] Zarovnani o stenu...");
  //smart_align_wall(true, 0, stred_od_zadu, 25.0f);// tady usime vzdy couvat....
  back_buttons(28.0f);
  delay(200);
    
  Serial.println("[Kolo 3] Spoustim smart_align_y...");
  forward_acc(130, 30.0f);
  delay(200);
  
  turn_on_spot_right(90, 20.0f);
  delay(100);

  back_buttons(25.0f);
  delay(200);
  
  forward_acc(130, 30.0f);
  delay(200);
  
  aktualni_pozice.x = 350;
  aktualni_pozice.y = 350;
  Serial.printf("[Kolo 3] Reset pred dojezdem: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.println("[Kolo 3] Spoustim dojed_na_svoje...");
  dojed_na_svoje(true);
  
  Serial.println("[Kolo 3] Vykladani nasich puku (levy box)...");
  open_r_box();
  forward_acc(240, 25.0f);
  aktualni_pozice.y = 510;
  close_r_box();
  Serial.printf("[Kolo 3] UKONCENO. Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  // ==================== KOLO 4 ====================
  Serial.printf("\n--- [KOLO 4] START --- Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.printf("[Kolo 4] Jizda na sever: jed_a_chytej_puky(%d, Y-osa)\n", jizda4_y - aktualni_pozice.y);
  jed_a_chytej_puky(1600 - aktualni_pozice.y, false );
  jed_a_chytej_puky(jizda4_y - aktualni_pozice.y, false, true, 50);
  
  Serial.println("[Kolo 4] Otoceni 90° doleva na zapad...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 4] Zarovnani o Vychodni stenu...");
  smart_align_wall(true, 0, stred_od_zadu, 25.0f);
  
  Serial.printf("[Kolo 4] Jizda na zapad: jed_a_chytej_puky(%d, X-osa)\n", jizda4_x - aktualni_pozice.x);
  jed_a_chytej_puky(jizda4_x - aktualni_pozice.x, true);
  
  Serial.println("[Kolo 4] Otoceni 90° doleva na jih...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 4] Popojeti na jih: jed_a_chytej_puky(600, Y-osa, zpet)...");
  jed_a_chytej_puky((aktualni_pozice.y - stred_od_predu - 550), false, false);
  
  Serial.println("[Kolo 4] Otoceni 90° doleva na vychod...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 4] Zarovnani o Zapadni stenu (na zacatku vlevo)...");
  smart_align_wall(true, 2500, 2500 - stred_od_zadu, 25.0f);
  
  Serial.printf("[Kolo 4] Jizda na vychod: jed_a_chytej_puky(%d, X-osa, zpet)\n", aktualni_pozice.x - stred_od_predu - 200);
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false);
  
  Serial.println("[Kolo 4] Otoceni 180° na zapad...");
  turn_on_spot_left(180, 20.0f);
  
  Serial.println("[Kolo 4] Zarovnani o stenu...");
  //smart_align_wall(true, 0, stred_od_zadu, 25.0f);// tady usime vzdy couvat....
  back_buttons(28.0f);
  delay(200);
    
  Serial.println("[Kolo 4] Spoustim smart_align_y...");
  forward_acc(130, 30.0f);
  delay(200);
  
  turn_on_spot_right(90, 20.0f);
  delay(100);

  back_buttons(25.0f);
  delay(200);
  
  forward_acc(130, 30.0f);
  delay(200);
  
  aktualni_pozice.x = 350;
  aktualni_pozice.y = 350;
  Serial.printf("[Kolo 4] Reset pred dojezdem: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.println("[Kolo 4] Spoustim dojed_na_svoje...");
  dojed_na_svoje(true);
  
  Serial.println("[Kolo 4] Vykladani nasich puku (levy box)...");
  open_r_box();
  forward(180, 25.0f);
  aktualni_pozice.y = 450;
  close_r_box();
  
  Serial.println("\n=== [STRATEGIE] MODRA KOMPLETNE DOKONCENA ===\n");
  
  rkBuzzerSet(true);
  delay(500);
  rkBuzzerSet(false);
  delay(500);
  rkBuzzerSet(true);
  delay(500);
  rkBuzzerSet(false);
}
void cervena(){
  Serial.println("\n=== [STRATEGIE] SPUSTENA CERVENA ===\n");
  
  // ==================== KOLO 1 ====================
  Serial.printf("\n--- [KOLO 1] START --- Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.printf("[Kolo 1] Jizda na sever: jed_a_chytej_puky(%d, Y-osa)\n", jizda1_y - aktualni_pozice.y);
  jed_a_chytej_puky(jizda1_y - aktualni_pozice.y, false);
  
  Serial.println("[Kolo 1] Otoceni 90° doleva na zapad...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 1] Zarovnani o pravou (Vychodni) stenu (X=0)...");
  smart_align_wall(true, 0, stred_od_zadu, 25.0f);
  delay(100);
  open_r_box();
  delay(100);
  
  Serial.printf("[Kolo 1] Jizda na zapad k levym krabicim: jed_a_chytej_puky(%d, X-osa)\n", jizda1_x - aktualni_pozice.x);
  jed_a_chytej_puky(jizda1_x - aktualni_pozice.x, true);
  
  Serial.println("[Kolo 1] Preskakuji couvani k souperove stene Y=2500.");
  turn_on_spot_left(90, 20.0f); // pouze se otocime na jih
  delay(100);
  backward(50, 15);
  delay(200);
  close_r_box();
  delay(100);
  
  Serial.printf("[Kolo 1] Jizda na jih podel levych krabic: jed_a_chytej_puky(%d, Y-osa, zpet)\n", aktualni_pozice.y - stred_od_predu - 200);
  jed_a_chytej_puky((aktualni_pozice.y - stred_od_predu - 200), false, false);
  
  Serial.println("[Kolo 1] Otoceni 90° doleva na vychod...");
  turn_on_spot_left(90, 20.0f);

  smart_align_wall(true, 2500, 2500 - stred_od_zadu, 25.0f);
  
  Serial.printf("[Kolo 1] Jizda na vychod k leve krabici: jed_a_chytej_puky(%d, X-osa, zpet)\n", aktualni_pozice.x - stred_od_predu - 200);
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false);
  
  Serial.println("[Kolo 1] Otoceni 182° na zapad...");
  turn_on_spot_left(182, 20.0f);
  
  Serial.println("[Kolo 1] Zarovnani o stenu...");
  //smart_align_wall(true, 0, stred_od_zadu, 25.0f);// tady usime vzdy couvat....
  back_buttons(28.0f);
  delay(200);
  
  Serial.println("[Kolo 1] Spoustim smart_align_y...");

  //nahrazuji smart_align_y(aktualni_pozice.y);

  forward_acc(130, 30.0f);
  delay(200);
  
  turn_on_spot_right(90, 20.0f);
  delay(100);

  back_buttons(25.0f);
  delay(200);
  
  forward_acc(130, 30.0f);
  delay(200);
  
  aktualni_pozice.x = 350;
  aktualni_pozice.y = 350;
  Serial.printf("[Kolo 1] Reset pred dojezdem: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.println("[Kolo 1] Spoustim dojed_na_svoje...");
  dojed_na_svoje(false);
  
  Serial.println("[Kolo 1] Vykladani nasich puku (levy box)...");
  open_l_box();
  forward_acc(240, 25.0f);
  aktualni_pozice.y = 510;
  close_l_box();
  Serial.printf("[Kolo 1] UKONCENO. Aktualni pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  // ==================== KOLO 2 ====================
  Serial.printf("\n--- [KOLO 2] START --- Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.printf("[Kolo 2] Jizda na sever: jed_a_chytej_puky(%d, Y-osa)\n", jizda2_y - aktualni_pozice.y);
  jed_a_chytej_puky(jizda2_y - aktualni_pozice.y, false);
  
  Serial.println("[Kolo 2] Otoceni 90° doleva na zapad...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 2] Zarovnani o Vychodni stenu...");
  smart_align_wall(true, 0, stred_od_zadu, 25.0f);
  delay(100);
  open_r_box();
  delay(100);
  
  Serial.printf("[Kolo 2] Jizda na zapad: jed_a_chytej_puky(%d, X-osa)\n", jizda2_x - aktualni_pozice.x);
  jed_a_chytej_puky(jizda2_x - aktualni_pozice.x, true);
  
  Serial.println("[Kolo 2] Otoceni 90° doleva na jih...");
  turn_on_spot_left(90, 20.0f);
  delay(100);
  backward(50, 15);
  delay(200);
  close_r_box();
  delay(100);
  
  
  // Serial.println("[Kolo 2] Popojeti na jih: jed_a_chytej_puky(600, Y-osa, zpet)...");
  jed_a_chytej_puky((aktualni_pozice.y - stred_od_predu - 550), false, false);
  // close_r_box();
  
  Serial.println("[Kolo 2] Otoceni 90° doleva na vychod...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 2] Zarovnani o Zapadni stenu (na zacatku vlevo)...");
  smart_align_wall(true, 2500, 2500 - stred_od_zadu, 25.0f);

  open_r_box();
  
  Serial.printf("[Kolo 2] Jizda na vychod k pravym krabicim: jed_a_chytej_puky(%d, X-osa, zpet)\n", aktualni_pozice.x - stred_od_predu - 200);
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 800, true, false);
  backward(50, 15);
  delay(200);
  close_r_box();

  jed_a_chytej_puky((aktualni_pozice.x - stred_od_predu - 200), true, false);
  
  Serial.println("[Kolo 2] Otoceni 180° na zapad...");
  turn_on_spot_left(180, 20.0f);
  
  Serial.println("[Kolo 2] Zarovnani o stenu...");
  //smart_align_wall(true, 0, stred_od_zadu, 25.0f);// tady usime vzdy couvat....
  back_buttons(28.0f);
  delay(200);
    
  Serial.println("[Kolo 2] Spoustim smart_align_y...");
  forward_acc(130, 30.0f);
  delay(200);
  
  turn_on_spot_right(90, 20.0f);
  delay(100);

  back_buttons(25.0f);
  delay(200);
  
  forward_acc(130, 30.0f);
  delay(200);
  
  aktualni_pozice.x = 350;
  aktualni_pozice.y = 350;
  Serial.printf("[Kolo 2] Reset pred dojezdem: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.println("[Kolo 2] Spoustim dojed_na_svoje...");
  dojed_na_svoje(false);
  
  Serial.println("[Kolo 2] Vykladani nasich puku (levy box)...");
  open_l_box();
  forward_acc(240, 25.0f);
  aktualni_pozice.y = 510;
  close_l_box();
  Serial.printf("[Kolo 2] UKONCENO. Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  // ==================== KOLO 3 ====================
  Serial.printf("\n--- [KOLO 3] START --- Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.printf("[Kolo 3] Jizda na sever: jed_a_chytej_puky(%d, Y-osa)\n", jizda3_y - aktualni_pozice.y);
  jed_a_chytej_puky(jizda3_y - aktualni_pozice.y, false);
  
  Serial.println("[Kolo 3] Otoceni 90° doleva na zapad...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 3] Zarovnani o Vychodni stenu...");
  smart_align_wall(true, 0, stred_od_zadu, 25.0f);
  delay(100);
  open_r_box();
  delay(100);
  
  Serial.printf("[Kolo 3] Jizda na zapad: jed_a_chytej_puky(%d, X-osa)\n", jizda3_x - aktualni_pozice.x);
  jed_a_chytej_puky(jizda3_x - aktualni_pozice.x, true);
  
  Serial.println("[Kolo 3] Otoceni 90° doleva na jih...");
  turn_on_spot_left(90, 20.0f);
  delay(100);
  backward(50, 15);
  delay(200);
  close_r_box();
  delay(100);
  
  Serial.println("[Kolo 3] Popojeti na jih: jed_a_chytej_puky(600, Y-osa, zpet)...");
  jed_a_chytej_puky(600, false, false);
  
  Serial.println("[Kolo 3] Otoceni 90° doleva na vychod...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 3] Zarovnani o Zapadni stenu (na zacatku vlevo)...");
  smart_align_wall(true, 2500, 2500 - stred_od_zadu, 25.0f);
  
  Serial.printf("[Kolo 3] Jizda na vychod: jed_a_chytej_puky(%d, X-osa, zpet)\n", aktualni_pozice.x - stred_od_predu - 200);
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false);
  
  Serial.println("[Kolo 3] Otoceni 180° na zapad...");
  turn_on_spot_left(180, 20.0f);
  
  Serial.println("[Kolo 3] Zarovnani o stenu...");
  //smart_align_wall(true, 0, stred_od_zadu, 25.0f);// tady usime vzdy couvat....
  back_buttons(28.0f);
  delay(200);
    
  Serial.println("[Kolo 3] Spoustim smart_align_y...");
  forward_acc(130, 30.0f);
  delay(200);
  
  turn_on_spot_right(90, 20.0f);
  delay(100);

  back_buttons(25.0f);
  delay(200);
  
  forward_acc(130, 30.0f);
  delay(200);
  
  aktualni_pozice.x = 350;
  aktualni_pozice.y = 350;
  Serial.printf("[Kolo 3] Reset pred dojezdem: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.println("[Kolo 3] Spoustim dojed_na_svoje...");
  dojed_na_svoje(false);
  
  Serial.println("[Kolo 3] Vykladani nasich puku (levy box)...");
  open_l_box();
  forward_acc(240, 25.0f);
  aktualni_pozice.y = 510;
  close_l_box();
  Serial.printf("[Kolo 3] UKONCENO. Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  // ==================== KOLO 4 ====================
  Serial.printf("\n--- [KOLO 4] START --- Pozice: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.printf("[Kolo 4] Jizda na sever: jed_a_chytej_puky(%d, Y-osa)\n", jizda4_y - aktualni_pozice.y);
  jed_a_chytej_puky(1600 - aktualni_pozice.y, false );
  jed_a_chytej_puky(jizda4_y - aktualni_pozice.y, false, true, 50);
  
  Serial.println("[Kolo 4] Otoceni 90° doleva na zapad...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 4] Zarovnani o Vychodni stenu...");
  smart_align_wall(true, 0, stred_od_zadu, 25.0f);
  
  Serial.printf("[Kolo 4] Jizda na zapad: jed_a_chytej_puky(%d, X-osa)\n", jizda4_x - aktualni_pozice.x);
  jed_a_chytej_puky(jizda4_x - aktualni_pozice.x, true);
  
  Serial.println("[Kolo 4] Otoceni 90° doleva na jih...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 4] Popojeti na jih: jed_a_chytej_puky(600, Y-osa, zpet)...");
  jed_a_chytej_puky((aktualni_pozice.y - stred_od_predu - 550), false, false);
  
  Serial.println("[Kolo 4] Otoceni 90° doleva na vychod...");
  turn_on_spot_left(90, 20.0f);
  
  Serial.println("[Kolo 4] Zarovnani o Zapadni stenu (na zacatku vlevo)...");
  smart_align_wall(true, 2500, 2500 - stred_od_zadu, 25.0f);
  
  Serial.printf("[Kolo 4] Jizda na vychod: jed_a_chytej_puky(%d, X-osa, zpet)\n", aktualni_pozice.x - stred_od_predu - 200);
  jed_a_chytej_puky(aktualni_pozice.x - stred_od_predu - 200, true, false);
  
  Serial.println("[Kolo 4] Otoceni 180° na zapad...");
  turn_on_spot_left(180, 20.0f);
  
  Serial.println("[Kolo 4] Zarovnani o stenu...");
  //smart_align_wall(true, 0, stred_od_zadu, 25.0f);// tady usime vzdy couvat....
  back_buttons(28.0f);
  delay(200);
    
  Serial.println("[Kolo 4] Spoustim smart_align_y...");
  forward_acc(130, 30.0f);
  delay(200);
  
  turn_on_spot_right(90, 20.0f);
  delay(100);

  back_buttons(25.0f);
  delay(200);
  
  forward_acc(130, 30.0f);
  delay(200);
  
  aktualni_pozice.x = 350;
  aktualni_pozice.y = 350;
  Serial.printf("[Kolo 4] Reset pred dojezdem: X=%d, Y=%d\n", aktualni_pozice.x, aktualni_pozice.y);
  
  Serial.println("[Kolo 4] Spoustim dojed_na_svoje...");
  dojed_na_svoje(false);
  
  Serial.println("[Kolo 4] Vykladani nasich puku (levy box)...");
  open_l_box();
  forward(180, 25.0f);
  aktualni_pozice.y = 450;
  close_l_box();
  
  Serial.println("\n=== [STRATEGIE] CERVENA KOMPLETNE DOKONCENA ===\n");
  
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
    cfg.left_wheel_diameter = 67.86;
    cfg.right_wheel_diameter = 68.14;
    cfg.motor_wheel_diameter = 68.00;
    cfg.roztec_kol = 242.48;

    // Korekční koeficienty
    cfg.konstanta_radius_vnejsi_kolo = 1.0f;
    cfg.konstanta_radius_vnitrni_kolo = 1.0f;
    cfg.korekce_nedotacivosti_left = 1.0f;
    cfg.korekce_nedotacivosti_right = 0.915f;

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
    rkSmartServoInit(0, 0, 240);
    rkLedYellow(true); // Žlutá svítí na začátku při čekání na výběr
    rkLedRed(false);
    rkLedBlue(false);
    rkLedGreen(false);
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
    mainTaskHandle = xTaskGetCurrentTaskHandle();
    rkLedYellow(false);
    rkLedBlue(true);
    rkLedRed(false);
    zavreni_dvirek();
    close_l_box();
    close_r_box();
    xTaskCreate(StopTask, "StopTask", 4096, NULL, 1, NULL);
    delay(1000);
    modra();
  }
  else if(rkButtonIsPressed(BTN_RIGHT)) {
    mainTaskHandle = xTaskGetCurrentTaskHandle();
    rkLedYellow(false);
    rkLedRed(true);
    rkLedBlue(false);
    zavreni_dvirek();
    close_l_box();
    close_r_box();
    xTaskCreate(StopTask, "StopTask", 4096, NULL, 1, NULL);
    delay(1000);
    cervena();
  }
  else if(rkButtonIsPressed(BTN_ON)){
    // === TEST POHYBU ===
    Serial.println("=== TEST POHYBU START ===");

    // 1) Rovně 1 metr
    Serial.println("1) Forward 1000 mm...");
    forward_acc(1500, 20.0f);
    delay(5000);

    // 2) Otočení o 90° doprava
    Serial.println("2) Turn right 90°...");
    turn_on_spot_right(90, 18.0f);
    delay(5000);

    // 3) Otočení o 180° doleva
    Serial.println("3) Turn left 180°...");
    turn_on_spot_left(180, 18.0f);
    delay(5000);

    // 4) Back buttons - couvání ke zdi
    Serial.println("4) Back buttons...");
    back_buttons(18.0f);

    Serial.println("=== TEST POHYBU KONEC ===");
    rkBuzzerSet(true);
    delay(300);
    rkBuzzerSet(false);
  }
  if(rkButtonIsPressed(BTN_UP)){
    Serial.println("=== TEST DOJEZD NA SVOJE (RED) START ===");
    rkBuzzerSet(true);
    delay(100);
    rkBuzzerSet(false);
    
    // Pro jistotu nastavíme odometrickou pozici mimo domov (např. 1000, 1000), 
    // aby se otestovalo samotné chování s detekcí
    aktualni_pozice.x = 1000;
    aktualni_pozice.y = 1000;
    
    dojed_na_svoje(false);
    
    Serial.println("=== TEST DOJEZD NA SVOJE (RED) KONEC ===");
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
    rkMotorsSetSpeed(32.0f, 32.0f);
    delay(1000); // Jede 1 sekundu vpřed
    rkMotorsSetSpeed(0, 0);
    Serial.println("Test: Pohyb vpřed OK");

    // 2. Použití zadních tlačítek (simulace funkce back_buttons)
    Serial.println("Test: Zadní tlačítka (back_buttons)");
    back_buttons(35.0f); // Otočí se o 180 stupňů (nebo jiná testovací akce)

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