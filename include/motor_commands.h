#include "Arduino.h"
#include "robotka.h"
#include <iostream>
#include <thread>
#include "RBCX.h"
/////////////////////////////////////
static const uint8_t Bbutton1 = 34;
static const uint8_t Bbutton2 = 35;
const float roztec = 200.0; // roztec kol v mm
float  koeficient_turn = 1.030; // koeficient pro otáčení na místě

float koeficient_r_f = 1.029;
float koeficient_l_f = 1.018;
float koeficient_r_b = 1.03;
float koeficient_l_b = 1.032;

////////////////////
// Před funkcí:
const float Kp = 55.0f;
const float Ki = 0.01f;
const float Kd = 0.1f;
int a = 300; // koeficient zrychlovani
//////////
void forward(float mm, float speed) {
    int M1_pos = 0, M4_pos = 0, odhylaka = 0, integral = 0, last_odchylka = 0;
    int M1_pos_up = 0, M4_pos_up = 0, M1_pos_start = 0, M4_pos_start = 0;
    int target = (32000 * speed/100);
    // std::cout << "target: " << target << std::endl;

    auto& man = rb::Manager::get(); // vytvoří referenci na man class
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    // Startovní pozice
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
        M1_pos_start = -info.position();
    });
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
        M4_pos_start = info.position();
    });
    delay(50); // pro jistotu, aby se stihly načíst pozice

    // Serial.printf("[FORWARD] Start M1_pos_start: %d, M4_pos_start: %d\n", M1_pos_start, M4_pos_start);

    int M1_must_go = ((mm / (127.5 * 3.141592)) * 40.4124852f * 48.f) + M1_pos_start; // přepočet mm na encodery
    int M4_must_go = ((mm / (127.5 * 3.141592)) * 40.4124852f * 48.f) + M4_pos_start;
    // Serial.printf("[FORWARD] M1_must_go: %d, M4_must_go: %d\n", M1_must_go, M4_must_go);

    // Zrychlování
    for(int i = 0; i < target-1; i += a) {
        odhylaka = M1_pos - M4_pos;
        integral += odhylaka;
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        man.motor(rb::MotorId::M1).power(-i * 0.9);
        man.motor(rb::MotorId::M4).power(i + odhylaka * Kp + integral * Ki + (odhylaka - last_odchylka) * Kd);
        // Serial.printf("[ACCEL] i: %d | M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", i, M1_pos, M4_pos, odhylaka, integral);
        delay(10);
    }

    // Získání aktuální pozice po zrychlení
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
        M1_pos_up = -info.position();
    });
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
        M4_pos_up = info.position();
    });
    delay(50);

    int M1_pos_diff = M1_pos_up - M1_pos_start;
    int M4_pos_diff = M4_pos_up - M4_pos_start;
    // Serial.printf("[AFTER ACCEL] M1_pos_up: %d, M4_pos_up: %d, M1_pos_diff: %d, M4_pos_diff: %d\n", M1_pos_up, M4_pos_up, M1_pos_diff, M4_pos_diff);

    integral = 0; // reset integrálu

    // Jízda na vzdálenost
    while (M1_pos < (M1_must_go - (2 * M1_pos_diff)) && M4_pos < (M4_must_go - (2 * M4_pos_diff))) {
        odhylaka = M1_pos - M4_pos;
        integral += odhylaka;
        integral += odhylaka;
        // omez ho v rozumných mezích, např. ±max_integral
        const int max_integral = 10000;
        if (integral >  max_integral) integral =  max_integral;
        if (integral < -max_integral) integral = -max_integral;
        man.motor(rb::MotorId::M1).power(-target*0.95);
        int rawPower = target + odhylaka * Kp + integral * Ki + (odhylaka - last_odchylka) * Kd;
        // saturace
        const int maxPower = 32000;
        if      (rawPower >  maxPower) rawPower =  maxPower;
        else if (rawPower < -maxPower) rawPower = -maxPower;
        man.motor(rb::MotorId::M4).power(rawPower);


        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });

        // Serial.printf("[CONST] M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d | powerM4: %d\n", M1_pos, M4_pos, odhylaka, integral, rawPower);

        delay(50);
        last_odchylka = odhylaka;
    }
        integral = 0; // reset integrálu

    // Zpomalení
    for(int i = target ; i > 0 ; i -= a) {
        odhylaka = M1_pos - M4_pos;
        integral += odhylaka;

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position(); 
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        if(i < 5000){
          while(M1_pos < ( M1_must_go - (speed/50.0 * 35))){
            delay(10);
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
              M1_pos = -info.position();
            });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
              M4_pos = info.position();
            });
            // Serial.printf("[finishing] M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", M1_pos, M4_pos, odhylaka, integral);
          }
        }
        man.motor(rb::MotorId::M1).power(-i);
        man.motor(rb::MotorId::M4).power(i );

        // Serial.printf("[DECEL] i: %d | M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", i, M1_pos, M4_pos, odhylaka, integral);

        delay(10);
    }
    int diff = abs(M1_pos - M4_pos);
    int time=0;
    int stop_time = 700; // časový limit pro dorovnání
    if ((M1_pos < M4_pos)) {
        // Serial.printf("[DOROVNÁNÍ] M1 dojíždí: %d ticků\n", diff);
        while ((M1_pos < (M4_pos-7)) && (time < stop_time)) {
            man.motor(rb::MotorId::M1).power(-3900);
            man.motor(rb::MotorId::M4).power(2500);
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
                M1_pos = -info.position();
            });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = info.position();
            });
            delay(5);
            time+=5;
            // Serial.printf("[time] %d\n", time);
        }
    } else if (M4_pos < M1_pos) {
        // Serial.printf("[DOROVNÁNÍ] M4 dojíždí: %d ticků\n", diff);
        while ((M4_pos < (M1_pos - 7)) && (time < stop_time)) {
            man.motor(rb::MotorId::M1).power(-2500);
            man.motor(rb::MotorId::M4).power(3900);
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = info.position();
            });
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
                M1_pos = -info.position();
            });
            // Serial.printf("[DOROVNÁNÍ] M1_pos: %d | M4_pos: %d | odchylka: %d\n", M1_pos, M4_pos, M1_pos - M4_pos);
            delay(5);
            time+=5;
            // Serial.printf("[time] %d\n", time);
        }
    }
    // Zastavit oba motory na konci
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    // Serial.println("[FORWARD] Dorovnání hotovo!");

    // Serial.println("[FORWARD] Hotovo!");
}
void encodery() {
  // Serial.printf("L: %f, R: %f\n", rkMotorsGetPositionLeft(), rkMotorsGetPositionRight());
}

void radius_r(int degrees, int polomer, int speed)
{
  float sR = speed * ((polomer) / (polomer + roztec));
  float sL = speed * 0.987;
  bool left_done =false;
  bool right_done = false;
  if(degrees > 0){
    float stupne = degrees * koeficient_r_f;
    rkMotorsDriveLeftAsync((((polomer + roztec) * PI * stupne) / 180) , sL, [&left_done]() {
        left_done = true;
    });
    rkMotorsDriveRightAsync((( polomer * PI * stupne) / 180) , sR, [&right_done]() {
        right_done = true;
    });
  }
  else{
  float stupne= degrees * koeficient_r_b;
    rkMotorsDriveLeftAsync((((polomer + roztec) * PI * stupne) / 180) , sL, [&left_done]() {
        left_done = true;
    });
    rkMotorsDriveRightAsync((( polomer * PI * stupne) / 180) , sR, [&right_done]() {
        right_done = true;
    });
  }
    while(!left_done || !right_done) {
        delay(10); // čekáme na dokončení obou motorů
    }
}
void radius_l(int degrees, int polomer, int speed){
  float sR = speed ;
  float sL = speed * ((polomer) / (polomer + roztec));
  bool left_done = false;
  bool right_done = false;
    if(degrees > 0){
        float stupne = degrees * koeficient_l_f;
        rkMotorsDriveLeftAsync((( polomer * PI * stupne) / 180) , sL, [&left_done]() {
            left_done = true;
        });
        rkMotorsDriveRightAsync((((polomer + roztec) * PI * stupne) / 180) , sR, [&right_done]() {
            right_done = true;
        });
    }
    else{
        float stupne = degrees * koeficient_l_b;
        rkMotorsDriveLeftAsync((( polomer * PI * stupne) / 180), sL, [&left_done]() {
            left_done = true;
        });
        rkMotorsDriveRightAsync((((polomer + roztec) * PI * stupne) / 180) , sR, [&right_done]() {
            right_done = true;
        });
    }
    while(!left_done || !right_done) {
        delay(10); // čekáme na dokončení obou motorů
    }
}
void turn_on_spot(int degrees){
  rkMotorsDrive(((degrees/360.0) * PI * roztec) * koeficient_turn , ((degrees/360.0) * -PI * roztec)* koeficient_turn, 30 , 30 );
    // Serial.printf("Otáčení na místě o %d stupňů\n", degrees);
    delay(500);
}
void back_buttons(int speed)
{
    int M1_pos = 0, M4_pos = 0, odchylka = 0, integral = 0, last_odchylka = 0;
    const float Kp = 55.0f;
    const float Ki = 0.01f;
    const float Kd = 0.1f;
    int target = (32000 * speed/100);
    auto& man = rb::Manager::get();
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    // Zrychlování
    for(int i = 0; i < target-1; i += a) {
        odchylka = M1_pos - M4_pos;
        integral += odchylka;
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        man.motor(rb::MotorId::M1).power(i * 0.94);
        man.motor(rb::MotorId::M4).power(-i + odchylka * Kp + integral * Ki + (odchylka - last_odchylka) * Kd);
        // Serial.printf("[ACCEL] i: %d | M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", i, M1_pos, M4_pos, odchylka, integral);
        delay(8);
    }
    while (true)
    {
        // Získání aktuálních pozic
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });

        odchylka = M1_pos - M4_pos;
        integral += odchylka;

        man.motor(rb::MotorId::M1).power(speed * 320);
        man.motor(rb::MotorId::M4).power(-speed * 320 + odchylka * Kp + integral * Ki + (odchylka - last_odchylka) * Kd);

        // Serial.printf("[BACK] M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", M1_pos, M4_pos, odchylka, integral);

        last_odchylka = odchylka;

        if ((digitalRead(Bbutton1) == LOW) && (digitalRead(Bbutton2) == LOW)) {
            break;
        }
        delay(50);
    }
    delay(150);
    // Serial.println("Obě Tlačítka STISKNUTY!");
    rkMotorsSetPower(0, 0);
}
//timeout