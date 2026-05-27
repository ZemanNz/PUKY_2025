#include <stdarg.h>
#include <stdio.h>
#include <Adafruit_TCS34725.h>
#include "_librk_context.h"
#include "_librk_smart_servo.h"
#include "robotka.h"
#ifdef USE_VL53L0X
#include <Adafruit_VL53L0X.h>
#include <rk_laser.cpp>
#endif
#include "RBCX.h"
#include <Adafruit_TCS34725.h>
#define TAG "robotka"

using namespace rb;
using namespace rk;
// Empty loop in case the user won't supply one
void __attribute__((weak)) loop() {
}

void rkSetup(const rkConfig& cfg) {
    gCtx.setup(cfg);
}

void rkControllerSendLog(const char* format, ...) {
    if (gCtx.prot() == nullptr) {
        ESP_LOGE(TAG, "%s: protocol not initialized!", __func__);
        return;
    }
    va_list args;
    va_start(args, format);
    gCtx.prot()->send_log(format, args);
    va_end(args);
}

void rkControllerSendLog(const std::string& text) {
    if (gCtx.prot() == nullptr) {
        ESP_LOGE(TAG, "%s: protocol not initialized!", __func__);
        return;
    }

    gCtx.prot()->send_log(text);
}

void rkControllerSend(const char* cmd, rbjson::Object* data) {
    if (gCtx.prot() == nullptr) {
        ESP_LOGE(TAG, "%s: protocol not initialized!", __func__);
        return;
    }
    gCtx.prot()->send(cmd, data);
}

void rkControllerSendMustArrive(const char* cmd, rbjson::Object* data) {
    if (gCtx.prot() == nullptr) {
        ESP_LOGE(TAG, "%s: protocol not initialized!", __func__);
        return;
    }
    gCtx.prot()->send_mustarrive(cmd, data);
}

uint32_t rkBatteryPercent() {
    return Manager::get().battery().pct();
}

uint32_t rkBatteryVoltageMv() {
    return Manager::get().battery().voltageMv();
}

int16_t rkTemperature() {
    return Manager::get().battery().temperatureC();
}

void rkMotorsSetPower(int8_t left, int8_t right) {
    gCtx.motors().setPower(left, right);
}

void rkMotorsSetPowerLeft(int8_t power) {
    gCtx.motors().setPowerById(gCtx.motors().idLeft(), power);
}

void rkMotorsSetPowerRight(int8_t power) {
    gCtx.motors().setPowerById(gCtx.motors().idRight(), power);
}

void rkMotorsSetPowerById(uint8_t id, int8_t power) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }
    gCtx.motors().setPowerById(rb::MotorId(id), power);
}

void rkMotorsSetSpeed(int8_t left, int8_t right) {
    gCtx.motors().setSpeed(left, right);
}

void rkMotorsSetSpeedLeft(int8_t speed) {
    gCtx.motors().setSpeedById(gCtx.motors().idLeft(), speed);
}

void rkMotorsSetSpeedRight(int8_t speed) {
    gCtx.motors().setSpeedById(gCtx.motors().idRight(), speed);
}

void rkMotorsSetSpeedById(uint8_t id, int8_t speed) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }
    gCtx.motors().setSpeedById(rb::MotorId(id), speed);
}
void rkMotorsDrive(float mmLeft, float mmRight, float speed_left, float speed_right) {
    SemaphoreHandle_t binary = xSemaphoreCreateBinary();
    gCtx.motors()
        .drive(mmLeft, mmRight, speed_left, speed_right, [=]() {
            xSemaphoreGive(binary);
        });
    xSemaphoreTake(binary, portMAX_DELAY);
    vSemaphoreDelete(binary);
}

void rkMotorsDriveLeft(float mm, uint8_t speed) {
    rkMotorsDriveById(uint8_t(gCtx.motors().idLeft()) + 1, mm, speed);
}

void rkMotorsDriveRight(float mm, uint8_t speed) {
    rkMotorsDriveById(uint8_t(gCtx.motors().idRight()) + 1, mm, speed);
}

void rkMotorsDriveById(uint8_t id, float mm, uint8_t speed) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }

    SemaphoreHandle_t binary = xSemaphoreCreateBinary();
    gCtx.motors()
        .driveById(rb::MotorId(id), mm, speed, [=]() {
            xSemaphoreGive(binary);
        });
    xSemaphoreTake(binary, portMAX_DELAY);
    vSemaphoreDelete(binary);
}

void rkMotorsDriveAsync(float mmLeft, float mmRight, uint8_t speed_left, uint8_t speed_right, std::function<void()> callback) {
    gCtx.motors().drive(mmLeft, mmRight, speed_left, speed_right, std::move(callback));
}

void rkMotorsDriveLeftAsync(float mm, uint8_t speed, std::function<void()> callback) {
    gCtx.motors().driveById(gCtx.motors().idLeft(), mm, speed, std::move(callback));
}

void rkMotorsDriveRightAsync(float mm, uint8_t speed, std::function<void()> callback) {
    gCtx.motors().driveById(gCtx.motors().idRight(), mm, speed, std::move(callback));
}

void rkMotorsDriveByIdAsync(uint8_t id, float mm, uint8_t speed, std::function<void()> callback) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }
    gCtx.motors().driveById(rb::MotorId(id), mm, speed, std::move(callback));
}

float rkMotorsGetPositionLeft(bool fetch) {
    return rkMotorsGetPositionById((uint8_t)gCtx.motors().idLeft() + 1, fetch);
}

float rkMotorsGetPositionRight(bool fetch) {
    return rkMotorsGetPositionById((uint8_t)gCtx.motors().idRight() + 1, fetch);
}

float rkMotorsGetPositionById(uint8_t id, bool fetch) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return 0.f;
    }

    if(fetch) {
        SemaphoreHandle_t binary = xSemaphoreCreateBinary();
        auto& m = rb::Manager::get().motor(rb::MotorId(id));
        m.requestInfo([=](rb::Motor& m) {
            xSemaphoreGive(binary);
        });
        xSemaphoreTake(binary, portMAX_DELAY);
        vSemaphoreDelete(binary);
    }

    return gCtx.motors().position(rb::MotorId(id));
}

void rkMotorsSetPositionLeft(float positionMm) {
    rkMotorsSetPositionById((uint8_t)gCtx.motors().idLeft() + 1, positionMm);
}

void rkMotorsSetPositionRight(float positionMm) {
    rkMotorsSetPositionById((uint8_t)gCtx.motors().idRight() + 1, positionMm);
}

void rkMotorsSetPositionById(uint8_t id, float positionMm) {
    id -= 1;
    if (id >= (int)rb::MotorId::MAX) {
        ESP_LOGE(TAG, "%s: invalid motor id, %d is out of range <1;4>.", __func__, id + 1);
        return;
    }

    gCtx.motors().setPosition(rb::MotorId(id), positionMm);
}

void rkMotorsJoystick(int32_t x, int32_t y) {
    gCtx.motors().joystick(x, y);
}

void rkLedRed(bool on) {
    Manager::get().leds().red(on);
}

void rkLedYellow(bool on) {
    Manager::get().leds().yellow(on);
}

void rkLedGreen(bool on) {
    Manager::get().leds().green(on);
}

void rkLedBlue(bool on) {
    Manager::get().leds().blue(on);
}

void rkLedAll(bool on) {
    auto& l = Manager::get().leds();
    l.red(on);
    l.yellow(on);
    l.green(on);
    l.blue(on);
}

void rkLedById(uint8_t id, bool on) {
    if (id == 0) {
        ESP_LOGE(TAG, "%s: invalid id %d, LEDs are indexed from 1, just like on the board (LED1, LED2...)!", __func__, id);
        return;
    } else if (id > 4) {
        ESP_LOGE(TAG, "%s: maximum LED id is 4, you are using %d!", __func__, id);
        return;
    }

    auto& l = Manager::get().leds();
    l.byId(LedId((1 << (id - 1))), on);
}

bool rkButtonIsPressed(rkButtonId id, bool waitForRelease) {
    auto& b = Manager::get().buttons();
    for (int i = 0; i < 3; ++i) {
        const bool pressed = b.byId(ButtonId(id));
        if (!pressed)
            return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (waitForRelease) {
        rkButtonWaitForRelease(id);
    }
    return true;
}

void rkButtonOnChangeAsync(std::function<bool(rkButtonId, bool)> callback) {
    Manager::get().buttons().onChange([callback](rb::ButtonId id, bool pressed) -> bool {
        return callback(rkButtonId(id), pressed);
    });
}

void rkButtonWaitForRelease(rkButtonId id) {
    int counter = 0;
    auto& b = Manager::get().buttons();
    while (true) {
        const bool pressed = b.byId(ButtonId(id));
        if (!pressed) {
            if (++counter > 3)
                return;
        } else {
            counter = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


uint16_t rkIrLeft() {
    return gCtx.irRead(gCtx.irChanLeft());
}

uint16_t rkIrRight() {
    return gCtx.irRead(gCtx.irChanRight());
}

uint32_t rkUltraMeasure(uint8_t id) {
    if (id == 0) {
        ESP_LOGE(TAG, "%s: invalid id %d, Ultrasounds are indexed from 1, just like on the board (U1, U2...)!", __func__, id);
        return 0;
    } else if (id > 4) {
        ESP_LOGE(TAG, "%s: maximum Ultrasound id is 4, you are using %d!", __func__, id);
        return 0;
    }

    return Manager::get().ultrasound(id - 1).measure();
}

void rkUltraMeasureAsync(uint8_t id, std::function<bool(uint32_t)> callback) {
    if (id == 0) {
        ESP_LOGE(TAG, "%s: invalid id %d, Ultrasounds are indexed from 1, just like on the board (U1, U2...)!", __func__, id);
        callback(0);
        return;
    } else if (id > 4) {
        ESP_LOGE(TAG, "%s: maximum Ultrasound id is 4, you are using %d!", __func__, id);
        callback(0);
        return;
    }

    Manager::get().ultrasound(id - 1).measureAsync(std::move(callback));
}

void rkBuzzerSet(bool on) {
    Manager::get().piezo().setState(on);
}

void rk_laser_init(const char* name,TwoWire& bus,  Adafruit_VL53L0X& lox, uint8_t   xshut_pin,uint8_t    new_address){

    return rk_laser_init_basic(name,  bus, lox, xshut_pin, new_address);
}

int rk_laser_measure(const char* name){
    return rk_laser_measure_basic(name);
}

#define MAX_COLOR_SENSORS 2

struct ColorSensor {
  const char*           name;
  Adafruit_TCS34725*    tcs;
  TwoWire*              bus;
};
static ColorSensor colorSensors[MAX_COLOR_SENSORS];
static uint8_t    colorCount = 0;

/**
 * @brief Inicializuje TCS34725 barevný senzor na zadané I2C sběrnici.
 *
 * Funkce ukládá instanci senzoru do interního pole podle jména a spouští
 * komunikaci s modulem. Každý senzor je identifikován unikátním jménem.
 *
 * @param name    Textový identifikátor senzoru (např. "front" nebo "down").
 * @param bus     Referenční I2C sběrnice (Wire nebo Wire1) pro komunikaci.
 * @param tcs     Reference na instanci Adafruit_TCS34725 pro daný senzor.
 * @return true   Pokud se senzor úspěšně inicializoval.
 * @return false  Pokud se nepodařilo spojení s modulem.
 */
bool rkColorSensorInit(const char* name, TwoWire& bus, Adafruit_TCS34725& tcs)
{
  if (colorCount >= MAX_COLOR_SENSORS) {
    Serial.println("ERROR: Max pocet color sensoru prekrocen");
    return false;
  }

  // nastav I2C sběrnici
  tcs.begin(TCS34725_ADDRESS, &bus);
  if (!tcs.begin(TCS34725_ADDRESS, &bus)) {
    Serial.print("ERROR: Nelze pripojit k color senzoru ");
    Serial.println(name);
    return false;
  }

  // ulož konfiguraci
  ColorSensor& s = colorSensors[colorCount++];
  s.name = name;
  s.tcs  = &tcs;
  s.bus  = &bus;

  return true;
}

/**
 * @brief Načte hodnoty RGB z barevného senzoru podle jeho identifikátoru.
 *
 * Funkce najde senzor v interním seznamu podle jména a zavolá getRGB, 
 * přičte kompanzní faktor a vrátí normalizované hodnoty v rozsahu 0.0–1.0.
 *
 * @param name  Textový identifikátor senzoru (stejný jako při init).
 * @param r     Ukazatel na float pro červenou složku (0.0–1.0).
 * @param g     Ukazatel na float pro zelenou složku (0.0–1.0).
 * @param b     Ukazatel na float pro modrou složku (0.0–1.0).
 * @return true   Pokud se měření úspěšně provedlo.
 * @return false  Pokud senzor není nalezen nebo měření selhalo.
 */
bool rkColorSensorGetRGB(const char* name, float* r, float* g, float* b)
{
    for (uint8_t i = 0; i < colorCount; i++) {
        ColorSensor& s = colorSensors[i];
        if (strcmp(s.name, name) == 0) {
            // Načti RGB hodnoty (0-255) přímo do poskytnutých proměnných
            s.tcs->getRGB(r, g, b);
            return true;
        }
    }
    return false;
}

void rkServosSetPosition(uint8_t id, float angleDegrees) {
    id -= 1;
    if (id >= rb::StupidServosCount) {
        ESP_LOGE(TAG, "%s: invalid id %d, must be <= %d!", __func__, id, rb::StupidServosCount);
        return;
    }
    gCtx.stupidServoSet(id, angleDegrees);
}

float rkServosGetPosition(uint8_t id) {
    id -= 1;
    if (id >= rb::StupidServosCount) {
        ESP_LOGE(TAG, "%s: invalid id %d, must be <= %d!", __func__, id, rb::StupidServosCount);
        return NAN;
    }
    return gCtx.stupidServoGet(id);
}

void rkServosDisable(uint8_t id) {
    id -= 1;
    if (id >= rb::StupidServosCount) {
        ESP_LOGE(TAG, "%s: invalid id %d, must be <= %d!", __func__, id, rb::StupidServosCount);
        return;
    }
    rb::Manager::get().stupidServo(id).disable();
}

lx16a::SmartServoBus& rkSmartServoBus(uint8_t servo_count) {
    static rk::SmartServoBusInitializer init(servo_count);
    return init.bus();
}
