#pragma once

#include <atomic>

#include "_librk_motors.h"

namespace rk {

class Context {
public:
    Context();
    ~Context();

    void setup(const rkConfig& cfg);

    rb::Protocol* prot() const { return m_prot; }
    Motors& motors() { return m_motors; }

    adc1_channel_t irChanLeft() const { return m_ir_left; }
    adc1_channel_t irChanRight() const { return m_ir_right; }

    uint16_t irRead(adc1_channel_t chan, uint16_t samples = 32);

    void stupidServoSet(uint8_t id, float positionDegrees);
    float stupidServoGet(uint8_t id);

private:

    void initIrSensors();

    Motors m_motors;
    rb::Protocol* m_prot;

    std::atomic<bool> m_initialized;

    std::atomic<bool> m_ir_installed;


    adc1_channel_t m_ir_left;
    adc1_channel_t m_ir_right;

    float m_stupid_servo_min;
    float m_stupid_servo_max;
};

extern Context gCtx;

};
