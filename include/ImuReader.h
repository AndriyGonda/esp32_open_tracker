#pragma once
#include <Arduino.h>
#include <Wire.h>

#define BMI160_ADDR 0x68

class ImuReader {
public:
    ImuReader(uint8_t sdaPin, uint8_t sclPin);
    bool begin();
    void update();
    void enableMotionInterrupt(uint8_t intPin);

    bool isMoving() const { return _moving; }
    bool isReady() const { return _ok && _baselineReady; }
    float getAccelMag() const { return _accelMag; }
    void setThreshold(float t) { _threshold = t; }

private:
    uint8_t _sdaPin, _sclPin;
    bool    _ok = false;

    float   _threshold = 0.25f;
    float   _accelMag  = 0.0f;
    bool    _moving    = false;

    static constexpr uint8_t WIN = 6;
    float   _window[WIN] = {};
    uint8_t _winIdx      = 0;
    bool    _winFull     = false;

    float   _gx = 0, _gy = 0, _gz = 9.81f;
    bool    _baselineReady = false;
    uint8_t _baselineCount = 0;
    static constexpr uint8_t BASELINE_SAMPLES = 20;

    unsigned long _lastReadMs = 0;
    static constexpr unsigned long READ_INTERVAL_MS = 100;

    uint8_t       _zeroCount      = 0;
    uint8_t       _reinitCount    = 0;
    static constexpr uint8_t MAX_ZERO_READS   = 3;
    static constexpr uint8_t MAX_REINIT_COUNT = 5;

    unsigned long _lastRetryMs = 0;
    static constexpr unsigned long RETRY_INTERVAL_MS = 20000UL; // 20 sec

    bool reinit();
    bool writeReg(uint8_t reg, uint8_t val);
    bool readAccel(float& ax, float& ay, float& az);
};
