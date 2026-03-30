#pragma once

#include <Arduino.h>
#include "TrackerConfig.h"

class BatteryMonitor {
public:
    BatteryMonitor(uint8_t pin, float dividerRatio, float calibrationFactor = 1.0f);

    void begin();
    void update();

    float getVoltage() const;
    bool isLow() const;
    bool isCritical() const;

private:
    uint8_t pin;
    float dividerRatio;
    float calibrationFactor;

    unsigned long lastPrintAt = 0;

    static constexpr unsigned long PRINT_INTERVAL_MS = 10000;
    static constexpr uint8_t UPDATE_SAMPLES = 8;
    static constexpr uint8_t READ_SAMPLES   = 4;
    float readVoltage(uint8_t samples) const;
};