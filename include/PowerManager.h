#pragma once

#include <Arduino.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include "BatteryMonitor.h"

#define POWER_LOW_BATT_V       3.2f   
#define POWER_CRITICAL_BATT_V  2.9f   
#define POWER_BUZZER_PIN       5
#define POWER_BUZZER_FREQ      2800
#define POWER_BEEP_INTERVAL_MS 30000 

class PowerManager {
public:
    explicit PowerManager(BatteryMonitor& battery);

    void begin();
    void update();

    bool isLowBattery() const;
    bool isCriticalBattery() const;
    bool isOperational() const;

private:
    BatteryMonitor& battery;
    float voltage = 0.0f;

    unsigned long lastBeepAt = 0;
    unsigned long lastVoltageCheckAt = 0;

    static constexpr unsigned long VOLTAGE_CHECK_MS = 5000;

    void beep(uint16_t durationMs);
    void goToDeepSleep();
};