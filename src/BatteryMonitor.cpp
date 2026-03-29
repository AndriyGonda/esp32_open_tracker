#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor(uint8_t pin, float dividerRatio, float calibrationFactor)
    : pin(pin),
      dividerRatio(dividerRatio),
      calibrationFactor(calibrationFactor) {
}

void BatteryMonitor::begin() {
    analogReadResolution(12);
    analogSetPinAttenuation(pin, ADC_11db);
    pinMode(pin, INPUT);
}

float BatteryMonitor::readVoltage(uint8_t samples) const {
    uint32_t sumMv = 0;

    for (uint8_t i = 0; i < samples; i++) {
        sumMv += analogReadMilliVolts(pin);
        delay(2);
    }

    float adcMv = sumMv / (float) samples;
    float batteryVoltage = (adcMv / 1000.0f) * dividerRatio * calibrationFactor;

    return batteryVoltage;
}

float BatteryMonitor::getVoltage() const {
    return readVoltage(READ_SAMPLES);
}

bool BatteryMonitor::isLow() const {
    return getVoltage() <= LOW_VOLTAGE;
}

bool BatteryMonitor::isCritical() const {
    return getVoltage() <= CRITICAL_VOLTAGE;
}

void BatteryMonitor::update() {
    unsigned long now = millis();

    if (now - lastPrintAt < PRINT_INTERVAL_MS) {
        return;
    }

    lastPrintAt = now;

    float voltage = readVoltage(UPDATE_SAMPLES);

    Serial.print("[BatteryMonitor] voltage=");
    Serial.print(voltage, 3);
    Serial.println(" V");

    if (voltage <= CRITICAL_VOLTAGE) {
        Serial.println("[BatteryMonitor] CRITICAL BATTERY");
    } else if (voltage <= LOW_VOLTAGE) {
        Serial.println("[BatteryMonitor] LOW BATTERY");
    }
}