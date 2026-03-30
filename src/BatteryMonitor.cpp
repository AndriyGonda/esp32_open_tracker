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

    float adcMv = sumMv / (float)samples;
    float batteryVoltage = (adcMv / 1000.0f) * dividerRatio * calibrationFactor;

    return batteryVoltage;
}

float BatteryMonitor::getVoltage() {
    float raw = readVoltage(READ_SAMPLES);

    voltageBuffer[voltageIndex] = raw;
    voltageIndex = (voltageIndex + 1) % VOLTAGE_SAMPLES;

    if (voltageIndex == 0) {
        voltageBufferFilled = true;
    }

    uint8_t count = voltageBufferFilled ? VOLTAGE_SAMPLES : voltageIndex;
    if (count == 0) return raw;

    float sum = 0;
    for (uint8_t i = 0; i < count; i++) {
        sum += voltageBuffer[i];
    }
    return sum / count;
}

bool BatteryMonitor::isLow() {
    return getVoltage() <= BATTERY_LOW_VOLTAGE;
}

bool BatteryMonitor::isCritical() {
    return getVoltage() <= BATTERY_CRITICAL_VOLTAGE;
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

    if (voltage <= BATTERY_CRITICAL_VOLTAGE) {
        Serial.println("[BatteryMonitor] CRITICAL BATTERY");
    } else if (voltage <= BATTERY_LOW_VOLTAGE) {
        Serial.println("[BatteryMonitor] LOW BATTERY");
    }
}