#include "PowerManager.h"

PowerManager::PowerManager(BatteryMonitor& battery)
    : battery(battery) {
}

void PowerManager::begin() {
    ledcAttach(POWER_BUZZER_PIN, POWER_BUZZER_FREQ, 8);
    ledcWrite(POWER_BUZZER_PIN, 0);
    Serial.println("[PowerManager] started");
}

void PowerManager::update() {
    unsigned long now = millis();

    if (now - lastVoltageCheckAt >= VOLTAGE_CHECK_MS) {
        lastVoltageCheckAt = now;
        voltage = battery.getVoltage();

        Serial.print("[PowerManager] voltage=");
        Serial.println(voltage, 2);
    }

    if (voltage < 0.5f) {
        return;
    }

    if (isCriticalBattery()) {
        goToDeepSleep();
        return;
    }

    if (isLowBattery()) {
        if (now - lastBeepAt >= POWER_BEEP_INTERVAL_MS) {
            lastBeepAt = now;
            beep(100);
            delay(100);
            beep(100);
            delay(100);
            beep(100);
        }
    }
}

bool PowerManager::isLowBattery() const {
    return battery.isLow();
}

bool PowerManager::isCriticalBattery() const {
    return battery.isCritical();
}

bool PowerManager::isOperational() const {
    return voltage < 0.5f || !battery.isCritical();
}

void PowerManager::beep(uint16_t durationMs) {
    ledcWrite(POWER_BUZZER_PIN, 64);
    delay(durationMs);
    ledcWrite(POWER_BUZZER_PIN, 0);
}

void PowerManager::goToDeepSleep() {
    Serial.println("[PowerManager] critical battery! shutting down...");

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    ledcWrite(POWER_BUZZER_PIN, 0);

    delay(100);

    Serial.println("[PowerManager] going to deep sleep for 1 minute");
    esp_sleep_enable_timer_wakeup(60ULL * 1000000ULL);
    esp_deep_sleep_start();
}