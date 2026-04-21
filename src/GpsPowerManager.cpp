#include "GpsPowerManager.h"

GpsPowerManager::GpsPowerManager(GpsReader& gps, ImuReader& imu)
    : gps(gps), imu(imu) {
}

void GpsPowerManager::begin() {
    lastImuMoving = false;
    imuStillSince = millis();
    motionSince = 0;
}

bool GpsPowerManager::isGpsAwake() const {
    return !gps.isSleeping();
}

void GpsPowerManager::handleWake() {
    if (!gps.isSleeping()) return;

    Serial.println("[GpsPower] wake by IMU");
    gps.wake();
    motionSince = millis();
}

void GpsPowerManager::handleSleep() {
    if (gps.isSleeping()) return;
    if (millis() - gps.getWakeStartedAt() < GPS_MIN_AWAKE_AFTER_WAKE_MS) return;
    if (imuStillSince == 0) return;
    if (millis() - imuStillSince < GPS_SLEEP_AFTER_IMU_STILL_MS) return;

    Serial.println("[GpsPower] sleep by IMU inactivity");
    gps.sleep();
}

void GpsPowerManager::update() {
    const bool imuReady = imu.isReady();
    const bool imuMoving = imuReady && imu.isMoving();

    if (!imuReady) {
        if (gps.isSleeping()) {
            Serial.println("[GpsPower] IMU not ready -> keep GPS awake");
            gps.wake();
        }
        lastImuMoving = false;
        imuStillSince = millis();
        return;
    }

    if (imuMoving) {
        if (!lastImuMoving) {
            Serial.println("[GpsPower] IMU motion edge");
        }
        imuStillSince = 0;
        handleWake();
    } else {
        if (lastImuMoving) {
            imuStillSince = millis();
            Serial.println("[GpsPower] IMU became still");
        } else if (imuStillSince == 0) {
            imuStillSince = millis();
        }
        handleSleep();
    }

    lastImuMoving = imuMoving;
}
