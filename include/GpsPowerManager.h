#pragma once

#include <Arduino.h>
#include "GpsReader.h"
#include "ImuReader.h"
#include "TrackerConfig.h"

class GpsPowerManager {
public:
    GpsPowerManager(GpsReader& gps, ImuReader& imu);

    void begin();
    void update();
    bool isGpsAwake() const;

private:
    GpsReader& gps;
    ImuReader& imu;

    bool lastImuMoving = false;
    unsigned long imuStillSince = 0;
    unsigned long motionSince = 0;

    void handleWake();
    void handleSleep();
};
