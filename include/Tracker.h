#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <HTTPClient.h>

#include "GpsReader.h"
#include "ImuReader.h"
#include "AppSettings.h"
#include "BatteryMonitor.h"
#include "ConfigPortal.h"
#include "LedController.h"
#include "TimezoneSync.h"
#include "TrackerConfig.h"

class Tracker {
public:
    Tracker(GpsReader& gps,
            ImuReader& imu,
            AppSettings& settings,
            BatteryMonitor& battery,
            ConfigPortal& portal,
            LedController& led);

    void begin();
    void update();
    void forceReleaseWifi();

private:
    GpsReader& gps;
    ImuReader& imu;
    AppSettings& settings;
    BatteryMonitor& battery;
    ConfigPortal& portal;
    LedController& led;
    TimezoneSync timezoneSync;

    bool wifiManagedByUs = false;
    bool lastConnectSuccess = false;
    bool timeSynced = false;
    bool flushing = false;
    bool hasLastKnownGood = false;
    bool _pinned = false;

    unsigned long wifiOnAt = 0;
    unsigned long lastWifiFailAt = 0;
    unsigned long lastSentAt = 0;
    unsigned long lastValidUnixTime = 0;
    unsigned long _stationarySince = 0;
    unsigned long _lastParkingUpdateMs = 0;
    unsigned long _lastIntervalMs = 0;

    float lastBearing = -1.0f;
    double lastKnownGoodLat = 0.0;
    double lastKnownGoodLng = 0.0;
    float lastKnownGoodBearing = 0.0f;
    double _pinLat = 0.0;
    double _pinLng = 0.0;

    static constexpr double COORD_EPS = 0.000001;
    static constexpr unsigned long ONE_YEAR_SECONDS = 365UL * 24UL * 60UL * 60UL;

    float _speedWindow[PARKING_WINDOW_SIZE] = {0};
    uint8_t _speedWindowIdx = 0;
    bool _speedWindowFull = false;

    File flushFile;

    bool isZeroCoordinate(double lat, double lng) const;
    bool isHomeCoordinate(double lat, double lng) const;
    bool isPinCandidateCoordinate(double lat, double lng) const;
    void storeLastKnownGood(double lat, double lng, float bearing);

    bool ensureWifi();
    void releaseWifi();

    unsigned long currentInterval(bool moving, float currentBearing) const;

    String buildUrl(double lat, double lng, float speed, float bearing,
                    unsigned long timestamp, float voltage,
                    float altitude, uint32_t satellites,
                    float hdop, uint32_t freeKb, float accel, bool invalid);

    void sendToServer(double lat, double lng, float speed, float bearing,
                      float accel, bool invalid);
    void saveToBlackbox(double lat, double lng, float speed, float bearing,
                        float accel, bool invalid);

    void startFlush();
    void flushNextLine();

    void parkingFilterUpdate(float speed);
    bool parkingIsMoving() const;
    void parkingApply(double& lat, double& lng, float& speed, bool moving);

    float bearingDiff(float a, float b) const;
    float distanceTo(double lat1, double lng1, double lat2, double lng2) const;

    void syncTimeNTP();
    unsigned long getSafeTimestamp(bool invalid);
};
