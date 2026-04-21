#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <esp_wifi.h>
#include "GpsReader.h"
#include "AppSettings.h"
#include "BatteryMonitor.h"
#include "ConfigPortal.h"
#include "LedController.h"
#include "TrackerConfig.h"
#include "TimezoneSync.h"

#if ENABLE_WIFI_POSITIONING
#include "WifiPositioning.h"
#endif

class Tracker {
public:
    Tracker(GpsReader& gps, AppSettings& settings, BatteryMonitor& battery, ConfigPortal& portal, LedController& led);

    void begin();
    void update();
    void forceReleaseWifi();

private:
    GpsReader&      gps;
    AppSettings&    settings;
    BatteryMonitor& battery;
    ConfigPortal&   portal;
    LedController&  led;
    TimezoneSync    timezoneSync;

    unsigned long lastSentAt  = 0;
    float         lastBearing = -1.0f;

    static constexpr unsigned long ONE_YEAR_SECONDS = 365UL * 24 * 3600;
    static constexpr double COORD_EPS = 0.000001;

    unsigned long lastValidUnixTime = 0;
    bool          timeSynced        = false;

    bool flushing  = false;
    File flushFile;

    bool          wifiManagedByUs    = false;
    unsigned long wifiOnAt           = 0;
    unsigned long lastWifiFailAt     = 0;
    bool          lastConnectSuccess = false;

    bool   hasLastKnownGood     = false;
    double lastKnownGoodLat     = 0.0;
    double lastKnownGoodLng     = 0.0;
    float  lastKnownGoodBearing = 0.0f;

#if ENABLE_WIFI_POSITIONING
    WifiPositioning wifiPositioning;
#endif

    float   _speedWindow[PARKING_WINDOW_SIZE] = {};
    uint8_t _speedWindowIdx  = 0;
    bool    _speedWindowFull = false;

    bool          _pinned          = false;
    double        _pinLat          = 0.0;
    double        _pinLng          = 0.0;
    unsigned long _stationarySince = 0;

    unsigned long _lastParkingUpdateMs = 0;
    unsigned long _lastIntervalMs      = TRACKER_INTERVAL_STATIC;

    void parkingFilterUpdate(float speed);
    bool parkingIsMoving() const;
    void parkingApply(double& lat, double& lng, float& speed, bool moving);

    unsigned long currentInterval(bool moving, float currentBearing) const;

    bool ensureWifi();
    void releaseWifi();
    void sendToServer(double lat, double lng, float speed, float bearing, float accel, bool invalid);
    void saveToBlackbox(double lat, double lng, float speed, float bearing, float accel, bool invalid);
    void startFlush();
    void flushNextLine();

    String buildUrl(double lat, double lng, float speed, float bearing,
                    unsigned long timestamp, float voltage,
                    float altitude, uint32_t satellites,
                    float hdop, uint32_t freeKb, float accel, bool invalid);

    float         bearingDiff(float a, float b) const;
    float         distanceTo(double lat1, double lng1, double lat2, double lng2) const;
    unsigned long getSafeTimestamp(bool invalid);
    void          syncTimeNTP();

    bool isZeroCoordinate(double lat, double lng) const;
    bool isHomeCoordinate(double lat, double lng) const;
    bool isPinCandidateCoordinate(double lat, double lng) const;
    void storeLastKnownGood(double lat, double lng, float bearing);
};
