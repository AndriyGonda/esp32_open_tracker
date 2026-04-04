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

#if ENABLE_IMU
#include "ImuReader.h"
#endif

#define FUSION_OR   0
#define FUSION_AND  1
#define FUSION_IMU  2

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

    unsigned long lastValidUnixTime = 0;
    bool          timeSynced        = false;

    bool flushing  = false;
    File flushFile;

    bool          wifiManagedByUs    = false;
    unsigned long wifiOnAt           = 0;
    unsigned long lastWifiFailAt     = 0;
    bool          lastConnectSuccess = false;

#if ENABLE_WIFI_POSITIONING
    WifiPositioning wifiPositioning;
#endif

#if ENABLE_IMU
    ImuReader imu{IMU_SDA_PIN, IMU_SCL_PIN};
    bool imuMoving() const;
#endif

    bool ensureWifi();
    void releaseWifi();

    // --- Parking filter ---
    float   _speedWindow[PARKING_WINDOW_SIZE] = {};
    uint8_t _speedWindowIdx  = 0;
    bool    _speedWindowFull = false;

    // Position-pinning state
    bool          _pinned          = false;
    double        _pinLat          = 0.0;
    double        _pinLng          = 0.0;
    unsigned long _stationarySince = 0;

    // Acceleration check state
    unsigned long _lastParkingUpdateMs = 0;
    unsigned long _lastIntervalMs      = TRACKER_INTERVAL_STATIC;

    void parkingFilterUpdate(float speed);
    bool parkingIsMoving() const;
    void parkingApply(double& lat, double& lng, float& speed);

    unsigned long currentInterval();
    bool shouldSend();

    void sendToServer(double lat, double lng, float speed, float bearing,
                      float accel, bool invalid);
    void saveToBlackbox(double lat, double lng, float speed, float bearing,
                        float accel, bool invalid);
    void startFlush();
    void flushNextLine();

    String buildUrl(double lat, double lng, float speed, float bearing,
                    unsigned long timestamp, float voltage,
                    float altitude, uint32_t satellites,
                    float hdop, uint32_t freeKb, float accel);

    float         bearingDiff(float a, float b);
    float         distanceTo(double lat1, double lng1, double lat2, double lng2);
    unsigned long getSafeTimestamp(bool invalid);
    void          syncTimeNTP();
};