#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include "GpsReader.h"
#include "AppSettings.h"
#include "BatteryMonitor.h"
#include "ConfigPortal.h"
#include "LedController.h"
#include "TrackerConfig.h"

class Tracker {
public:
    Tracker(GpsReader& gps, AppSettings& settings, BatteryMonitor& battery, ConfigPortal& portal, LedController& led);

    void begin();
    void update();

private:
    GpsReader& gps;
    AppSettings& settings;
    BatteryMonitor& battery;
    ConfigPortal& portal;
    LedController& led;

    unsigned long lastSentAt = 0;
    float lastBearing = -1.0f;

    double latAccum = 0.0;
    double lngAccum = 0.0;
    uint8_t accumCount = 0;
    unsigned long lastAccumAt = 0;
    static constexpr unsigned long ACCUM_INTERVAL_MS = 10000;
    static constexpr unsigned long ONE_YEAR_SECONDS = 365UL * 24 * 3600;

    unsigned long lastValidUnixTime = 0;
    uint64_t lastValidMicros = 0;
    bool timeSynced = false;

    bool flushing = false;
    File flushFile;

    unsigned long currentInterval();
    bool shouldSend();

    void sendToServer(double lat, double lng, float speed, float bearing, bool invalid);
    void saveToBlackbox(double lat, double lng, float speed, float bearing, bool invalid);
    void startFlush();
    void flushNextLine();

    String buildUrl(double lat, double lng, float speed, float bearing,
                    unsigned long timestamp, float voltage,
                    float altitude, uint32_t satellites,
                    float hdop, uint32_t freeKb);

    float bearingDiff(float a, float b);
    float distanceTo(double lat1, double lng1, double lat2, double lng2);
    unsigned long getSafeTimestamp(bool invalid);
    void syncTimeNTP();
};