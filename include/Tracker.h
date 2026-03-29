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

    bool flushing = false;
    File flushFile;

    unsigned long currentInterval();
    bool shouldSend();

    void sendToServer(double lat, double lng, float speed, float bearing, bool invalid);
    void saveToBlackbox(double lat, double lng, float speed, float bearing, bool invalid);
    void startFlush();
    void flushNextLine();

    String buildUrl(double lat, double lng, float speed, float bearing, unsigned long timestamp);
    float bearingDiff(float a, float b);
    float distanceTo(double lat1, double lng1, double lat2, double lng2);
};