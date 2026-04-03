#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

class TimezoneSync {
public:
    bool fetch();
    long getOffsetSec() const;
    bool isSynced() const;

private:
    long offsetSec = 0;
    bool synced = false;

    static constexpr const char* API_URL = "http://ip-api.com/json/?fields=offset";
};