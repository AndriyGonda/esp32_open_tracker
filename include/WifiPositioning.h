#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

struct WifiPosition {
    double lat;
    double lng;
    float accuracy;
    bool valid;
};

class WifiPositioning {
public:
    WifiPosition locate();

private:
    static constexpr const char* API_URL = "https://beacondb.net/v1/geolocate";
    static constexpr uint8_t MAX_SCAN_RESULTS = 10;
    static constexpr uint32_t HTTP_TIMEOUT_MS = 5000;
};