#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "AppSettings.h"
#include "LedController.h"
#include "ConfigPortal.h"

class WifiConnector {
public:
    WifiConnector(AppSettings& settings, LedController& led, ConfigPortal& portal);

    bool connectToFirstAvailableSavedNetwork();
    void storeCredentials();
    void update();
    void markLastReconnectNow();

private:
    AppSettings& settings;
    LedController& led;
    ConfigPortal& portal;

    unsigned long lastReconnectAt = 0;
    unsigned long reconnectStartedAt = 0;
    uint8_t reconnectIndex = 0;
    bool reconnecting = false;

    static constexpr unsigned long RECONNECT_INTERVAL_MS = 30000;
    static constexpr unsigned long RECONNECT_TIMEOUT_MS  = 15000;

    void startReconnect();
    void checkReconnect();
};