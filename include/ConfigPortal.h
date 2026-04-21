#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include "AppSettings.h"

class ConfigPortal {
public:
    ConfigPortal(AppSettings& settings, uint16_t port = 80);

    void begin();
    void loop();
    void start();
    void stop();
    bool isActive() const;

private:
    AppSettings& settings;
    WebServer server;
    bool active = false;

    static constexpr const char* AP_SSID = "ESP Nav";
    static constexpr const char* AP_PASS = "12345678";

    String htmlEscape(const String& s);
    String buildRootPage();
    String buildScanPage();

    void handleRoot();
    void handleSaveServer();
    void handleAddWifi();
    void handleDeleteWifi();
    void handleScan();
    void handleNotFound();

    void setupRoutes();
};