#include <Arduino.h>
#include "AppSettings.h"
#include "LedController.h"
#include "WifiConnector.h"
#include "ConfigPortal.h"
#include "ButtonHandler.h"
#include "BatteryMonitor.h"
#include "GpsReader.h"
#include "Tracker.h"
#include "PowerManager.h"

#define BUTTON_PIN 10
#define LONG_PRESS_MS 3000

#ifndef LED_BUILTIN
#define LED_BUILTIN 8
#endif

AppSettings appSettings;
LedController ledController(LED_BUILTIN);
ConfigPortal configPortal(appSettings);
WifiConnector wifiConnector(appSettings, ledController, configPortal);
ButtonHandler buttonHandler(BUTTON_PIN, LONG_PRESS_MS);

BatteryMonitor batteryMonitor(1, 2.0f);
PowerManager powerManager(batteryMonitor);

GpsReader gpsReader(3, 4);
Tracker tracker(gpsReader, appSettings, batteryMonitor, configPortal, ledController);

static void printLoadedSettings() {
    Serial.println();
    Serial.println("=== Loaded settings ===");
    Serial.print("Server host: ");
    Serial.println(appSettings.getServerHost());
    Serial.print("Server port: ");
    Serial.println(appSettings.getServerPort());
    Serial.print("Saved WiFi count: ");
    Serial.println(appSettings.getWifiCount());

    for (uint8_t i = 0; i < appSettings.getWifiCount(); i++) {
        const auto& wifi = appSettings.getWifi(i);
        Serial.print("Saved SSID #");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(wifi.ssid);
        Serial.print("Saved password length: ");
        Serial.println(wifi.password.length());
    }

    Serial.println("=======================");
    Serial.println();
}

void setup() {
    delay(300);
    Serial.begin(115200);
    delay(300);
    btStop();
    setCpuFrequencyMhz(80);

    ledController.begin();
    buttonHandler.begin();
    configPortal.begin();

    appSettings.load();
    printLoadedSettings();

    batteryMonitor.begin();
    powerManager.begin();
    gpsReader.begin();

    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi off at startup, Tracker will connect when needed");

    tracker.begin();
}

void loop() {
    powerManager.update();

    if (!powerManager.isOperational()) {
        delay(10);
        return;
    }

    if (buttonHandler.isLongPressTriggered()) {
        if (!configPortal.isActive()) {
            Serial.println("Long press detected. Opening config portal...");
            tracker.forceReleaseWifi(); 
            configPortal.start();
        }
    }

    ledController.update();
    wifiConnector.update();
    configPortal.loop();
    batteryMonitor.update();
    gpsReader.update();
    tracker.update();

    delay(10);
}