#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include "AppSettings.h"
#include "LedController.h"
#include "WifiConnector.h"
#include "ConfigPortal.h"
#include "ButtonHandler.h"
#include "BatteryMonitor.h"
#include "GpsReader.h"
#include "ImuReader.h"
#include "GpsPowerManager.h"
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
ImuReader imuReader(IMU_SDA_PIN, IMU_SCL_PIN);
GpsPowerManager gpsPowerManager(gpsReader, imuReader);
Tracker tracker(gpsReader, imuReader, appSettings, batteryMonitor, configPortal, ledController);

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

static void init_watchdog() {
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 30000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_reconfigure(&wdt_config);
    esp_task_wdt_add(NULL);
}

void setup() {
    delay(300);
    pinMode(GPS_SLEEP_PIN, OUTPUT);
    digitalWrite(GPS_SLEEP_PIN, GPS_WAKE_ACTIVE_LEVEL);

    Serial.begin(115200);
    delay(300);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_GPIO) {
        Serial.println("[Boot] wakeup by IMU motion interrupt");
    } else if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("[Boot] wakeup by timer");
    } else {
        Serial.println("[Boot] normal boot");
    }

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

    bool imuOk = false;
    for (uint8_t attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("[Main] IMU init attempt %d/3\n", attempt);
        if (imuReader.begin()) {
            imuReader.setThreshold(IMU_ACCEL_THRESHOLD);
#ifdef IMU_INT_PIN
            imuReader.enableMotionInterrupt(IMU_INT_PIN);
#endif
            imuOk = true;
            break;
        }
        delay(1000);
    }
    if (!imuOk) {
        Serial.println("[Main] IMU init failed, GPS power manager will keep GPS awake");
    }

    gpsPowerManager.begin();

    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi off at startup, Tracker will connect when needed");

    tracker.begin();
    init_watchdog();
}

void loop() {
    esp_task_wdt_reset();
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
    imuReader.update();
    gpsPowerManager.update();
    gpsReader.update();
    tracker.update();

    delay(10);
}
