#include "WifiConnector.h"

static const char* wifiStatusToString(wl_status_t status) {
    switch (status) {
        case WL_NO_SHIELD:       return "WL_NO_SHIELD";
        case WL_IDLE_STATUS:     return "WL_IDLE_STATUS";
        case WL_NO_SSID_AVAIL:   return "WL_NO_SSID_AVAIL";
        case WL_SCAN_COMPLETED:  return "WL_SCAN_COMPLETED";
        case WL_CONNECTED:       return "WL_CONNECTED";
        case WL_CONNECT_FAILED:  return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
        case WL_DISCONNECTED:    return "WL_DISCONNECTED";
        default:                 return "UNKNOWN";
    }
}

WifiConnector::WifiConnector(AppSettings& settings, LedController& led, ConfigPortal& portal)
    : settings(settings), led(led), portal(portal) {
}

void WifiConnector::storeCredentials() {
    uint8_t count = settings.getWifiCount();
    if (count == 0) {
        Serial.println("[WifiConnector] no credentials to store");
        return;
    }

    auto wifi = settings.getWifi(0);
    String ssid = wifi.ssid;
    ssid.trim();
    if (ssid.isEmpty()) return;

    WiFi.persistent(true);
    WiFi.setSleep(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), wifi.password.c_str());
    delay(100);
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_OFF);

    Serial.print("[WifiConnector] credentials stored for: ");
    Serial.println(ssid);
}

bool WifiConnector::connectToFirstAvailableSavedNetwork() {
    uint8_t count = settings.getWifiCount();

    if (count == 0) {
        Serial.println("No saved WiFi networks.");
        return false;
    }

    WiFi.persistent(false);
    WiFi.setSleep(false);
    WiFi.mode(WIFI_MODE_NULL);
    delay(300);
    WiFi.mode(WIFI_STA);
    delay(300);
    WiFi.disconnect(false, false);
    delay(300);

    for (uint8_t i = 0; i < count; i++) {
        auto wifi = settings.getWifi(i);
        String ssid = wifi.ssid;
        String password = wifi.password;
        ssid.trim();

        if (ssid.isEmpty()) continue;

        Serial.println();
        Serial.print("Trying WiFi #");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(ssid);
        Serial.print("Password length: ");
        Serial.println(password.length());

        WiFi.disconnect(false, false);
        delay(500);
        WiFi.begin(ssid.c_str(), password.c_str());

        unsigned long startedAt = millis();
        wl_status_t lastStatus = WL_IDLE_STATUS;

        Serial.print("begin() done, initial status: ");
        Serial.println(wifiStatusToString(WiFi.status()));

        while (millis() - startedAt < 15000) {
            wl_status_t status = WiFi.status();

            if (status != lastStatus) {
                Serial.print("WiFi status: ");
                Serial.print(wifiStatusToString(status));
                Serial.print(" at ms=");
                Serial.println(millis() - startedAt);
                lastStatus = status;
            }

            if (status == WL_CONNECTED) {
                Serial.println("WiFi connected.");
                Serial.print("IP: ");
                Serial.println(WiFi.localIP());
                return true;
            }

            if (status == WL_CONNECT_FAILED ||
                status == WL_NO_SSID_AVAIL  ||
                status == WL_CONNECTION_LOST) {
                Serial.println("Breaking due to terminal status");
                break;
            }

            delay(250);
        }

        Serial.print("Exited loop at ms=");
        Serial.println(millis() - startedAt);
        Serial.println("Failed. Trying next saved network...");
    }

    Serial.println("Could not connect to any saved WiFi.");
    return false;
}

void WifiConnector::update() {
    if (portal.isActive()) return;

    if (WiFi.getMode() == WIFI_OFF) return;

    if (WiFi.status() == WL_CONNECTED) {
        reconnecting = false;
        return;
    }

    unsigned long now = millis();

    if (!reconnecting) {
        if (now - lastReconnectAt < RECONNECT_INTERVAL_MS) return;
        startReconnect();
        return;
    }

    checkReconnect();
}

void WifiConnector::startReconnect() {
    uint8_t count = settings.getWifiCount();
    if (count == 0) return;

    reconnectIndex = 0;
    reconnecting = true;
    lastReconnectAt = millis();
    reconnectStartedAt = millis();

    auto wifi = settings.getWifi(reconnectIndex);
    String ssid = wifi.ssid;
    ssid.trim();
    if (ssid.isEmpty()) return;

    Serial.print("[WifiConnector] reconnecting to: ");
    Serial.println(ssid);

    WiFi.disconnect(false, false);
    WiFi.begin(ssid.c_str(), wifi.password.c_str());
}

void WifiConnector::checkReconnect() {
    wl_status_t status = WiFi.status();

    if (status == WL_CONNECTED) {
        Serial.println("[WifiConnector] reconnected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        reconnecting = false;
        return;
    }

    bool timedOut = millis() - reconnectStartedAt >= RECONNECT_TIMEOUT_MS;
    bool failed   = status == WL_CONNECT_FAILED  ||
                    status == WL_NO_SSID_AVAIL    ||
                    status == WL_CONNECTION_LOST;

    if (!timedOut && !failed) return;

    reconnectIndex++;
    uint8_t count = settings.getWifiCount();

    if (reconnectIndex >= count) {
        Serial.println("[WifiConnector] all networks failed, will retry in 30 sec");
        reconnecting = false;
        lastReconnectAt = millis();
        return;
    }

    auto wifi = settings.getWifi(reconnectIndex);
    String ssid = wifi.ssid;
    ssid.trim();
    if (ssid.isEmpty()) return;

    Serial.print("[WifiConnector] trying next: ");
    Serial.println(ssid);

    reconnectStartedAt = millis();
    WiFi.disconnect(false, false);
    WiFi.begin(ssid.c_str(), wifi.password.c_str());
}

void WifiConnector::markLastReconnectNow() {
    lastReconnectAt = millis();
}