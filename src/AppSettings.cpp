#include "AppSettings.h"

namespace {
    bool keyExists(Preferences& preferences, const String& key) {
        return preferences.isKey(key.c_str());
    }
}

void AppSettings::load() {
    preferences.begin("esp-nav", true);

    wifiCount = preferences.getUChar("count", 0);
    if (wifiCount > MAX_WIFI) {
        wifiCount = MAX_WIFI;
    }

    for (uint8_t i = 0; i < MAX_WIFI; i++) {
        wifiList[i].ssid = "";
        wifiList[i].password = "";
    }

    for (uint8_t i = 0; i < wifiCount; i++) {
        wifiList[i].ssid = preferences.getString(("s" + String(i)).c_str(), "");
        wifiList[i].password = preferences.getString(("p" + String(i)).c_str(), "");
    }

    serverHost = preferences.getString("host", "hw.m2m.eu");
    serverPort = preferences.getUShort("port", 5055);

    preferences.end();
}

void AppSettings::save() {
    preferences.begin("esp-nav", false);

    preferences.putUChar("count", wifiCount);

    for (uint8_t i = 0; i < wifiCount; i++) {
        preferences.putString(("s" + String(i)).c_str(), wifiList[i].ssid);
        preferences.putString(("p" + String(i)).c_str(), wifiList[i].password);
    }

    for (uint8_t i = wifiCount; i < MAX_WIFI; i++) {
        String ssidKey = "s" + String(i);
        String passKey = "p" + String(i);

        if (keyExists(preferences, ssidKey)) {
            preferences.remove(ssidKey.c_str());
        }

        if (keyExists(preferences, passKey)) {
            preferences.remove(passKey.c_str());
        }
    }

    preferences.putString("host", serverHost);
    preferences.putUShort("port", serverPort);

    preferences.end();
}

uint8_t AppSettings::getWifiCount() const {
    return wifiCount;
}

const AppSettings::WifiEntry& AppSettings::getWifi(uint8_t index) const {
    return wifiList[index];
}

bool AppSettings::addOrUpdateWifi(const String& rawSsid, const String& rawPassword) {
    String ssid = rawSsid;
    String password = rawPassword;

    ssid.trim();
    if (ssid.isEmpty()) {
        return false;
    }

    for (uint8_t i = 0; i < wifiCount; i++) {
        if (wifiList[i].ssid == ssid) {
            wifiList[i].password = password;
            return true;
        }
    }

    if (wifiCount >= MAX_WIFI) {
        return false;
    }

    wifiList[wifiCount].ssid = ssid;
    wifiList[wifiCount].password = password;
    wifiCount++;

    return true;
}


bool AppSettings::removeWifi(uint8_t index) {
    if (index >= wifiCount) {
        return false;
    }

    for (uint8_t i = index; i + 1 < wifiCount; i++) {
        wifiList[i] = wifiList[i + 1];
    }

    wifiCount--;

    if (wifiCount < MAX_WIFI) {
        wifiList[wifiCount].ssid = "";
        wifiList[wifiCount].password = "";
    }

    return true;
}

const String& AppSettings::getServerHost() const {
    return serverHost;
}

uint16_t AppSettings::getServerPort() const {
    return serverPort;
}

void AppSettings::setServerHost(const String& host) {
    serverHost = host;
}

void AppSettings::setServerPort(uint16_t port) {
    serverPort = port;
}