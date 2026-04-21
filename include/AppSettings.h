#pragma once

#include <Arduino.h>
#include <Preferences.h>

class AppSettings {
public:
    static const uint8_t MAX_WIFI = 10;

    struct WifiEntry {
        String ssid;
        String password;
    };

    void load();
    void save();

    uint8_t getWifiCount() const;
    const WifiEntry& getWifi(uint8_t index) const;

    bool addOrUpdateWifi(const String& ssid, const String& password);
    bool removeWifi(uint8_t index);

    const String& getServerHost() const;
    uint16_t getServerPort() const;

    void setServerHost(const String& host);
    void setServerPort(uint16_t port);

private:
    Preferences preferences;
    WifiEntry wifiList[MAX_WIFI];
    uint8_t wifiCount = 0;

    String serverHost = "hw.m2m.eu"; // support Traccar OsmAnd protocol or Traccar instance
    uint16_t serverPort = 5055;
};