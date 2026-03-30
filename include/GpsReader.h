#pragma once

#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

class GpsReader {
public:
    GpsReader(uint8_t rxPin, uint8_t txPin, uint32_t baud = 9600);

    void begin();
    void update();

    bool hasLocation() const;
    bool hasTime() const;

    double getLatitude();
    double getLongitude();
    double getAltitude();
    float getSpeed();
    float getBearing();
    double getHdop();
    uint32_t getSatellites();
    uint32_t getDate();
    uint32_t getTime();
    unsigned long getUnixTime();

private:
    uint8_t rxPin;
    uint8_t txPin;
    uint32_t baud;

    HardwareSerial gpsSerial;
    TinyGPSPlus gps;

    static constexpr uint8_t SPEED_SAMPLES = 5;
    float speedBuffer[SPEED_SAMPLES] = {0};
    uint8_t speedIndex = 0;
};