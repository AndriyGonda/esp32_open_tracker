#include "GpsReader.h"

GpsReader::GpsReader(uint8_t rxPin, uint8_t txPin, uint32_t baud)
    : rxPin(rxPin), txPin(txPin), baud(baud), gpsSerial(1) {
}

void GpsReader::begin() {
    pinMode(GPS_SLEEP_PIN, OUTPUT);
    digitalWrite(GPS_SLEEP_PIN, GPS_WAKE_ACTIVE_LEVEL);
    gpsSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
    sleeping = false;
    wakeStartedAt = millis();
    lastDataAt = millis();
    Serial.println("[GpsReader] started");
}

void GpsReader::wake() {
    if (!sleeping) return;

    digitalWrite(GPS_SLEEP_PIN, GPS_WAKE_ACTIVE_LEVEL);
    delay(20);
    gpsSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
    sleeping = false;
    wakeStartedAt = millis();
    lastDataAt = wakeStartedAt;
    memset(speedBuffer, 0, sizeof(speedBuffer));
    speedIndex = 0;
    Serial.println("[GpsReader] wake");
}

void GpsReader::sleep() {
    if (sleeping) return;

    gpsSerial.flush();
    gpsSerial.end();
    digitalWrite(GPS_SLEEP_PIN, GPS_SLEEP_ACTIVE_LEVEL);
    sleeping = true;
    memset(speedBuffer, 0, sizeof(speedBuffer));
    speedIndex = 0;
    Serial.println("[GpsReader] sleep");
}

bool GpsReader::isSleeping() const {
    return sleeping;
}

void GpsReader::update() {
    if (sleeping) return;

    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
        lastDataAt = millis();
    }

    if (gps.satellites.isUpdated()) {
        Serial.print("[GpsReader] satellites=");
        Serial.println(gps.satellites.value());
    }
    if (gps.location.isUpdated()) {
        Serial.print("[GpsReader] lat=");
        Serial.print(getLatitude(), 6);
        Serial.print("  lng=");
        Serial.print(getLongitude(), 6);
        Serial.print("  hdop=");
        Serial.print(getHdop(), 1);
        Serial.print("  sats=");
        Serial.println(getSatellites());
    }

    if (gps.time.isUpdated()) {
        Serial.printf("[GpsReader] time=%02d:%02d:%02d  date=%02d.%02d.%04d\n",
            gps.time.hour(),
            gps.time.minute(),
            gps.time.second(),
            gps.date.day(),
            gps.date.month(),
            gps.date.year()
        );
    }
}

bool GpsReader::hasLocation() const {
    if (sleeping) return false;
    if (millis() - wakeStartedAt < GPS_WAKE_SETTLE_MS) return false;
    if (millis() - lastDataAt > GPS_STALE_DATA_MS) return false;
    return gps.location.isValid();
}

bool GpsReader::hasTime() const {
    if (sleeping) return false;
    if (millis() - wakeStartedAt < GPS_WAKE_SETTLE_MS) return false;
    if (millis() - lastDataAt > GPS_STALE_DATA_MS) return false;
    return gps.time.isValid();
}

double GpsReader::getLatitude()     { return gps.location.lat(); }
double GpsReader::getLongitude()    { return gps.location.lng(); }

double GpsReader::getAltitude() {
    double alt = gps.altitude.meters();
    return alt < 0.0 ? 0.0 : alt;
}

float GpsReader::getBearing()       { return gps.course.deg(); }
double GpsReader::getHdop()         { return gps.hdop.hdop(); }
uint32_t GpsReader::getSatellites() { return gps.satellites.value(); }
uint32_t GpsReader::getDate()       { return gps.date.value(); }
uint32_t GpsReader::getTime()       { return gps.time.value(); }
unsigned long GpsReader::getWakeStartedAt() const { return wakeStartedAt; }
unsigned long GpsReader::getLastDataAt() const { return lastDataAt; }

unsigned long GpsReader::getUnixTime() {
    if (!gps.date.isValid() || !gps.time.isValid()) return 0;

    struct tm t = {};
    t.tm_year = gps.date.year() - 1900;
    t.tm_mon  = gps.date.month() - 1;
    t.tm_mday = gps.date.day();
    t.tm_hour = gps.time.hour();
    t.tm_min  = gps.time.minute();
    t.tm_sec  = gps.time.second();

    return (unsigned long)mktime(&t);
}

float GpsReader::getSpeed() {
    float raw = gps.speed.kmph();
    speedBuffer[speedIndex] = raw;
    speedIndex = (speedIndex + 1) % SPEED_SAMPLES;

    float sum = 0;
    for (uint8_t i = 0; i < SPEED_SAMPLES; i++) {
        sum += speedBuffer[i];
    }
    return sum / SPEED_SAMPLES;
}
