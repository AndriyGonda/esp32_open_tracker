#include "GpsReader.h"

GpsReader::GpsReader(uint8_t rxPin, uint8_t txPin, uint32_t baud)
    : rxPin(rxPin), txPin(txPin), baud(baud), gpsSerial(1) {
}

void GpsReader::begin() {
    gpsSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
    Serial.println("[GpsReader] started");
}

void GpsReader::update() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
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

bool GpsReader::hasLocation() const { return gps.location.isValid(); }
bool GpsReader::hasTime() const     { return gps.time.isValid(); }

double GpsReader::getLatitude()     { return gps.location.lat(); }
double GpsReader::getLongitude()    { return gps.location.lng(); }

double GpsReader::getAltitude() {
    double alt = gps.altitude.meters();
    return alt < 0.0 ? 0.0 : alt;
}

float GpsReader::getSpeed()         { return gps.speed.kmph(); }
float GpsReader::getBearing()       { return gps.course.deg(); }
double GpsReader::getHdop()         { return gps.hdop.hdop(); }
uint32_t GpsReader::getSatellites() { return gps.satellites.value(); }
uint32_t GpsReader::getDate()       { return gps.date.value(); }
uint32_t GpsReader::getTime()       { return gps.time.value(); }

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