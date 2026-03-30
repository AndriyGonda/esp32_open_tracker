#include "Tracker.h"

Tracker::Tracker(GpsReader& gps, AppSettings& settings, BatteryMonitor& battery, ConfigPortal& portal, LedController& led)
    : gps(gps), settings(settings), battery(battery), portal(portal), led(led) {
}

void Tracker::begin() {
    if (!LittleFS.begin(true)) {
        Serial.println("[Tracker] LittleFS mount failed");
        return;
    }
    Serial.println("[Tracker] LittleFS mounted");
    Serial.print("[Tracker] LittleFS total: ");
    Serial.print(LittleFS.totalBytes() / 1024);
    Serial.println(" KB");
}

void Tracker::update() {
    if (portal.isActive()) {
        if (flushing) {
            flushFile.close();
            flushing = false;
        }
        return;
    }

    if (!gps.hasLocation()) {
        return;
    }

    if (gps.getHdop() > MAX_HDOP) {
        Serial.print("[Tracker] poor HDOP: ");
        Serial.println(gps.getHdop());
        return;
    }

    if (flushing && WiFi.status() != WL_CONNECTED) {
        flushFile.close();
        flushing = false;
        return;
    }

    if (flushing) {
        flushNextLine();
        return;
    }

    if (!shouldSend()) {
        return;
    }

    lastSentAt = millis();

    double latRaw = gps.getLatitude();
    double lngRaw = gps.getLongitude();

    double lat = latRaw;
    double lng = lngRaw;
    bool isInvalidCoordinates = false;

    if (ENABLE_HOME_POINT_FILTERING) {
        float dist = distanceTo(latRaw, lngRaw, TRACKER_HOME_LAT, TRACKER_HOME_LNG);
        if (dist / 1000.0f > TRACKER_HOME_RADIUS_KM) {
            lat = SENDING_LAT;
            lng = SENDING_LNG;
            isInvalidCoordinates = true;
        }
    }

    float speed   = isInvalidCoordinates ? 0.0 : gps.getSpeed() ;
    float bearing = gps.getBearing();

    if (lastBearing < 0.0f) {
        lastBearing = bearing;
    }

    if (WiFi.status() == WL_CONNECTED) {
        if (LittleFS.exists(TRACKER_BLACKBOX_PATH)) {
            startFlush();
        } else {
            sendToServer(lat, lng, speed, bearing, isInvalidCoordinates);
        }
    } else {
        saveToBlackbox(lat, lng, speed, bearing, isInvalidCoordinates);
    }

    lastBearing = bearing;
}

unsigned long Tracker::currentInterval() {
    if (!gps.hasLocation()) {
        return TRACKER_INTERVAL_STATIC;
    }

    float speed   = gps.getSpeed();
    float bearing = gps.getBearing();

    if (speed < TRACKER_SPEED_THRESHOLD) {
        return TRACKER_INTERVAL_STATIC;
    }

    if (lastBearing >= 0.0f && bearingDiff(lastBearing, bearing) >= TRACKER_BEARING_THRESHOLD) {
        return TRACKER_INTERVAL_TURNING;
    }

    return TRACKER_INTERVAL_MOVING;
}

bool Tracker::shouldSend() {
    return millis() - lastSentAt >= currentInterval();
}

String Tracker::buildUrl(double lat, double lng, float speed, float bearing,
                         unsigned long timestamp, float voltage,
                         float altitude, uint32_t satellites,
                         float hdop, uint32_t freeKb) {
    String url = "http://";
    url += settings.getServerHost();
    url += ":";
    url += settings.getServerPort();
    url += "/?id=";
    url += TRACKER_DEVICE_ID;
    url += "&lat=";
    url += String(lat, 6);
    url += "&lon=";
    url += String(lng, 6);
    if (timestamp) {
        url += "&timestamp=";
        url += timestamp;
    }
    url += "&speed=";
    url += String(speed * 0.539957f, 1);
    url += "&bearing=";
    url += String(bearing, 1);
    url += "&batt=";
    url += String(voltage, 2);
    url += "&altitude=";
    url += String(altitude, 0);
    url += "&sat=";
    url += String(satellites);
    url += "&hdop=";
    url += String(hdop, 1);
    url += "&free_kb=";
    url += String(freeKb);
    return url;
}

void Tracker::sendToServer(double lat, double lng, float speed, float bearing, bool invalid) {
    unsigned long timestamp = gps.hasTime() ? gps.getUnixTime() : millis() / 1000;
    if (invalid) {
        timestamp = 0;
    }

    String url = buildUrl(lat, lng, speed, bearing, timestamp,
                          battery.getVoltage(),
                          (float)gps.getAltitude(),
                          gps.getSatellites(),
                          (float)gps.getHdop(),
                          (LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024);

    Serial.print("[Tracker] sending: ");
    Serial.println(url);

    HTTPClient http;
    http.setTimeout(3000);
    http.setConnectTimeout(5000);
    http.begin(url);

    int code = http.GET();

    if (code > 0 || code == HTTPC_ERROR_READ_TIMEOUT) {
        Serial.println("[Tracker] sent (no response expected)");
        led.blink(1, 50, 50);
    } else {
        Serial.print("[Tracker] HTTP error: ");
        Serial.println(http.errorToString(code));
        saveToBlackbox(lat, lng, speed, bearing, invalid);
    }

    http.end();
}

void Tracker::saveToBlackbox(double lat, double lng, float speed, float bearing, bool invalid) {
    size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();

    if (freeBytes < TRACKER_BLACKBOX_MIN_FREE) {
        Serial.println("[Tracker] blackbox: not enough space, skipping");
        return;
    }

    unsigned long timestamp = gps.hasTime() ? gps.getUnixTime() : millis() / 1000;
    float voltage    = battery.getVoltage();
    float altitude   = (float)gps.getAltitude();
    uint32_t sats    = gps.getSatellites();
    float hdop       = (float)gps.getHdop();
    uint32_t freeKb  = freeBytes / 1024;

    File f = LittleFS.open(TRACKER_BLACKBOX_PATH, "a");
    if (!f) {
        Serial.println("[Tracker] blackbox: failed to open file");
        return;
    }

    f.print(timestamp);   f.print(",");
    f.print(String(lat, 6));   f.print(",");
    f.print(String(lng, 6));   f.print(",");
    f.print(String(speed, 1)); f.print(",");
    f.print(String(bearing, 1)); f.print(",");
    f.print(String(voltage, 2)); f.print(",");
    f.print(String(altitude, 0)); f.print(",");
    f.print(String(sats));  f.print(",");
    f.print(String(hdop, 1)); f.print(",");
    f.println(String(freeKb));
    f.close();

    Serial.println("[Tracker] blackbox: saved");
}

void Tracker::startFlush() {
    flushFile = LittleFS.open(TRACKER_BLACKBOX_PATH, "r");
    if (!flushFile) {
        return;
    }
    flushing = true;
    Serial.println("[Tracker] start flushing blackbox...");
}

void Tracker::flushNextLine() {
    if (!flushFile.available()) {
        flushFile.close();
        flushing = false;
        LittleFS.remove(TRACKER_BLACKBOX_PATH);
        Serial.println("[Tracker] blackbox flushed and cleared");
        return;
    }

    String line = flushFile.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) return;

    int i0 = line.indexOf(',');
    int i1 = line.indexOf(',', i0 + 1);
    int i2 = line.indexOf(',', i1 + 1);
    int i3 = line.indexOf(',', i2 + 1);
    int i4 = line.indexOf(',', i3 + 1);
    int i5 = line.indexOf(',', i4 + 1);
    int i6 = line.indexOf(',', i5 + 1);
    int i7 = line.indexOf(',', i6 + 1);
    int i8 = line.indexOf(',', i7 + 1);

    if (i0 < 0 || i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 ||
        i5 < 0 || i6 < 0 || i7 < 0 || i8 < 0) return;

    unsigned long timestamp = line.substring(0, i0).toInt();
    double lat         = line.substring(i0 + 1, i1).toDouble();
    double lng         = line.substring(i1 + 1, i2).toDouble();
    float speed        = line.substring(i2 + 1, i3).toFloat();
    float bearing      = line.substring(i3 + 1, i4).toFloat();
    float voltage      = line.substring(i4 + 1, i5).toFloat();
    float altitude     = line.substring(i5 + 1, i6).toFloat();
    uint32_t satellites = line.substring(i6 + 1, i7).toInt();
    float hdop         = line.substring(i7 + 1, i8).toFloat();
    uint32_t freeKb    = line.substring(i8 + 1).toInt();

    String url = buildUrl(lat, lng, speed, bearing, timestamp,
                          voltage, altitude, satellites, hdop, freeKb);

    HTTPClient http;
    http.setTimeout(10000);
    http.setConnectTimeout(5000);
    http.begin(url);
    int code = http.GET();
    http.end();

    if (code > 0 || code == HTTPC_ERROR_READ_TIMEOUT) {
        led.blink(1, 50, 50);
        Serial.println("[Tracker] flush line: sent");
    } else {
        Serial.print("[Tracker] flush line error: ");
        Serial.println(http.errorToString(code));
    }
}

float Tracker::bearingDiff(float a, float b) {
    float diff = fabs(a - b);
    if (diff > 180.0f) diff = 360.0f - diff;
    return diff;
}

float Tracker::distanceTo(double lat1, double lng1, double lat2, double lng2) {
    const float R = 6371000.0f;
    float dLat = radians(lat2 - lat1);
    float dLng = radians(lng2 - lng1);
    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(radians(lat1)) * cos(radians(lat2)) *
              sin(dLng / 2) * sin(dLng / 2);
    return R * 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
}