#include <time.h>
#include <esp_timer.h>
#include <esp_wifi.h>
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
    syncTimeNTP();
}

bool Tracker::ensureWifi() {
    if (WiFi.status() == WL_CONNECTED) {
        lastWifiFailAt = 0;
        return true;
    }

    if (lastWifiFailAt > 0 &&
        millis() - lastWifiFailAt < WIFI_RETRY_INTERVAL_MS) {
        Serial.println("[Tracker] WiFi skip (recent fail)");
        return false;
    }

    Serial.println("[Tracker] WiFi on...");
    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(WIFI_TX_POWER);
    WiFi.begin();

    unsigned long startedAt = millis();
    while (millis() - startedAt < 10000) {
        if (WiFi.status() == WL_CONNECTED) {
            wifiManagedByUs = true;
            wifiOnAt = millis();
            lastWifiFailAt = 0;
            Serial.println("[Tracker] WiFi ready");
            if (!timeSynced) syncTimeNTP();
            return true;
        }
        delay(100);
    }

    Serial.println("[Tracker] WiFi connect failed");
    lastWifiFailAt = millis();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
}

void Tracker::releaseWifi() {
    if (!wifiManagedByUs) return;

    unsigned long onDuration = millis() - wifiOnAt;
    Serial.print("[Tracker] WiFi off, was on for ");
    Serial.print(onDuration);
    Serial.println(" ms");

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiManagedByUs = false;
}

void Tracker::update() {
    if (portal.isActive()) {
        if (flushing) {
            flushFile.close();
            flushing = false;
        }
        wifiManagedByUs = false;
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
        releaseWifi();
        return;
    }

    if (flushing) {
        flushNextLine();
        return;
    }

    float speed = gps.getSpeed();
    if (speed > TRACKER_MAX_VALID_SPEED) {
        speed = 0.0f;
    }

    bool moving = speed >= TRACKER_SPEED_THRESHOLD;

    static bool lastMoving = false;
    if (moving != lastMoving) {
        Serial.print("[Tracker] mode: ");
        Serial.println(moving ? "MOVING" : "STATIONARY");
        lastMoving = moving;
    }

    if (!moving) {
        unsigned long now = millis();
        if (now - lastAccumAt >= ACCUM_INTERVAL_MS) {
            lastAccumAt = now;
            latAccum += gps.getLatitude();
            lngAccum += gps.getLongitude();
            accumCount++;
        }
    }

    if (!shouldSend()) {
        return;
    }

    lastSentAt = millis();

    double latRaw = gps.getLatitude();
    double lngRaw = gps.getLongitude();
    double lat, lng;

    if (!moving && accumCount > 0) {
        lat = latAccum / accumCount;
        lng = lngAccum / accumCount;
        latAccum = 0.0;
        lngAccum = 0.0;
        accumCount = 0;
    } else {
        lat = latRaw;
        lng = lngRaw;
        latAccum = 0.0;
        lngAccum = 0.0;
        accumCount = 0;
    }

    bool isInvalidCoordinates = false;

    if (ENABLE_HOME_POINT_FILTERING) {
        float dist = distanceTo(lat, lng, TRACKER_HOME_LAT, TRACKER_HOME_LNG);
        if (dist / 1000.0f > TRACKER_HOME_RADIUS_KM) {
            lat = SENDING_LAT;
            lng = SENDING_LNG;
            isInvalidCoordinates = true;
        }
    }

    if (isInvalidCoordinates) {
        speed = 0.0f;
    }

    float bearing = gps.getBearing();
    if (lastBearing < 0.0f) {
        lastBearing = bearing;
    }

    if (moving) {
        if (movingBufferCount < WIFI_MOVING_BATCH_SIZE) {
            TrackPoint& p = movingBuffer[movingBufferCount++];
            p.lat        = lat;
            p.lng        = lng;
            p.speed      = speed;
            p.bearing    = bearing;
            p.invalid    = isInvalidCoordinates;
            p.timestamp  = getSafeTimestamp(isInvalidCoordinates);
            p.voltage    = battery.getVoltage();
            p.altitude   = (float)gps.getAltitude();
            p.satellites = gps.getSatellites();
            p.hdop       = (float)gps.getHdop();
            p.freeKb     = (LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024;
        }

        if (movingBufferCount >= WIFI_MOVING_BATCH_SIZE) {
            sendMovingBatch();
        }
    } else {
        if (movingBufferCount > 0) {
            sendMovingBatch();
        }

        if (ensureWifi()) {
            Serial.print("[Tracker] WiFi status before send: ");
            Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");

            if (LittleFS.exists(TRACKER_BLACKBOX_PATH)) {
                startFlush();
            } else {
                sendToServer(lat, lng, speed, bearing, isInvalidCoordinates);
                releaseWifi();
            }
        } else {
            saveToBlackbox(lat, lng, speed, bearing, isInvalidCoordinates);
        }
    }

    lastBearing = bearing;
}

void Tracker::sendMovingBatch() {
    if (movingBufferCount == 0) return;

    if (!ensureWifi()) {
        for (uint8_t i = 0; i < movingBufferCount; i++) {
            TrackPoint& p = movingBuffer[i];
            saveToBlackbox(p.lat, p.lng, p.speed, p.bearing, p.invalid);
        }
        movingBufferCount = 0;
        return;
    }

    Serial.print("[Tracker] sending batch: ");
    Serial.println(movingBufferCount);

    for (uint8_t i = 0; i < movingBufferCount; i++) {
        TrackPoint& p = movingBuffer[i];

        String url = buildUrl(p.lat, p.lng, p.speed, p.bearing, p.timestamp,
                              p.voltage, p.altitude, p.satellites, p.hdop, p.freeKb);

        HTTPClient http;
        http.setTimeout(3000);
        http.setConnectTimeout(3000);
        http.begin(url);
        int code = http.GET();
        http.end();

        if (code > 0 || code == HTTPC_ERROR_READ_TIMEOUT) {
            led.blink(1, 30, 30);
            Serial.print("[Tracker] batch point sent: ");
            Serial.println(i + 1);
        } else {
            Serial.print("[Tracker] batch point error: ");
            Serial.println(http.errorToString(code));
            for (uint8_t j = i; j < movingBufferCount; j++) {
                TrackPoint& fp = movingBuffer[j];
                saveToBlackbox(fp.lat, fp.lng, fp.speed, fp.bearing, fp.invalid);
            }
            break;
        }

        delay(50);
    }

    movingBufferCount = 0;
    releaseWifi();
}

unsigned long Tracker::currentInterval() {
    if (!gps.hasLocation()) {
        return TRACKER_INTERVAL_STATIC;
    }

    float speed   = gps.getSpeed();
    float bearing = gps.getBearing();

    if (speed > TRACKER_MAX_VALID_SPEED) {
        speed = 0.0;
    }

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
    unsigned long timestamp = getSafeTimestamp(invalid);

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

    unsigned long timestamp = getSafeTimestamp(invalid);
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

    f.print(timestamp);        f.print(",");
    f.print(String(lat, 6));   f.print(",");
    f.print(String(lng, 6));   f.print(",");
    f.print(String(speed, 1)); f.print(",");
    f.print(String(bearing, 1)); f.print(",");
    f.print(String(voltage, 2)); f.print(",");
    f.print(String(altitude, 0)); f.print(",");
    f.print(String(sats));     f.print(",");
    f.print(String(hdop, 1));  f.print(",");
    f.println(String(freeKb));
    f.close();

    Serial.println("[Tracker] blackbox: saved");
}

void Tracker::startFlush() {
    flushFile = LittleFS.open(TRACKER_BLACKBOX_PATH, "r");
    if (!flushFile) {
        releaseWifi();
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
        releaseWifi();
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
    double lat          = line.substring(i0 + 1, i1).toDouble();
    double lng          = line.substring(i1 + 1, i2).toDouble();
    float speed         = line.substring(i2 + 1, i3).toFloat();
    float bearing       = line.substring(i3 + 1, i4).toFloat();
    float voltage       = line.substring(i4 + 1, i5).toFloat();
    float altitude      = line.substring(i5 + 1, i6).toFloat();
    uint32_t satellites = line.substring(i6 + 1, i7).toInt();
    float hdop          = line.substring(i7 + 1, i8).toFloat();
    uint32_t freeKb     = line.substring(i8 + 1).toInt();

    String url = buildUrl(lat, lng, speed, bearing, timestamp,
                          voltage, altitude, satellites, hdop, freeKb);

    HTTPClient http;
    http.setTimeout(5000);
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

void Tracker::syncTimeNTP() {
    if (WiFi.status() != WL_CONNECTED) return;

    configTime(0, 0, "pool.ntp.org", "time.google.com");

    struct tm timeinfo;
    unsigned long startedAt = millis();

    while (!getLocalTime(&timeinfo)) {
        if (millis() - startedAt > 5000) {
            Serial.println("[Tracker] NTP sync failed");
            return;
        }
        delay(100);
    }

    lastValidUnixTime = mktime(&timeinfo);
    lastValidMicros = esp_timer_get_time();
    timeSynced = true;

    Serial.print("[Tracker] NTP synced: ");
    Serial.println(lastValidUnixTime);
}

unsigned long Tracker::getSafeTimestamp(bool invalid) {
    if (!timeSynced && WiFi.status() == WL_CONNECTED) {
        syncTimeNTP();
    }

    static constexpr unsigned long MIN_VALID_TIME = 1704067200UL;

    if (!invalid) {
        unsigned long gpsTime = gps.hasTime() ? gps.getUnixTime() : 0;

        if (gpsTime > 0) {
            bool tooOld = gpsTime < MIN_VALID_TIME;
            bool tooFarInFuture = lastValidUnixTime > 0 &&
                                  gpsTime > lastValidUnixTime + ONE_YEAR_SECONDS;

            if (!tooOld && !tooFarInFuture) {
                lastValidUnixTime = gpsTime;
                lastValidMicros = esp_timer_get_time();
                return gpsTime;
            }

            Serial.print("[Tracker] suspicious GPS time: ");
            Serial.println(gpsTime);
        }
    } else {
        Serial.println("[Tracker] GPS time untrusted (spoofing)");
    }

    if (lastValidUnixTime > 0) {
        uint64_t elapsed = (esp_timer_get_time() - lastValidMicros) / 1000000ULL;
        return lastValidUnixTime + (unsigned long)elapsed;
    }

    return millis() / 1000;
}

void Tracker::forceReleaseWifi() {
    if (flushing) {
        flushFile.close();
        flushing = false;
    }
    movingBufferCount = 0;
    wifiManagedByUs = false;
    Serial.println("[Tracker] WiFi force released for portal");
}