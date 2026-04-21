#include <time.h>
#include <math.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include "Tracker.h"

Tracker::Tracker(GpsReader& gps, AppSettings& settings, BatteryMonitor& battery, ConfigPortal& portal, LedController& led)
    : gps(gps), settings(settings), battery(battery), portal(portal), led(led) {
}

bool Tracker::isZeroCoordinate(double lat, double lng) const {
    return fabs(lat) < COORD_EPS && fabs(lng) < COORD_EPS;
}

bool Tracker::isHomeCoordinate(double lat, double lng) const {
    return fabs(lat - TRACKER_HOME_LAT) < COORD_EPS &&
           fabs(lng - TRACKER_HOME_LNG) < COORD_EPS;
}

bool Tracker::isPinCandidateCoordinate(double lat, double lng) const {
    if (isZeroCoordinate(lat, lng)) return false;
    if (isHomeCoordinate(lat, lng)) return false;
    return true;
}

void Tracker::storeLastKnownGood(double lat, double lng, float bearing) {
    if (isZeroCoordinate(lat, lng)) return;
    if (isHomeCoordinate(lat, lng)) return;

    hasLastKnownGood = true;
    lastKnownGoodLat = lat;
    lastKnownGoodLng = lng;
    lastKnownGoodBearing = bearing;
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

    if (lastWifiFailAt > 0 && millis() - lastWifiFailAt < WIFI_RETRY_INTERVAL_MS) {
        Serial.print("[Tracker] WiFi skip (recent fail), retry in ");
        Serial.print((WIFI_RETRY_INTERVAL_MS - (millis() - lastWifiFailAt)) / 1000);
        Serial.println(" sec");
        return false;
    }

    static constexpr unsigned long WIFI_TOTAL_TIMEOUT_MS = 15000;
    unsigned long wifiStart = millis();

    Serial.println("[Tracker] WiFi on...");
    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(WIFI_TX_POWER);

    uint8_t count = settings.getWifiCount();
    bool connected = false;

    for (uint8_t i = 0; i < count && !connected; i++) {
        if (millis() - wifiStart >= WIFI_TOTAL_TIMEOUT_MS) {
            Serial.println("[Tracker] WiFi total timeout");
            break;
        }

        auto wifi = settings.getWifi(i);
        String ssid = wifi.ssid;
        ssid.trim();
        if (ssid.isEmpty()) continue;

        Serial.print("[Tracker] trying: ");
        Serial.println(ssid);

        WiFi.begin(ssid.c_str(), wifi.password.c_str());

        unsigned long startedAt = millis();
        while (millis() - startedAt < 10000 && millis() - wifiStart < WIFI_TOTAL_TIMEOUT_MS) {
            if (WiFi.status() == WL_CONNECTED) {
                connected = true;
                break;
            }
            if (WiFi.status() == WL_CONNECT_FAILED || WiFi.status() == WL_NO_SSID_AVAIL) {
                break;
            }
            delay(100);
        }

        if (!connected) {
            WiFi.disconnect(false, false);
            delay(200);
        }
    }

    if (connected) {
        wifiManagedByUs = true;
        wifiOnAt = millis();
        lastWifiFailAt = 0;
        lastConnectSuccess = true;
        Serial.println("[Tracker] WiFi ready");
        if (!timeSynced) syncTimeNTP();
        return true;
    }

    Serial.println("[Tracker] WiFi connect failed");
    lastWifiFailAt = millis();
    lastConnectSuccess = false;
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

unsigned long Tracker::currentInterval(bool moving, float currentBearing) const {
    if (!moving) return TRACKER_INTERVAL_STATIC;

    if (lastBearing >= 0.0f && bearingDiff(lastBearing, currentBearing) >= TRACKER_BEARING_THRESHOLD) {
        return TRACKER_INTERVAL_TURNING;
    }

    return TRACKER_INTERVAL_MOVING;
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

    const bool gpsValid =
        gps.hasLocation() &&
        gps.getHdop() <= MAX_HDOP &&
        gps.getSatellites() >= MIN_SATELLITES;

    float speed = gpsValid ? gps.getSpeed() : 0.0f;
    if (speed > TRACKER_MAX_VALID_SPEED) speed = 0.0f;

#if ENABLE_PARKING_FILTER
    parkingFilterUpdate(speed);
    bool moving = gpsValid ? parkingIsMoving() : false;
#else
    bool moving = gpsValid && (speed >= TRACKER_SPEED_THRESHOLD);
#endif

    static bool lastMovingPrinted = false;
    if (moving != lastMovingPrinted) {
        Serial.print("[Tracker] mode: ");
        Serial.println(moving ? "MOVING" : "STATIONARY");

        if (!moving && wifiManagedByUs) {
            releaseWifi();
        }

        lastMovingPrinted = moving;
    }

    double lat = 0.0;
    double lng = 0.0;
    bool isInvalidCoordinates = false;
    bool hasUsableCoordinates = false;

    float currentBearing = gps.getBearing();

    if (gpsValid) {
        lat = gps.getLatitude();
        lng = gps.getLongitude();

        if (ENABLE_HOME_POINT_FILTERING) {
            float rawDist = distanceTo(lat, lng, TRACKER_HOME_LAT, TRACKER_HOME_LNG);

            if (rawDist / 1000.0f > TRACKER_HOME_RADIUS_KM) {
                Serial.printf("[Tracker] GPS garbage detected: %.6f, %.6f\n", lat, lng);
                isInvalidCoordinates = true;

                if (hasLastKnownGood) {
                    lat = lastKnownGoodLat;
                    lng = lastKnownGoodLng;
                    currentBearing = lastKnownGoodBearing;
                    hasUsableCoordinates = true;
                    Serial.printf("[Tracker] using last known good: %.6f, %.6f\n", lat, lng);
                } else if (battery.isLow()) {
                    lat = TRACKER_HOME_LAT;
                    lng = TRACKER_HOME_LNG;
                    hasUsableCoordinates = true;
                    Serial.println("[Tracker] no last known good, low battery -> using HOME");
                } else {
                    lat = 0.0;
                    lng = 0.0;
                    hasUsableCoordinates = false;
                    Serial.println("[Tracker] garbage GPS dropped");
                }
            } else {
                hasUsableCoordinates = !isZeroCoordinate(lat, lng);
                if (hasUsableCoordinates) storeLastKnownGood(lat, lng, currentBearing);
            }
        } else {
            hasUsableCoordinates = !isZeroCoordinate(lat, lng);
            if (hasUsableCoordinates) storeLastKnownGood(lat, lng, currentBearing);
        }
    } else {
        isInvalidCoordinates = true;

        if (hasLastKnownGood) {
            lat = lastKnownGoodLat;
            lng = lastKnownGoodLng;
            currentBearing = lastKnownGoodBearing;
            hasUsableCoordinates = true;
            Serial.printf("[Tracker] GPS unavailable, using last known good: %.6f, %.6f\n", lat, lng);
        } else if (battery.isLow()) {
            lat = TRACKER_HOME_LAT;
            lng = TRACKER_HOME_LNG;
            hasUsableCoordinates = true;
            Serial.println("[Tracker] GPS unavailable, low battery -> using HOME");
        } else {
            Serial.println("[Tracker] GPS unavailable, no cached point");
        }
    }

#if ENABLE_PARKING_FILTER
    parkingApply(lat, lng, speed, moving);
#endif

    unsigned long sendInterval = currentInterval(moving, currentBearing);
    if (millis() - lastSentAt < sendInterval) {
        return;
    }
    lastSentAt = millis();

    if (!hasUsableCoordinates && !_pinned) {
        Serial.println("[Tracker] no usable coordinates -> skip send");
        return;
    }

    if (isInvalidCoordinates) {
        speed = 0.0f;
        if (ensureWifi()) {
            sendToServer(lat, lng, speed, currentBearing, 0.0f, true);
            releaseWifi();
        } else {
            saveToBlackbox(lat, lng, speed, currentBearing, 0.0f, true);
        }
        return;
    }

    float bearing = currentBearing;
    if (lastBearing < 0.0f) {
        lastBearing = bearing;
    }

    float accel = 0.0f;

    if (moving) {
        if (lastConnectSuccess && WiFi.status() != WL_CONNECTED) {
            ensureWifi();
        }

        if (WiFi.status() == WL_CONNECTED) {
            if (LittleFS.exists(TRACKER_BLACKBOX_PATH)) {
                startFlush();
            } else {
                sendToServer(lat, lng, speed, bearing, accel, false);
            }
        } else {
            saveToBlackbox(lat, lng, speed, bearing, accel, false);
        }
    } else {
        if (ensureWifi()) {
            Serial.print("[Tracker] WiFi status before send: ");
            Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");

            if (LittleFS.exists(TRACKER_BLACKBOX_PATH)) {
                startFlush();
            } else {
                sendToServer(lat, lng, speed, bearing, accel, false);
                releaseWifi();
            }
        } else {
            saveToBlackbox(lat, lng, speed, bearing, accel, false);
        }
    }

    lastBearing = bearing;
}

String Tracker::buildUrl(double lat, double lng, float speed, float bearing,
                         unsigned long timestamp, float voltage,
                         float altitude, uint32_t satellites,
                         float hdop, uint32_t freeKb, float accel, bool invalid) {
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
    url += "&accel=";
    url += String(accel, 3);
    url += "&invalid=";
    url += invalid ? "true" : "false";
    return url;
}

void Tracker::sendToServer(double lat, double lng, float speed, float bearing,
                           float accel, bool invalid) {
    if (isZeroCoordinate(lat, lng)) {
        Serial.printf("[Tracker] zero coordinates blocked, skip send: %.6f, %.6f\n", lat, lng);
        return;
    }

    if (isHomeCoordinate(lat, lng) && !battery.isLow()) {
        Serial.printf("[Tracker] HOME blocked, battery OK: %.6f, %.6f\n", lat, lng);
        return;
    }

    unsigned long timestamp = getSafeTimestamp(invalid);

    String url = buildUrl(lat, lng, speed, bearing, timestamp,
                          battery.getVoltage(),
                          (float)gps.getAltitude(),
                          gps.getSatellites(),
                          (float)gps.getHdop(),
                          (LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024,
                          accel, invalid);

    if (url.length() > 512) {
        Serial.println("[Tracker] URL too long, saving to blackbox");
        saveToBlackbox(lat, lng, speed, bearing, accel, invalid);
        return;
    }

    Serial.print("[Tracker] sending: ");
    Serial.println(url);

    HTTPClient http;
    http.setTimeout(3000);
    http.setConnectTimeout(5000);
    http.begin(url);

    int code = http.GET();
    http.end();

    if (code > 0 || code == HTTPC_ERROR_READ_TIMEOUT || code == HTTPC_ERROR_CONNECTION_LOST) {
        Serial.println("[Tracker] sent");
        led.blink(1, 50, 50);
    } else {
        Serial.print("[Tracker] HTTP error: ");
        Serial.println(http.errorToString(code));
        saveToBlackbox(lat, lng, speed, bearing, accel, invalid);
    }
}

void Tracker::saveToBlackbox(double lat, double lng, float speed, float bearing,
                             float accel, bool invalid) {
    if (isZeroCoordinate(lat, lng)) {
        Serial.printf("[Tracker] zero coordinates blocked, skip blackbox: %.6f, %.6f\n", lat, lng);
        return;
    }

    if (isHomeCoordinate(lat, lng) && !battery.isLow()) {
        Serial.printf("[Tracker] HOME blocked, skip blackbox: %.6f, %.6f\n", lat, lng);
        return;
    }

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

    f.print(timestamp);           f.print(",");
    f.print(String(lat, 6));      f.print(",");
    f.print(String(lng, 6));      f.print(",");
    f.print(String(speed, 1));    f.print(",");
    f.print(String(bearing, 1));  f.print(",");
    f.print(String(voltage, 2));  f.print(",");
    f.print(String(altitude, 0)); f.print(",");
    f.print(String(sats));        f.print(",");
    f.print(String(hdop, 1));     f.print(",");
    f.print(String(freeKb));      f.print(",");
    f.print(String(accel, 3));    f.print(",");
    f.println(invalid ? "1" : "0");
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

    int i0  = line.indexOf(',');
    int i1  = line.indexOf(',', i0 + 1);
    int i2  = line.indexOf(',', i1 + 1);
    int i3  = line.indexOf(',', i2 + 1);
    int i4  = line.indexOf(',', i3 + 1);
    int i5  = line.indexOf(',', i4 + 1);
    int i6  = line.indexOf(',', i5 + 1);
    int i7  = line.indexOf(',', i6 + 1);
    int i8  = line.indexOf(',', i7 + 1);
    int i9  = line.indexOf(',', i8 + 1);
    int i10 = line.indexOf(',', i9 + 1);

    if (i0 < 0 || i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 ||
        i5 < 0 || i6 < 0 || i7 < 0 || i8 < 0 || i9 < 0 || i10 < 0) {
        Serial.println("[Tracker] flush line malformed, skipped");
        return;
    }

    unsigned long timestamp = line.substring(0, i0).toInt();
    double lat          = line.substring(i0 + 1, i1).toDouble();
    double lng          = line.substring(i1 + 1, i2).toDouble();
    float speed         = line.substring(i2 + 1, i3).toFloat();
    float bearing       = line.substring(i3 + 1, i4).toFloat();
    float voltage       = line.substring(i4 + 1, i5).toFloat();
    float altitude      = line.substring(i5 + 1, i6).toFloat();
    uint32_t satellites = line.substring(i6 + 1, i7).toInt();
    float hdop          = line.substring(i7 + 1, i8).toFloat();
    uint32_t freeKb     = line.substring(i8 + 1, i9).toInt();
    float accel         = line.substring(i9 + 1, i10).toFloat();
    bool invalid        = line.substring(i10 + 1).toInt() == 1;

    if (isZeroCoordinate(lat, lng)) {
        Serial.printf("[Tracker] flush skipped zero coordinates: %.6f, %.6f\n", lat, lng);
        return;
    }

    if (isHomeCoordinate(lat, lng) && !battery.isLow()) {
        Serial.printf("[Tracker] flush skipped HOME coordinates: %.6f, %.6f\n", lat, lng);
        return;
    }

    String url = buildUrl(lat, lng, speed, bearing, timestamp,
                          voltage, altitude, satellites, hdop, freeKb, accel, invalid);

    HTTPClient http;
    http.setTimeout(5000);
    http.setConnectTimeout(5000);
    http.begin(url);
    int code = http.GET();
    http.end();

    if (code > 0 || code == HTTPC_ERROR_READ_TIMEOUT || code == HTTPC_ERROR_CONNECTION_LOST) {
        led.blink(1, 50, 50);
        Serial.println("[Tracker] flush line: sent");
    } else {
        Serial.print("[Tracker] flush line error: ");
        Serial.println(http.errorToString(code));
    }
}

void Tracker::parkingFilterUpdate(float speed) {
    unsigned long now = millis();
    _lastIntervalMs = (_lastParkingUpdateMs > 0)
                    ? (now - _lastParkingUpdateMs)
                    : (unsigned long)TRACKER_INTERVAL_STATIC;
    _lastParkingUpdateMs = now;

    _speedWindow[_speedWindowIdx] = speed;
    _speedWindowIdx = (_speedWindowIdx + 1) % PARKING_WINDOW_SIZE;
    if (_speedWindowIdx == 0) _speedWindowFull = true;
}

bool Tracker::parkingIsMoving() const {
    uint8_t total = _speedWindowFull ? PARKING_WINDOW_SIZE : _speedWindowIdx;

    if (total < PARKING_MOTION_MIN_MOVING) {
        return gps.getSpeed() >= TRACKER_SPEED_THRESHOLD;
    }

    uint8_t movingCount = 0;
    for (uint8_t i = 0; i < total; i++) {
        if (_speedWindow[i] >= TRACKER_SPEED_THRESHOLD) movingCount++;
    }

    if (movingCount < PARKING_MOTION_MIN_MOVING) return false;

    uint8_t idxCurr = (_speedWindowIdx + PARKING_WINDOW_SIZE - 1) % PARKING_WINDOW_SIZE;
    uint8_t idxPrev = (_speedWindowIdx + PARKING_WINDOW_SIZE - 2) % PARKING_WINDOW_SIZE;

    float currSpeed = _speedWindow[idxCurr];
    float prevSpeed = _speedWindow[idxPrev];

    float intervalSec = (float)_lastIntervalMs / 1000.0f;
    if (intervalSec < 0.1f) intervalSec = 0.1f;

    float accel = fabs(currSpeed - prevSpeed) / intervalSec;

    if (accel > PARKING_MAX_ACCELERATION) {
        Serial.print("[Parking] speed spike rejected, accel=");
        Serial.print(accel);
        Serial.println(" km/h/s");
        return false;
    }

    return true;
}

void Tracker::parkingApply(double& lat, double& lng, float& speed, bool moving) {
    if (moving) {
        if (_pinned) {
            Serial.println("[Parking] pin released, device moving");
        }
        _pinned = false;
        _stationarySince = 0;
        return;
    }

    if (_stationarySince == 0) {
        _stationarySince = millis();
        Serial.println("[Parking] stationary timer started");
    }

    unsigned long stationaryMs = millis() - _stationarySince;

    if (!_pinned) {
        if (stationaryMs >= (unsigned long)PARKING_PIN_DELAY_SEC * 1000UL) {
            if (!isPinCandidateCoordinate(lat, lng)) {
                Serial.printf("[Parking] pin skipped for blocked coordinates: %.6f, %.6f\n", lat, lng);
                speed = 0.0f;
                return;
            }

            _pinLat = lat;
            _pinLng = lng;
            _pinned = true;
            Serial.print("[Parking] pin set: ");
            Serial.print(String(_pinLat, 6));
            Serial.print(", ");
            Serial.println(String(_pinLng, 6));
        }
        return;
    }

    lat = _pinLat;
    lng = _pinLng;
    speed = 0.0f;
}

float Tracker::bearingDiff(float a, float b) const {
    float diff = fabs(a - b);
    if (diff > 180.0f) diff = 360.0f - diff;
    return diff;
}

float Tracker::distanceTo(double lat1, double lng1, double lat2, double lng2) const {
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

    if (!timezoneSync.isSynced()) {
        timezoneSync.fetch();
    }

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

    lastValidUnixTime = mktime(&timeinfo) + timezoneSync.getOffsetSec();
    timeSynced = true;

    Serial.print("[Tracker] NTP synced: ");
    Serial.print(lastValidUnixTime);
    Serial.print(" (offset=");
    Serial.print(timezoneSync.getOffsetSec());
    Serial.println(")");
}

unsigned long Tracker::getSafeTimestamp(bool invalid) {
    if (!timeSynced && WiFi.status() == WL_CONNECTED) {
        syncTimeNTP();
    }

    static constexpr unsigned long MIN_VALID_TIME = 1704067200UL;
    static constexpr unsigned long MAX_VALID_TIME = 1893456000UL;

    if (!invalid) {
        unsigned long gpsTime = gps.hasTime() ? gps.getUnixTime() : 0;

        if (gpsTime > 0) {
            bool tooOld         = gpsTime < MIN_VALID_TIME;
            bool tooFarInFuture = gpsTime > MAX_VALID_TIME;
            bool inconsistent   = lastValidUnixTime > 0 && gpsTime > lastValidUnixTime + ONE_YEAR_SECONDS;

            if (!tooOld && !tooFarInFuture && !inconsistent) {
                lastValidUnixTime = gpsTime;
                return gpsTime;
            }

            Serial.print("[Tracker] suspicious GPS time rejected: ");
            Serial.println(gpsTime);
        }
    } else {
        Serial.println("[Tracker] invalid coords, using last valid timestamp");
    }

    if (timeSynced) {
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            unsigned long ntpTime = (unsigned long)mktime(&timeinfo) + timezoneSync.getOffsetSec();
            if (ntpTime >= MIN_VALID_TIME && ntpTime <= MAX_VALID_TIME) {
                if (!invalid) lastValidUnixTime = ntpTime;
                return ntpTime;
            }
            Serial.print("[Tracker] suspicious NTP time rejected: ");
            Serial.println(ntpTime);
            timeSynced = false;
        }
    }

    if (lastValidUnixTime >= MIN_VALID_TIME && lastValidUnixTime <= MAX_VALID_TIME) {
        Serial.println("[Tracker] using last known valid time");
        return lastValidUnixTime;
    }

    return millis() / 1000;
}

void Tracker::forceReleaseWifi() {
    if (flushing) {
        flushFile.close();
        flushing = false;
    }
    wifiManagedByUs = false;
    lastConnectSuccess = false;
    Serial.println("[Tracker] WiFi force released for portal");
}
