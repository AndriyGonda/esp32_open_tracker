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

#if ENABLE_IMU
    bool imuOk = false;
    for (uint8_t attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("[Tracker] IMU init attempt %d/3\n", attempt);
        if (imu.begin()) {
            imu.setThreshold(IMU_ACCEL_THRESHOLD);
            Serial.println("[Tracker] IMU ready");
            imuOk = true;
            break;
        }
        delay(1000);
    }
    if (!imuOk) {
        Serial.println("[Tracker] IMU init failed, working without IMU");
    }
#endif

    syncTimeNTP();
}


bool Tracker::ensureWifi() {
    if (WiFi.status() == WL_CONNECTED) {
        lastWifiFailAt = 0;
        return true;
    }

    if (lastWifiFailAt > 0 &&
        millis() - lastWifiFailAt < WIFI_RETRY_INTERVAL_MS) {
        Serial.print("[Tracker] WiFi skip (recent fail), retry in ");
        Serial.print((WIFI_RETRY_INTERVAL_MS - (millis() - lastWifiFailAt)) / 1000);
        Serial.println(" sec");
        return false;
    }

    Serial.println("[Tracker] WiFi on...");
    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(WIFI_TX_POWER);

    uint8_t count = settings.getWifiCount();
    bool connected = false;

    for (uint8_t i = 0; i < count && !connected; i++) {
        auto wifi = settings.getWifi(i);
        String ssid = wifi.ssid;
        ssid.trim();
        if (ssid.isEmpty()) continue;

        Serial.print("[Tracker] trying: ");
        Serial.println(ssid);

        WiFi.begin(ssid.c_str(), wifi.password.c_str());

        unsigned long startedAt = millis();
        while (millis() - startedAt < 10000) {
            if (WiFi.status() == WL_CONNECTED) {
                connected = true;
                break;
            }
            if (WiFi.status() == WL_CONNECT_FAILED ||
                WiFi.status() == WL_NO_SSID_AVAIL) {
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
        return;
    }

    if (gps.getSatellites() < MIN_SATELLITES) {
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

#if ENABLE_IMU
    imu.update();

    static unsigned long lastImuPrint = 0;
    if (millis() - lastImuPrint >= 2000) {
        lastImuPrint = millis();
        Serial.printf("[IMU] accel=%.3f m/s²  motion=%s\n",
            imu.getAccelMag(),
            imu.isMoving() ? "YES" : "NO");
    }
#endif

#if ENABLE_PARKING_FILTER
    parkingFilterUpdate(speed);
    bool gpsMoving = parkingIsMoving();
#else
    bool gpsMoving = speed >= TRACKER_SPEED_THRESHOLD;
#endif

#if ENABLE_IMU
    bool _imuSays  = imu.isMoving();
    bool _imuValid = imu.getAccelMag() > 0.0f;
    bool moving;
    if (!_imuValid) {
        moving = gpsMoving;
    } else if (IMU_FUSION_STRATEGY == FUSION_AND) {
        moving = gpsMoving && _imuSays;
    } else if (IMU_FUSION_STRATEGY == FUSION_OR) {
        moving = gpsMoving || _imuSays;
    } else if (IMU_FUSION_STRATEGY == FUSION_IMU) {
        moving = _imuSays;
    } else {
        moving = gpsMoving;
    }
#else
    bool moving = gpsMoving;
#endif

    static bool lastMoving = false;
    if (moving != lastMoving) {
        Serial.print("[Tracker] mode: ");
        Serial.println(moving ? "MOVING" : "STATIONARY");

        if (!moving && wifiManagedByUs) {
            releaseWifi();
        }

        lastMoving = moving;
    }

    if (!shouldSend()) {
        return;
    }

    lastSentAt = millis();

    double lat = gps.getLatitude();
    double lng = gps.getLongitude();
    bool isInvalidCoordinates = false;

#if ENABLE_PARKING_FILTER
    parkingApply(lat, lng, speed);
#endif

    if (ENABLE_HOME_POINT_FILTERING) {
        float dist = distanceTo(lat, lng, TRACKER_HOME_LAT, TRACKER_HOME_LNG);
        if (dist / 1000.0f > TRACKER_HOME_RADIUS_KM) {
#if ENABLE_WIFI_POSITIONING
            Serial.println("[Tracker] coordinates outside home zone, trying WiFi positioning...");
            if (ensureWifi()) {
                WifiPosition wpos = wifiPositioning.locate();
                if (wpos.valid) {
                    Serial.print("[Tracker] WiFi position found, accuracy=");
                    Serial.println(wpos.accuracy);
                    lat = wpos.lat;
                    lng = wpos.lng;
                } else {
                    Serial.println("[Tracker] WiFi positioning failed, using fallback");
                    if (battery.isLow()) {
                        Serial.println("[Tracker] low battery, falling back to home coords");
                        lat = TRACKER_HOME_LAT;
                        lng = TRACKER_HOME_LNG;
                    } else {
                        lat = SENDING_LAT;
                        lng = SENDING_LNG;
                    }
                    isInvalidCoordinates = true;
                }
            } else {
                if (battery.isLow()) {
                    Serial.println("[Tracker] low battery, falling back to home coords");
                    lat = TRACKER_HOME_LAT;
                    lng = TRACKER_HOME_LNG;
                } else {
                    lat = SENDING_LAT;
                    lng = SENDING_LNG;
                }
                isInvalidCoordinates = true;
            }
#else
            if (battery.isLow()) {
                Serial.println("[Tracker] low battery, falling back to home coords");
                lat = TRACKER_HOME_LAT;
                lng = TRACKER_HOME_LNG;
            } else {
                lat = SENDING_LAT;
                lng = SENDING_LNG;
            }
            isInvalidCoordinates = true;
#endif
        }
    }

    if (isInvalidCoordinates) {
        speed = 0.0f;
    }

    float bearing = gps.getBearing();
    if (lastBearing < 0.0f) {
        lastBearing = bearing;
    }

#if ENABLE_IMU
    float accel = imu.getAccelMag();
#else
    float accel = 0.0f;
#endif

    if (moving) {
        if (lastConnectSuccess) {
            if (WiFi.status() != WL_CONNECTED) {
                ensureWifi();
            }
        }

        if (WiFi.status() == WL_CONNECTED) {
            if (LittleFS.exists(TRACKER_BLACKBOX_PATH)) {
                startFlush();
            } else {
                sendToServer(lat, lng, speed, bearing, accel, isInvalidCoordinates);
            }
        } else {
            saveToBlackbox(lat, lng, speed, bearing, accel, isInvalidCoordinates);
        }
    } else {
        if (ensureWifi()) {
            Serial.print("[Tracker] WiFi status before send: ");
            Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");

            if (LittleFS.exists(TRACKER_BLACKBOX_PATH)) {
                startFlush();
            } else {
                sendToServer(lat, lng, speed, bearing, accel, isInvalidCoordinates);
                releaseWifi();
            }
        } else {
            saveToBlackbox(lat, lng, speed, bearing, accel, isInvalidCoordinates);
        }
    }

    lastBearing = bearing;
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
                         float hdop, uint32_t freeKb, float accel) {
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
    return url;
}

void Tracker::sendToServer(double lat, double lng, float speed, float bearing,
                           float accel, bool invalid) {
    unsigned long timestamp = getSafeTimestamp(invalid);

    String url = buildUrl(lat, lng, speed, bearing, timestamp,
                          battery.getVoltage(),
                          (float)gps.getAltitude(),
                          gps.getSatellites(),
                          (float)gps.getHdop(),
                          (LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024,
                          accel);

    Serial.print("[Tracker] sending: ");
    Serial.println(url);

    HTTPClient http;
    http.setTimeout(3000);
    http.setConnectTimeout(5000);
    http.begin(url);

    int code = http.GET();
    http.end();

    if (code > 0 || code == HTTPC_ERROR_READ_TIMEOUT
                 || code == HTTPC_ERROR_CONNECTION_LOST) {
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
    f.println(String(accel, 3));
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
    int i9 = line.indexOf(',', i8 + 1);

    if (i0 < 0 || i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0 ||
        i5 < 0 || i6 < 0 || i7 < 0 || i8 < 0 || i9 < 0) return;

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
    float accel         = line.substring(i9 + 1).toFloat();

    String url = buildUrl(lat, lng, speed, bearing, timestamp,
                          voltage, altitude, satellites, hdop, freeKb, accel);

    HTTPClient http;
    http.setTimeout(5000);
    http.setConnectTimeout(5000);
    http.begin(url);
    int code = http.GET();
    http.end();

    if (code > 0 || code == HTTPC_ERROR_READ_TIMEOUT
                 || code == HTTPC_ERROR_CONNECTION_LOST) {
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

void Tracker::parkingApply(double& lat, double& lng, float& speed) {
    bool moving = parkingIsMoving();

#if ENABLE_IMU
    if (!moving && imu.isMoving() && imu.getAccelMag() > 0.0f) {
        Serial.printf("[Parking] IMU motion override, accel=%.3f m/s²\n",
                      imu.getAccelMag());
        moving = true;
    }
#endif

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

    float drift = distanceTo(lat, lng, _pinLat, _pinLng);
    if (drift <= PARKING_PIN_RADIUS_M) {
        lat   = _pinLat;
        lng   = _pinLng;
        speed = 0.0f;
    } else {
        Serial.print("[Parking] drift anomaly ignored: ");
        Serial.print(drift);
        Serial.println(" m");
        lat   = _pinLat;
        lng   = _pinLng;
        speed = 0.0f;
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

    if (!invalid) {
        unsigned long gpsTime = gps.hasTime() ? gps.getUnixTime() : 0;

        if (gpsTime > 0) {
            bool tooOld = gpsTime < MIN_VALID_TIME;
            bool tooFarInFuture = lastValidUnixTime > 0 &&
                                  gpsTime > lastValidUnixTime + ONE_YEAR_SECONDS;

            if (!tooOld && !tooFarInFuture) {
                lastValidUnixTime = gpsTime;
                return gpsTime;
            }

            Serial.print("[Tracker] suspicious GPS time: ");
            Serial.println(gpsTime);
        }
    } else {
        Serial.println("[Tracker] GPS time untrusted (spoofing)");
    }

    if (timeSynced) {
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            return (unsigned long)mktime(&timeinfo) + timezoneSync.getOffsetSec();
        }
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

#if ENABLE_IMU
bool Tracker::imuMoving() const {
    return imu.isMoving();
}
#endif