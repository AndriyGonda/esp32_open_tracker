#include "TimezoneSync.h"

bool TimezoneSync::fetch() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[TimezoneSync] no WiFi");
        return false;
    }

    HTTPClient http;
    http.setTimeout(5000);
    http.begin(API_URL);

    int code = http.GET();

    if (code != 200) {
        Serial.print("[TimezoneSync] HTTP error: ");
        Serial.println(code);
        http.end();
        return false;
    }

    String response = http.getString();
    http.end();

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, response);

    if (err) {
        Serial.print("[TimezoneSync] JSON error: ");
        Serial.println(err.c_str());
        return false;
    }

    if (doc["offset"].isNull()) {
        Serial.println("[TimezoneSync] no offset in response");
        return false;
    }

    offsetSec = doc["offset"].as<long>();
    synced = true;

    Serial.print("[TimezoneSync] offset: ");
    Serial.print(offsetSec);
    Serial.println(" sec");

    return true;
}

long TimezoneSync::getOffsetSec() const {
    return offsetSec;
}

bool TimezoneSync::isSynced() const {
    return synced;
}