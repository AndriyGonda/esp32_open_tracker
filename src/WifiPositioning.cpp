#include "WifiPositioning.h"

WifiPosition WifiPositioning::locate() {
    WifiPosition result = {0.0, 0.0, 0.0, false};

    int found = WiFi.scanNetworks(false, true);

    if (found <= 0) {
        Serial.println("[WifiPositioning] no networks found");
        WiFi.scanDelete();
        return result;
    }

    Serial.print("[WifiPositioning] found networks: ");
    Serial.println(found);

    // build JSON
    String body = "{\"wifiAccessPoints\":[";
    int count = min(found, (int)MAX_SCAN_RESULTS);

    for (int i = 0; i < count; i++) {
        if (i > 0) body += ",";
        body += "{\"macAddress\":\"";
        body += WiFi.BSSIDstr(i);
        body += "\",\"signalStrength\":";
        body += WiFi.RSSI(i);
        body += "}";
    }
    body += "]}";

    WiFi.scanDelete();

    Serial.print("[WifiPositioning] request: ");
    Serial.println(body);

    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.begin(API_URL);
    http.addHeader("Content-Type", "application/json");

    int code = http.POST(body);

    if (code != 200) {
        Serial.print("[WifiPositioning] HTTP error: ");
        Serial.println(code);
        http.end();
        return result;
    }

    String response = http.getString();
    http.end();

    Serial.print("[WifiPositioning] response: ");
    Serial.println(response);

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, response);

    if (err) {
        Serial.print("[WifiPositioning] JSON error: ");
        Serial.println(err.c_str());
        return result;
    }

    if (!doc["location"]["lat"].isNull()) {
        result.lat      = doc["location"]["lat"].as<double>();
        result.lng      = doc["location"]["lng"].as<double>();
        result.accuracy = doc["accuracy"].as<float>();
        result.valid    = true;

        Serial.print("[WifiPositioning] lat=");
        Serial.print(result.lat, 6);
        Serial.print(" lng=");
        Serial.print(result.lng, 6);
        Serial.print(" accuracy=");
        Serial.println(result.accuracy);
    }

    return result;
}