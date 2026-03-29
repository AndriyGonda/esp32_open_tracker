#include "ConfigPortal.h"

ConfigPortal::ConfigPortal(AppSettings& settings, uint16_t port)
    : settings(settings), server(port) {
}

void ConfigPortal::begin() {
    setupRoutes();
}

void ConfigPortal::loop() {
    if (active) {
        server.handleClient();
    }
}

bool ConfigPortal::isActive() const {
    return active;
}

void ConfigPortal::start() {
    if (active) {
        return;
    }

    Serial.println("Starting config portal...");

    server.stop();

    WiFi.persistent(false);
    WiFi.setSleep(false);

    WiFi.mode(WIFI_MODE_NULL);
    delay(300);

    WiFi.mode(WIFI_AP_STA);
    delay(300);

    bool ok = WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
    if (!ok) {
        Serial.println("Failed to start SoftAP");
        return;
    }

    delay(500);

    Serial.print("AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.begin();
    active = true;
}

void ConfigPortal::stop() {
    if (!active) {
        return;
    }

    server.stop();
    WiFi.softAPdisconnect(true);
    delay(300);

    WiFi.mode(WIFI_MODE_NULL);
    delay(300);

    active = false;
}

void ConfigPortal::setupRoutes() {
    server.on("/", HTTP_GET, [this]() { handleRoot(); });
    server.on("/saveServer", HTTP_POST, [this]() { handleSaveServer(); });
    server.on("/addWifi", HTTP_POST, [this]() { handleAddWifi(); });
    server.on("/deleteWifi", HTTP_POST, [this]() { handleDeleteWifi(); });
    server.on("/scan", HTTP_POST, [this]() { handleScan(); });
    server.onNotFound([this]() { handleNotFound(); });
}

String ConfigPortal::htmlEscape(const String& s) {
    String out;
    out.reserve(s.length());

    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&#39;"; break;
            default: out += c; break;
        }
    }

    return out;
}

String ConfigPortal::buildRootPage() {
    String html;
    html.reserve(12000);

    html += "<!doctype html><html><head><meta charset='utf-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>ESP Nav Setup</title>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;max-width:900px;margin:20px auto;padding:0 12px;background:#f4f6f8;color:#222;}";
    html += ".card{background:#fff;padding:16px;border-radius:14px;margin-bottom:16px;box-shadow:0 2px 10px rgba(0,0,0,0.08);}";
    html += "input,button{padding:10px;margin:6px 0;width:100%;box-sizing:border-box;}";
    html += "button{cursor:pointer;border:none;border-radius:10px;}";
    html += ".ok{background:#198754;color:#fff;}";
    html += ".danger{background:#d9534f;color:#fff;}";
    html += ".muted{color:#666;font-size:13px;}";
    html += "table{width:100%;border-collapse:collapse;}";
    html += "th,td{padding:10px;border-bottom:1px solid #ddd;text-align:left;vertical-align:middle;}";
    html += "form{margin:0;}";
    html += "</style></head><body>";

    html += "<div class='card'>";
    html += "<h2>ESP Nav configuration</h2>";
    html += "<div class='muted'>Long press button on GPIO 10 to open this page.</div>";
    html += "<p><b>AP:</b> ESP Nav</p>";
    html += "<p><b>Priority:</b> networks are tried in the same order as they were added.</p>";
    html += "</div>";

    html += "<div class='card'>";
    html += "<h3>Navigation server</h3>";
    html += "<form method='POST' action='/saveServer'>";
    html += "<label>Host / IP</label>";
    html += "<input name='host' maxlength='128' value='" + htmlEscape(settings.getServerHost()) + "'>";
    html += "<label>Port</label>";
    html += "<input name='port' type='number' min='1' max='65535' value='" + String(settings.getServerPort()) + "'>";
    html += "<button class='ok' type='submit'>Save server settings</button>";
    html += "</form>";
    html += "</div>";

    html += "<div class='card'>";
    html += "<h3>Add WiFi network</h3>";

    if (settings.getWifiCount() >= AppSettings::MAX_WIFI) {
        html += "<p><b>Maximum reached:</b> 10 networks</p>";
    } else {
        html += "<form method='POST' action='/addWifi'>";
        html += "<label>SSID</label>";
        html += "<input name='ssid' maxlength='32' required>";
        html += "<label>Password</label>";
        html += "<input name='pass' maxlength='64' type='password'>";
        html += "<button class='ok' type='submit'>Add WiFi</button>";
        html += "</form>";
    }

    html += "</div>";

    html += "<div class='card'>";
    html += "<h3>Saved WiFi networks</h3>";

    if (settings.getWifiCount() == 0) {
        html += "<p>No saved networks.</p>";
    } else {
        html += "<table>";
        html += "<tr><th>#</th><th>SSID</th><th>Action</th></tr>";

        for (uint8_t i = 0; i < settings.getWifiCount(); i++) {
            const auto& wifi = settings.getWifi(i);

            html += "<tr>";
            html += "<td>" + String(i + 1) + "</td>";
            html += "<td>" + htmlEscape(wifi.ssid) + "</td>";
            html += "<td>";
            html += "<form method='POST' action='/deleteWifi'>";
            html += "<input type='hidden' name='index' value='" + String(i) + "'>";
            html += "<button class='danger' type='submit'>Delete</button>";
            html += "</form>";
            html += "</td>";
            html += "</tr>";
        }

        html += "</table>";
    }

    html += "</div>";

    html += "<div class='card'>";
    html += "<h3>Nearby WiFi scan</h3>";
    html += "<form method='POST' action='/scan'>";
    html += "<button type='submit'>Scan nearby networks</button>";
    html += "</form>";
    html += "</div>";

    html += "</body></html>";
    return html;
}

String ConfigPortal::buildScanPage() {
    String html;
    html.reserve(8000);

    html += "<!doctype html><html><head><meta charset='utf-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>WiFi Scan</title>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;max-width:900px;margin:20px auto;padding:0 12px;background:#f4f6f8;color:#222;}";
    html += ".card{background:#fff;padding:16px;border-radius:14px;margin-bottom:16px;box-shadow:0 2px 10px rgba(0,0,0,0.08);}";
    html += "table{width:100%;border-collapse:collapse;}";
    html += "th,td{padding:10px;border-bottom:1px solid #ddd;text-align:left;}";
    html += "a{display:inline-block;margin-top:12px;}";
    html += "</style></head><body>";
    html += "<div class='card'><h2>Scan results</h2>";

    int count = WiFi.scanNetworks();
    if (count <= 0) {
        html += "<p>No networks found.</p>";
    } else {
        html += "<table><tr><th>SSID</th><th>RSSI</th><th>Encryption</th></tr>";

        for (int i = 0; i < count; i++) {
            String encryption = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Protected";

            html += "<tr>";
            html += "<td>" + htmlEscape(WiFi.SSID(i)) + "</td>";
            html += "<td>" + String(WiFi.RSSI(i)) + "</td>";
            html += "<td>" + encryption + "</td>";
            html += "</tr>";
        }

        html += "</table>";
    }

    WiFi.scanDelete();

    html += "<a href='/'>Back</a>";
    html += "</div></body></html>";

    return html;
}

void ConfigPortal::handleRoot() {
    server.send(200, "text/html; charset=utf-8", buildRootPage());
}

void ConfigPortal::handleSaveServer() {
    String host = server.arg("host");
    host.trim();

    int port = server.arg("port").toInt();
    if (port <= 0 || port > 65535) {
        port = 5055;
    }

    settings.setServerHost(host.isEmpty() ? "hw.m2m.eu" : host);
    settings.setServerPort((uint16_t)port);
    settings.save();

    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "Saved");
}

void ConfigPortal::handleAddWifi() {
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    ssid.trim();

    if (!settings.addOrUpdateWifi(ssid, pass)) {
        server.send(400, "text/plain", "Could not save WiFi");
        return;
    }

    settings.save();

    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "Added");
}

void ConfigPortal::handleDeleteWifi() {
    int index = server.arg("index").toInt();

    if (!settings.removeWifi((uint8_t)index)) {
        server.send(400, "text/plain", "Invalid index");
        return;
    }

    settings.save();

    server.sendHeader("Location", "/", true);
    server.send(303, "text/plain", "Deleted");
}

void ConfigPortal::handleScan() {
    server.send(200, "text/html; charset=utf-8", buildScanPage());
}

void ConfigPortal::handleNotFound() {
    server.send(404, "text/plain", "Not found");
}