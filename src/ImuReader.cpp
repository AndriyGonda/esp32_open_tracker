#include "ImuReader.h"

ImuReader::ImuReader(uint8_t sdaPin, uint8_t sclPin)
    : _sdaPin(sdaPin), _sclPin(sclPin) {}

bool ImuReader::begin() {
    Wire.begin(_sdaPin, _sclPin);
    Wire.setClock(100000);
    delay(100);
    Serial.println("[IMU] Wire started");

    Wire.beginTransmission(BMI160_ADDR);
    uint8_t err = Wire.endTransmission();
    Serial.print("[IMU] ping: ");
    Serial.println(err == 0 ? "OK" : "FAIL");
    if (err != 0) return false;

    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    delay(2);
    Wire.requestFrom((uint8_t)BMI160_ADDR, (uint8_t)1);
    if (!Wire.available()) {
        Serial.println("[IMU] chip ID read failed");
        return false;
    }
    uint8_t id = Wire.read();
    Serial.printf("[IMU] chip ID: 0x%02X\n", id);
    if (id != 0xD1) {
        Serial.println("[IMU] unexpected chip ID");
        return false;
    }

    writeReg(0x7E, 0xB6); // soft reset
    delay(200);
    writeReg(0x7E, 0x11); // acc normal mode
    delay(100);

    _ok = true;
    Serial.println("[IMU] BMI160 ready");
    return true;
}

void ImuReader::update() {
    if (!_ok) return;

    unsigned long now = millis();
    if (now - _lastReadMs < READ_INTERVAL_MS) return;
    _lastReadMs = now;

    float ax, ay, az;
    if (!readAccel(ax, ay, az)) return;

    if (!_baselineReady) {
        float alpha = 1.0f / (float)(_baselineCount + 1);
        _gx = _gx * (1.0f - alpha) + ax * alpha;
        _gy = _gy * (1.0f - alpha) + ay * alpha;
        _gz = _gz * (1.0f - alpha) + az * alpha;
        if (++_baselineCount >= BASELINE_SAMPLES) {
            _baselineReady = true;
            Serial.printf("[IMU] baseline ready g=(%.2f,%.2f,%.2f)\n", _gx, _gy, _gz);
        }
        return;
    }

    const float LP = 0.02f;
    _gx = _gx * (1.0f - LP) + ax * LP;
    _gy = _gy * (1.0f - LP) + ay * LP;
    _gz = _gz * (1.0f - LP) + az * LP;

    float dx = ax - _gx, dy = ay - _gy, dz = az - _gz;
    float mag = sqrt(dx*dx + dy*dy + dz*dz);

    _window[_winIdx] = mag;
    _winIdx = (_winIdx + 1) % WIN;
    if (_winIdx == 0) _winFull = true;

    uint8_t total = _winFull ? WIN : _winIdx;
    float sum = 0;
    for (uint8_t i = 0; i < total; i++) sum += _window[i];
    _accelMag = (total > 0) ? sum / (float)total : 0.0f;
    _moving = (_accelMag >= _threshold);
}

bool ImuReader::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool ImuReader::readAccel(float& ax, float& ay, float& az) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(0x12); // ACC_X_L
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)BMI160_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return false;

    int16_t rx = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t ry = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t rz = (int16_t)(Wire.read() | (Wire.read() << 8));

    // ACC_RANGE=0x03 → ±2g → 16384 LSB/g
    const float G = 9.80665f;
    ax = (float)rx / 16384.0f * G;
    ay = (float)ry / 16384.0f * G;
    az = (float)rz / 16384.0f * G;
    return true;
}