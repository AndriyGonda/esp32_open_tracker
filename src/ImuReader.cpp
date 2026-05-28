#include "ImuReader.h"
#include <esp_sleep.h>
#include <driver/gpio.h>

ImuReader::ImuReader(uint8_t sdaPin, uint8_t sclPin)
    : _sdaPin(sdaPin), _sclPin(sclPin) {}

bool ImuReader::begin() {
    Wire.end();
    delay(100);
    Wire.begin(_sdaPin, _sclPin);
    Wire.setClock(100000);
    delay(200);
    Serial.println("[IMU] Wire started");

    Wire.beginTransmission(BMI160_ADDR);
    uint8_t err = Wire.endTransmission();
    Serial.print("[IMU] ping: ");
    Serial.println(err == 0 ? "OK" : "FAIL");
    if (err != 0) {
        _ok = false;
        _lastRetryMs = millis();
        return false;
    }

    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    delay(2);
    Wire.requestFrom((uint8_t)BMI160_ADDR, (uint8_t)1);
    if (!Wire.available()) {
        Serial.println("[IMU] chip ID read failed");
        _ok = false;
        _lastRetryMs = millis();
        return false;
    }

    uint8_t id = Wire.read();
    Serial.printf("[IMU] chip ID: 0x%02X\n", id);
    if (id != 0xD1) {
        Serial.println("[IMU] unexpected chip ID");
        _ok = false;
        _lastRetryMs = millis();
        return false;
    }

    writeReg(0x7E, 0xB6); // soft reset
    delay(500);
    writeReg(0x7E, 0x11); // accel normal mode
    delay(200);

    _ok = true;
    _zeroCount = 0;
    _reinitCount = 0;
    _baselineReady = false;
    _baselineCount = 0;
    _winIdx = 0;
    _winFull = false;
    _accelMag = 0.0f;
    _moving = false;
    _gx = 0; _gy = 0; _gz = 9.81f;

    Serial.println("[IMU] BMI160 ready");
    return true;
}

void ImuReader::enableMotionInterrupt(uint8_t intPin) {
    if (!_ok) return;

    writeReg(0x53, 0x08); // INT_OUT_CTRL: INT1 active high, push-pull
    writeReg(0x54, 0x00); // INT_LATCH: non-latched
    writeReg(0x55, 0x04); // INT_MAP_0: any-motion to INT1
    writeReg(0x5F, 0x00); // INT_MOTION_0: duration = 1 sample
    writeReg(0x60, 0x14); // INT_MOTION_1: threshold ~100mg
    writeReg(0x50, 0x07); // INT_EN_0: enable any-motion X, Y, Z

    Serial.println("[IMU] motion interrupt enabled on INT1");

    gpio_wakeup_enable((gpio_num_t)intPin, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    Serial.printf("[IMU] gpio wakeup configured on GPIO%d\n", intPin);
}

bool ImuReader::reinit() {
    if (_reinitCount >= MAX_REINIT_COUNT) {
        Serial.println("[IMU] max reinit attempts reached, switching to delayed retry mode");
        _ok = false;
        _lastRetryMs = millis();
        return false;
    }

    _reinitCount++;
    Serial.printf("[IMU] reinit attempt %d/%d\n", _reinitCount, MAX_REINIT_COUNT);

    _zeroCount     = 0;
    _baselineReady = false;
    _baselineCount = 0;
    _winIdx        = 0;
    _winFull       = false;
    _accelMag      = 0.0f;
    _moving        = false;
    _gx = 0; _gy = 0; _gz = 9.81f;

    bool ok = begin();
    if (!ok) {
        Serial.println("[IMU] reinit failed");
    } else {
        Serial.println("[IMU] reinit OK");
    }
    return ok;
}

void ImuReader::update() {
    unsigned long now = millis();

    if (!_ok) {
        if (now - _lastRetryMs >= RETRY_INTERVAL_MS) {
            _lastRetryMs = now;
            Serial.println("[IMU] device offline, retrying init...");
            begin();
        }
        return;
    }

    if (now - _lastReadMs < READ_INTERVAL_MS) return;
    _lastReadMs = now;

    float ax, ay, az;
    if (!readAccel(ax, ay, az)) {
        Serial.println("[IMU] read failed, marking offline");
        _ok = false;
        _accelMag = 0.0f;
        _moving = false;
        _baselineReady = false;
        _lastRetryMs = now;
        return;
    }

    if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
        _zeroCount++;
        Serial.printf("[IMU] zero read %d/%d\n", _zeroCount, MAX_ZERO_READS);

        if (_zeroCount >= MAX_ZERO_READS) {
            Serial.println("[IMU] too many zero reads, marking IMU offline");
            _ok = false;
            _accelMag = 0.0f;
            _moving = false;
            _baselineReady = false;
            _lastRetryMs = now;
        }
        return;
    }

    _zeroCount   = 0;
    _reinitCount = 0;

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
    Wire.write(0x12);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)BMI160_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return false;

    int16_t rx = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t ry = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t rz = (int16_t)(Wire.read() | (Wire.read() << 8));

    const float G = 9.80665f;
    ax = (float)rx / 16384.0f * G;
    ay = (float)ry / 16384.0f * G;
    az = (float)rz / 16384.0f * G;
    return true;
}
