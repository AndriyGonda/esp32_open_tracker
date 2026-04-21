#pragma once

#include <Arduino.h>

class LedController {
public:
    explicit LedController(uint8_t pin);

    void begin();
    void on();
    void off();
    void blink(uint8_t times, uint16_t onMs = 120, uint16_t offMs = 120);
    void update();

private:
    uint8_t pin;

    uint8_t blinkTotal = 0;
    uint8_t blinkCount = 0;
    uint16_t blinkOnMs = 0;
    uint16_t blinkOffMs = 0;
    unsigned long lastToggleAt = 0;
    bool blinkState = false;
};