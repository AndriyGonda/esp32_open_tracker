#include "LedController.h"

LedController::LedController(uint8_t pin) : pin(pin) {
}

void LedController::begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    blinkTotal = 0;
    blinkCount = 0;
    blinkState = false;
}

void LedController::on() {
    blinkTotal = 0;
    blinkCount = 0;
    blinkState = false;
    digitalWrite(pin, LOW);
}

void LedController::off() {
    blinkTotal = 0;
    blinkCount = 0;
    blinkState = false;
    digitalWrite(pin, HIGH);
}

void LedController::blink(uint8_t times, uint16_t onMs, uint16_t offMs) {
    blinkTotal   = times;
    blinkCount   = 0;
    blinkOnMs    = onMs;
    blinkOffMs   = offMs;
    blinkState   = false;
    lastToggleAt = millis();
    digitalWrite(pin, HIGH);
}

void LedController::update() {
    if (blinkTotal == 0) return;

    if (blinkCount >= blinkTotal) {
        digitalWrite(pin, HIGH);
        blinkTotal = 0;
        blinkCount = 0;
        blinkState = false;
        return;
    }

    unsigned long now = millis();
    uint16_t interval = blinkState ? blinkOnMs : blinkOffMs;

    if (now - lastToggleAt < interval) return;

    lastToggleAt = now;
    blinkState = !blinkState;
    digitalWrite(pin, blinkState ? LOW : HIGH);

    if (!blinkState) {
        blinkCount++;
    }
}