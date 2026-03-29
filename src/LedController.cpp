#include "LedController.h"

LedController::LedController(uint8_t pin) : pin(pin) {
}

void LedController::begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void LedController::on() {
    blinkTotal = 0;  
    digitalWrite(pin, HIGH);
}

void LedController::off() {
    blinkTotal = 0;  
    digitalWrite(pin, LOW);
}

void LedController::blink(uint8_t times, uint16_t onMs, uint16_t offMs) {
    blinkTotal  = times;
    blinkCount  = 0;
    blinkOnMs   = onMs;
    blinkOffMs  = offMs;
    blinkState  = false;
    lastToggleAt = millis();
    digitalWrite(pin, LOW);
}

void LedController::update() {
    if (blinkCount >= blinkTotal) {
        return;
    }

    unsigned long now = millis();
    uint16_t interval = blinkState ? blinkOnMs : blinkOffMs;

    if (now - lastToggleAt < interval) {
        return;
    }

    lastToggleAt = now;
    blinkState = !blinkState;
    digitalWrite(pin, blinkState ? HIGH : LOW);

    if (!blinkState) {
        blinkCount++;
    }
}