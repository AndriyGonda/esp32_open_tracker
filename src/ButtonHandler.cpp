#include "ButtonHandler.h"

ButtonHandler::ButtonHandler(uint8_t pin, unsigned long longPressMs)
    : pin(pin), longPressMs(longPressMs) {
}

void ButtonHandler::begin() {
    pinMode(pin, INPUT_PULLUP);
}

bool ButtonHandler::isLongPressTriggered() {
    bool pressed = digitalRead(pin) == LOW;

    if (pressed) {
        if (pressStart == 0) {
            pressStart = millis();
        }

        if (!longHandled && (millis() - pressStart >= longPressMs)) {
            longHandled = true;
            return true;
        }
    } else {
        pressStart = 0;
        longHandled = false;
    }

    return false;
}