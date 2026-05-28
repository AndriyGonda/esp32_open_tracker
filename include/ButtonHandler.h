#pragma once

#include <Arduino.h>

class ButtonHandler {
public:
    ButtonHandler(uint8_t pin, unsigned long longPressMs);

    void begin();
    bool isLongPressTriggered();

private:
    uint8_t pin;
    unsigned long longPressMs;
    unsigned long pressStart = 0;
    bool longHandled = false;
};