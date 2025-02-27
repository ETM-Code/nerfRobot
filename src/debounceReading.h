#ifndef DEBOUNCE_READING_H
#define DEBOUNCE_READING_H

#include <Arduino.h>

constexpr unsigned long DEBOUNCE_TIME = 10;  // More efficient than #define

namespace debounce {
    static unsigned long lastJoystickReading = 0;
    static unsigned long lastIMUReading = 0;
    static unsigned long lastButtonReading = 0;

    inline bool shouldntRead(unsigned long &lastReading) {
        unsigned long currentTime = millis();
        if (currentTime - lastReading > DEBOUNCE_TIME) {
            lastReading = currentTime;
            return false;
        }
        return true;
    }

    inline bool shouldntReadJoystick() { return shouldntRead(lastJoystickReading); }
    inline bool shouldntReadIMU() { return shouldntRead(lastIMUReading); }
    inline bool shouldntReadButtons() { return shouldntRead(lastButtonReading); }
}

#endif  // DEBOUNCE_READING_H
