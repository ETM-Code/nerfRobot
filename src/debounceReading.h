#ifndef DEBOUNCE_READING_H
#define DEBOUNCE_READING_H
#define DEBOUNCE_TIME 10

#include <Arduino.h>

static unsigned long lastJoystickReading = 0;
static unsigned long lastIMUReading = 0;
static unsigned long lastButtonReading = 0;
inline bool shouldntReadJoystick(void){
    unsigned long currentTime = millis();
    if(currentTime - lastJoystickReading > DEBOUNCE_TIME){
        lastJoystickReading = currentTime;
        return false;
    }
    return true;
}

inline bool shouldntReadIMU(void){
    unsigned long currentTime = millis();
    if(currentTime - lastIMUReading > DEBOUNCE_TIME){
        lastIMUReading = currentTime;
        return false;
    }
    return true;
}

inline bool shouldntReadButtons(void){
    unsigned long currentTime = millis();
    if(currentTime - lastButtonReading > DEBOUNCE_TIME){
        lastButtonReading = currentTime;
        return false;
    }
    return true;
}
#endif
