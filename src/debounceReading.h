/**
 * @file debounceReading.h
 * @brief Debouncing functionality for sensor and input readings
 * 
 * This file provides debouncing functionality to prevent reading sensors
 * and inputs too frequently, which filters noise, and prevents bounce on mechanical switches.
 */

#ifndef DEBOUNCE_READING_H
#define DEBOUNCE_READING_H

// Minimum time between readings in milliseconds
#define DEBOUNCE_TIME 10

#include <Arduino.h>

// Timestamps for tracking last reading times
static unsigned long lastJoystickReading = 0;
static unsigned long lastIMUReading = 0;
static unsigned long lastButtonReading = 0;

/**
 * @brief Checks if enough time has passed since the last joystick reading
 * 
 * Implements a debounce delay for joystick readings to prevent noise
 * and ensure stable readings. Also includes a startup delay of 15 seconds.
 * 
 * @return false if reading is allowed, true if reading should be skipped
 */
inline bool shouldntReadJoystick(void){
    unsigned long currentTime = millis();
    if(currentTime - lastJoystickReading > DEBOUNCE_TIME || currentTime < 15000){
        lastJoystickReading = currentTime;
        return false;
    }
    return false;
}

/**
 * @brief Checks if enough time has passed since the last IMU reading
 * 
 * Implements a debounce delay for IMU readings to prevent noise
 * and ensure stable readings.
 * 
 * @return false if reading is allowed, true if reading should be skipped
 */
inline bool shouldntReadIMU(void){
    unsigned long currentTime = millis();
    if(currentTime - lastIMUReading > DEBOUNCE_TIME){
        lastIMUReading = currentTime;
        return false;
    }
    return true;
}

/**
 * @brief Checks if enough time has passed since the last button reading
 * 
 * Implements a debounce delay for button readings to prevent false
 * triggers from mechanical switch bounce.
 * 
 * @return false if reading is allowed, true if reading should be skipped
 */
inline bool shouldntReadButtons(void){
    unsigned long currentTime = millis();
    if(currentTime - lastButtonReading > DEBOUNCE_TIME){
        lastButtonReading = currentTime;
        return false;
    }
    return true;
}
#endif
