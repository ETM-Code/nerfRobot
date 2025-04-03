/**
 * @file buttons.cpp
 * @brief Implementation of button handling functions for the Nerf Robot Controller
 * 
 * This file implements the button handling functions defined in buttons.h,
 * including debouncing logic and calibration triggers.
 */

#include <Arduino.h>
#include "pins.h"
#include "buttons.h"
#include "joystick.h"
#include "imu.h"

// Timestamps for button debouncing
static unsigned long lastIMUButtonPress = 0;
static unsigned long lastJoystickButtonPress = 0;
static unsigned long lastFireButtonPress = 0;

// Flag to enable/disable unified calibration mode
const bool unifiedCalibration = true;

/**
 * @brief Initializes all button pins with pull-up resistors
 */
void setupButtons(void) {
    pinMode(IMU_CALIBRATION_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK_CALIBRATION_PIN, INPUT_PULLUP);
}

/**
 * @brief Performs unified calibration of both IMU and joystick
 * This function is called when unified calibration mode is enabled
 */
void uniCalib(void) {
    Serial.println("Unifying Calibration");
    calibrateIMU();
    calibrateJoystick();
}

/**
 * @brief Checks if the IMU calibration button is pressed and handles calibration
 * @return true if button is pressed and debounced, false otherwise
 */
bool imuPressed(void) {
   if(millis() - lastIMUButtonPress > IMU_BUTTON_PRESS_INTERVAL) {
        if (digitalRead(IMU_CALIBRATION_PIN) == HIGH) {
            lastIMUButtonPress = millis();
        }
        if(unifiedCalibration) {
            uniCalib();
        } else {
            calibrateIMU();
        }
        return true;
   }
   return false;
}

/**
 * @brief Checks if the fire button is pressed
 * @return true if button is pressed and debounced, false otherwise
 */
bool firePressed(void) {
    if(millis() - lastFireButtonPress > FIRE_BUTTON_PRESS_INTERVAL) {
        if (digitalRead(FIRE_BUTTON_PIN) == HIGH) {
            lastFireButtonPress = millis();
            return true;
        }
    }
    return false;
}

/**
 * @brief Checks if the joystick calibration button is pressed and handles calibration
 * @return true if button is pressed and debounced, false otherwise
 */
bool joystickPressed(void) {
    if(millis() - lastJoystickButtonPress > JOYSTICK_BUTTON_PRESS_INTERVAL) {
        if (digitalRead(JOYSTICK_CALIBRATION_PIN) == HIGH) {
            lastJoystickButtonPress = millis();
            if(unifiedCalibration) {
                uniCalib();
            } else {
                calibrateJoystick();
            }
            return true;
        }
    }
   return false;
}

