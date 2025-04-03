/**
 * @file joystick.cpp
 * @brief Implementation of joystick control functions for the Nerf Robot Controller
 * 
 * This file implements the joystick control functions defined in joystick.h,
 * including analog reading, calibration, and value processing.
 */

#include <Arduino.h>
#include "pins.h"
#include "joystick.h"
#include "debounceReading.h"

// Calibration values for joystick center position
int calibZeroX = 1024/2;  // Default center position for X-axis
int calibZeroY = 1024/2;  // Default center position for Y-axis

/**
 * @brief Initializes the joystick analog input pins
 */
void setupJoystick(void) {
    pinMode(STICK_X, INPUT);
    pinMode(STICK_Y, INPUT);
}

/**
 * @brief Reads and processes the X-axis value from the joystick
 * 
 * The function:
 * 1. Reads the raw analog value
 * 2. Maps it to a centered range (-2048 to +2048)
 * 3. Scales it to a normalized range (-1.0 to 1.0)
 * 
 * @return Processed X-axis value (-1.0 to 1.0)
 */
float readJoystickX(void) {
    // Read raw value and center it around zero
    if(shouldntReadJoystick()) return 800000;
    int rawValue = map(analogRead(STICK_X), 0, 4095, 4095, 0) - calibZeroX;
    // Serial.println(analogRead(STICK_X));
    
    // Map to a -512 to +512 range
    // Positive values for right movement, negative for left
    if (rawValue > 0) {
        return map(rawValue, 0, 4095 - calibZeroX, 0, 2048)/2048.00f;
    } else {
        return map(rawValue, -calibZeroX, 0, -2048, 0)/2048.00f;
    }
}

/**
 * @brief Reads and processes the Y-axis value from the joystick
 * 
 * The function:
 * 1. Reads the raw analog value
 * 2. Maps it to a centered range (-2048 to +2048)
 * 3. Scales it to a normalized range (-1.0 to 1.0)
 * 
 * @return Processed Y-axis value (-1.0 to 1.0)
 */
float readJoystickY(void) {
    // Read raw value and center it around zero
    if(shouldntReadJoystick()) return 800000;
    int rawValue = map(analogRead(STICK_Y), 0, 4095, 4095, 0) - calibZeroY;
    // Serial.println(analogRead(STICK_Y));
    
    // Map to a -1 to +1 range
    // Positive values for up movement, negative for down
    if (rawValue > 0) {
        return map(rawValue, 0, 4095 - calibZeroY, 0, 2048)/2048.00f;
    } else {
        return map(rawValue, -calibZeroY, 0, -2048, 0)/2048.00f;
    }
}

/**
 * @brief Calibrates the joystick by reading and storing the center position
 * 
 * This function should be called when the joystick is in its neutral position
 * to ensure accurate readings during operation.
 */
void calibrateJoystick(void) {
    calibZeroX = analogRead(STICK_X);
    calibZeroY = analogRead(STICK_Y);
}
