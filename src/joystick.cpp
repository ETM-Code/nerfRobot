#include <Arduino.h>
#include "pins.h"
#include "joystick.h"
#include "debounceReading.h"

int calibZeroX = 1024/2;
int calibZeroY = 1024/2;

// Setup the joystick pins
void setupJoystick(void) {
    pinMode(STICK_X, INPUT);
    pinMode(STICK_Y, INPUT);
}

// Read the joystick X axis
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

// Read the joystick Y axis
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

// Calibrate the joystick
void calibrateJoystick(void) {
    calibZeroX = analogRead(STICK_X);
    calibZeroY = analogRead(STICK_Y);
}
