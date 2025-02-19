#include <Arduino.h>
#include "pins.h"
#include "joystick.h"

int calibZeroX = 1024/2;
int calibZeroY = 1024/2;

// Setup the joystick pins
void setupJoystick(void) {
    pinMode(STICK_X, INPUT);
    pinMode(STICK_Y, INPUT);
}

// Read the joystick X axis
int readJoystickX(void) {
    // Read raw value and center it around zero
    int rawValue = analogRead(STICK_X) - calibZeroX;
    
    // Map to a -512 to +512 range
    // Positive values for right movement, negative for left
    if (rawValue > 0) {
        return map(rawValue, 0, 1023 - calibZeroX, 0, 512);
    } else {
        return map(rawValue, -calibZeroX, 0, -512, 0);
    }
}

// Read the joystick Y axis
int readJoystickY(void) {
    // Read raw value and center it around zero
    int rawValue = analogRead(STICK_Y) - calibZeroY;
    
    // Map to a -512 to +512 range
    // Positive values for up movement, negative for down
    if (rawValue > 0) {
        return map(rawValue, 0, 1023 - calibZeroY, 0, 512);
    } else {
        return map(rawValue, -calibZeroY, 0, -512, 0);
    }
}

// Calibrate the joystick
void calibrateJoystick(void) {
    calibZeroX = analogRead(STICK_X);
    calibZeroY = analogRead(STICK_Y);
}
