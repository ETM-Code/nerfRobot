#include <Arduino.h>
#include "pins.h"
#include "buttons.h"
#include "joystick.h"
#include "imu.h"

static unsigned long lastCalibrationPress = 0;

void setupButtons(void) {
    pinMode(CALIBRATION_PIN, INPUT_PULLUP); // Using a single button
}

bool calibrationPressed(void) {
    if (digitalRead(CALIBRATION_PIN) == HIGH) {
        lastCalibrationPress = millis();
    }

    if (millis() - lastCalibrationPress > CALIBRATION_BUTTON_PRESS_INTERVAL) {
        calibrateIMU();       // Call IMU calibration
        calibrateJoystick();  // Call Joystick calibration
        Serial.println("Calibration Complete.");
        return true;
    }
    return false;
}


