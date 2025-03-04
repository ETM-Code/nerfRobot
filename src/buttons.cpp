#include <Arduino.h>
#include "pins.h"
#include "buttons.h"
#include "joystick.h"
#include "imu.h"
static unsigned long lastIMUButtonPress = 0;
static unsigned long lastJoystickButtonPress = 0;
const bool unifiedCalibration = true;

void setupButtons(void) {
    pinMode(IMU_CALIBRATION_PIN, INPUT_PULLUP);
    pinMode(JOYSTICK_CALIBRATION_PIN, INPUT_PULLUP);
}

void uniCalib(void) {
    Serial.println("Unifying Calibration");
    calibrateIMU();
    calibrateJoystick();
}

bool imuPressed(void) {
    if (digitalRead(IMU_CALIBRATION_PIN) == HIGH) {
        lastIMUButtonPress = millis();
    }
   if(millis() - lastIMUButtonPress > IMU_BUTTON_PRESS_INTERVAL) {
    if(unifiedCalibration) {
        uniCalib();
    } else {
        calibrateIMU();
    }
    return true;
   }
   return false;
}


bool joystickPressed(void) {
    if (digitalRead(JOYSTICK_CALIBRATION_PIN) == HIGH) {
        lastJoystickButtonPress = millis();
    }
    if(millis() - lastJoystickButtonPress > JOYSTICK_BUTTON_PRESS_INTERVAL) {
        if(unifiedCalibration) {
            uniCalib();
        } else {
            calibrateJoystick();
        }
        return true;
    }
   return false;
}

