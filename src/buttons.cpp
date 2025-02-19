#include <Arduino.h>
#include "pins.h"
#include "buttons.h"
#include "joystick.h"
#include "imu.h"
static unsigned long lastIMUButtonPress = 0;
static unsigned long lastJoystickButtonPress = 0;

void setupButtons(void) {
    pinMode(IMU_CALIBRATION_PIN, INPUT);
    pinMode(JOYSTICK_CALIBRATION_PIN, INPUT);
}

bool imuPressed(void) {
    if (digitalRead(IMU_CALIBRATION_PIN) == HIGH) {
        lastIMUButtonPress = millis();
    }
   if(millis() - lastIMUButtonPress > IMU_BUTTON_PRESS_INTERVAL) {
    calibrateIMU();
    return true;
   }
   return false;
}


bool joystickPressed(void) {
    if (digitalRead(JOYSTICK_CALIBRATION_PIN) == HIGH) {
        lastJoystickButtonPress = millis();
    }
    return (millis() - lastJoystickButtonPress) > JOYSTICK_BUTTON_PRESS_INTERVAL;
}