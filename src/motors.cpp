/**
 * @file motors.cpp
 * @brief Implementation of motor control functions for the Nerf Robot Controller
 * 
 * This file implements the motor control functions defined in motors.h,
 * including PWM control and motor speed calculations.
 */

#include <Arduino.h>
#include "pins.h"
#include "motors.h"
#include <math.h>

/**
 * @brief Checks if a tilt value is significant enough to warrant motor movement
 * @param tilt The tilt value to check
 * @return true if the tilt value exceeds the threshold, false otherwise
 */
bool validTilt(float tilt){
    return abs(tilt) > 0.3;
}

/**
 * @brief Activates the motor enable pins to allow motor movement
 */
void activateMotors() {
    digitalWrite(MOTOR_ENABLE_RIGHT_PIN, HIGH);
    digitalWrite(MOTOR_ENABLE_LEFT_PIN, HIGH);
}

/**
 * @brief Deactivates the motor enable pins to stop motor movement
 */
void deactivateMotors() {
    digitalWrite(MOTOR_ENABLE_RIGHT_PIN, LOW);
    digitalWrite(MOTOR_ENABLE_LEFT_PIN, LOW);
}

/**
 * @brief Sets the speed and direction of both motors based on tilt values
 * 
 * The function uses tiltX and tiltY to determine motor speeds:
 * - tiltY controls the overall speed (forward/backward)
 * - tiltX controls the speed distribution between left and right motors
 * 
 * @param tiltX Tilt value for X-axis (controls left/right movement)
 * @param tiltY Tilt value for Y-axis (controls forward/backward movement)
 */
void setMotorSpeeds(float tiltX, float tiltY) {
    // Y axis controls the speed of the motors
    // X axis controls speed distribution
    if(validTilt(tiltY)) {
        float speed = abs(tiltY) * 127;  // Scale to PWM range (0-255)
        float leftSpeed = speed * (validTilt(tiltX)?1:(1 + tiltX));
        float rightSpeed = speed * (validTilt(tiltX)?1:(1 - tiltX));
        analogWrite(MOTOR_LEFT_PIN, leftSpeed);
        analogWrite(MOTOR_RIGHT_PIN, rightSpeed);
    }
    analogWrite(MOTOR_LEFT_PIN, 255);
    analogWrite(MOTOR_RIGHT_PIN, 255);
}

/**
 * @brief Initializes the motor control pins and sets up PWM
 */
void setupMotors() {
    pinMode(MOTOR_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE_LEFT_PIN, OUTPUT);
    activateMotors();
}

