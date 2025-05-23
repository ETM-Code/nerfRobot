#include <Arduino.h>
#include "pins.h"
#include "motors.h"
#include <math.h>

bool validTilt(float tilt){
    return abs(tilt) > 0.3;
}

void activateMotors() {
    digitalWrite(MOTOR_ENABLE_RIGHT_PIN, HIGH);
    digitalWrite(MOTOR_ENABLE_LEFT_PIN, HIGH);
}

void deactivateMotors() {
    digitalWrite(MOTOR_ENABLE_RIGHT_PIN, LOW);
    digitalWrite(MOTOR_ENABLE_LEFT_PIN, LOW);
}

void setMotorSpeeds(float tiltX, float tiltY) {
    //Y axis controls the speed of the motors
    //X axis controls speed distribution
    if(validTilt(tiltY)) {
        float speed = abs(tiltY) * 127;
        float leftSpeed = speed * (validTilt(tiltX)?1:(1 + tiltX));
        float rightSpeed = speed * (validTilt(tiltX)?1:(1 - tiltX));
        analogWrite(MOTOR_LEFT_PIN, leftSpeed);
        analogWrite(MOTOR_RIGHT_PIN, rightSpeed);
    }
}

void setupMotors() {
    pinMode(MOTOR_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE_RIGHT_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE_LEFT_PIN, OUTPUT);
    activateMotors();
}

