#ifndef PINS_H
#define PINS_H

//Servos
#define SERVO1_PIN 2
#define SERVO2_PIN 3

//DC Motors
#define MOTOR_RIGHT_PIN A2
#define MOTOR_LEFT_PIN A1
#define MOTOR_ENABLE_RIGHT_PIN 11
#define MOTOR_ENABLE_LEFT_PIN 10
#define MOTOR_PWM_PIN 9
//IMU - Using Arduino Nano ESP32's I2C pins
#define SDA_PIN A4
#define SCL_PIN A5

//Joystick
#define STICK_X A6
#define STICK_Y A3

//Calibration Pins
#define IMU_CALIBRATION_PIN 7
#define JOYSTICK_CALIBRATION_PIN 12

#endif // PINS_H