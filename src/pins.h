#ifndef PINS_H
#define PINS_H

//Servos
#define SERVO1_PIN 2
#define SERVO2_PIN 3

//DC Motors
#define MOTOR_RIGHT_PIN 9
#define MOTOR_LEFT_PIN 5
#define MOTOR_ENABLE_RIGHT_PIN 11
#define MOTOR_ENABLE_LEFT_PIN 10
//IMU - Using Arduino Nano ESP32's I2C pins
#define SDA_PIN A4
#define SCL_PIN A5

//Joystick
#define STICK_X A7
#define STICK_Y A3

//Calibration Pins
#define IMU_CALIBRATION_PIN 7
#define JOYSTICK_CALIBRATION_PIN 12

#endif // PINS_H