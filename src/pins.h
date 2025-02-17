#ifndef PINS_H
#define PINS_H

//Servos
#define SERVO1_PIN 2
#define SERVO2_PIN 3

//DC Motors
#define MOTOR_RIGHT_PIN A2
#define MOTOR_LEFT_PIN A1

//IMU - Using Arduino Nano ESP32's I2C pins
#define SDA_PIN A4  // GPIO 8
#define SCL_PIN A5  // GPIO 9

//Joystick
#define STICK_X A3
#define STICK_Y A7

//Calibration Pins
#define IMU_CALIBRATION_PIN 7
#define JOYSTICK_CALIBRATION_PIN 12

#endif // PINS_H