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
#define SDA_PIN A4  // GPIO 8
#define SCL_PIN A5  // GPIO 9

//Joystick
#define STICK_X A7
#define STICK_Y A3

//Calibration Pins
#define CALIBRATION_PIN 7


#endif // PINS_H
