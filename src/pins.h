/**
 * @file pins.h
 * @brief Pin definitions for the Nerf Robot Controller
 * 
 * This file contains all the pin assignments for the various components
 * of the Nerf Robot Controller, including servos, motors, IMU, and buttons.
 */

#ifndef PINS_H
#define PINS_H

// Servo motor pins for turret control
#define SERVO1_PIN 2  // Controls horizontal rotation of the turret
#define SERVO2_PIN 3  // Controls vertical rotation of the turret

// DC Motor pins for robot movement
#define MOTOR_RIGHT_PIN 5      // Right motor control pin
#define MOTOR_LEFT_PIN 7       // Left motor control pin
#define MOTOR_ENABLE_RIGHT_PIN 11  // Right motor enable pin
#define MOTOR_ENABLE_LEFT_PIN 10   // Left motor enable pin

// IMU (MPU6050) I2C pins - Using Arduino Nano ESP32's built-in I2C pins
#define SDA_PIN A4  // I2C Data pin
#define SCL_PIN A5  // I2C Clock pin

// Analog Joystick pins for manual control
#define STICK_X A6  // X-axis of the joystick
#define STICK_Y A3  // Y-axis of the joystick

// Calibration and Control pins
#define IMU_CALIBRATION_PIN 7      // Button for IMU calibration
#define JOYSTICK_CALIBRATION_PIN 12 // Button for joystick calibration
#define FIRE_BUTTON_PIN 13         // Button connected to solenoid for firing mechanism

#endif // PINS_H