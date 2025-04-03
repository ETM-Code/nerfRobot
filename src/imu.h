/**
 * @file imu.h
 * @brief IMU (MPU6050) interface for the Nerf Robot Controller
 * 
 * This file contains function declarations for handling the MPU6050 IMU sensor,
 * including initialization, reading, and calibration functions.
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "debounceReading.h"

// Global IMU instance
extern Adafruit_MPU6050 imu;

/**
 * @brief Displays detailed information about the MPU6050 sensor configuration
 */
void displaySensorDetails(void);

/**
 * @brief Initializes the MPU6050 sensor with appropriate settings
 * @return true if initialization was successful, false otherwise
 */
bool initializeIMU(void);

/**
 * @brief Reads acceleration data from the IMU
 * @param tiltX Reference to store X-axis acceleration
 * @param tiltY Reference to store Y-axis acceleration
 * @param tiltZ Reference to store Z-axis acceleration
 */
void readIMUData(float& tiltX, float& tiltY, float& tiltZ);

/**
 * @brief Calibrates the IMU by reading and storing the zero position
 */
void calibrateIMU(void);

/**
 * @brief Checks if a tilt value is significant enough to be considered valid
 * @param tilt The tilt value to check
 * @return true if the tilt value exceeds the threshold, false otherwise
 */
bool validTilt(float tilt);

#endif // IMU_H