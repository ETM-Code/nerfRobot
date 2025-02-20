#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "debounceReading.h"

// Global IMU instance
extern Adafruit_MPU6050 imu;

// Function declarations
void displaySensorDetails(void);
bool initializeIMU(void);
void readIMUData(float& tiltX, float& tiltY, float& tiltZ);
void calibrateIMU(void);
bool validTilt(float tilt);

#endif // IMU_H