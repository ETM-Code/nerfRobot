#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_FXOS8700.h>
#include <algorithm>

// Global IMU instance
extern Adafruit_FXOS8700 imu;

// Function declarations
void displaySensorDetails(void);
bool initializeIMU(void);
void readIMUData(float& tiltX, float& tiltY, float& tiltZ);
void calibrateIMU(void);
bool validTilt(float tilt);

#endif // IMU_H 