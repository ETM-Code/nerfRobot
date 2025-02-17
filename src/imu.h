#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_LIS331.h>

// Global IMU instance
extern LIS331 accelerometer;

// Function declarations
void displaySensorDetails(void);
bool initializeIMU(void);
void readIMUData(float& tiltX, float& tiltY, float& tiltZ, 
                 float& magX, float& magY, float& magZ);

#endif // IMU_H 