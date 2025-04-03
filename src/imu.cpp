/**
 * @file imu.cpp
 * @brief Implementation of IMU (MPU6050) functions for the Nerf Robot Controller
 * 
 * This file implements the IMU control functions defined in imu.h,
 * including sensor initialization, reading, and calibration.
 */

#include "imu.h"
#include "Wire.h"
#include "pins.h"
#include <Adafruit_MPU6050.h>
#include <algorithm>

// Initialize the IMU instance
Adafruit_MPU6050 imu;

// Calibration offsets for IMU zero position
static float zeroX = 0;
static float zeroY = 0;

// Moving average filter parameters
const int FILTER_SIZE = 5;  // Number of samples for moving average
static float xBuffer[FILTER_SIZE] = {0};
static float yBuffer[FILTER_SIZE] = {0};
static float zBuffer[FILTER_SIZE] = {0};
static int bufferIndex = 0;

/**
 * @brief Displays detailed information about the MPU6050 sensor configuration
 * 
 * Prints sensor specifications including range and resolution to the Serial monitor.
 */
void displaySensorDetails(void) {
    Serial.println("------------------------------------");
    Serial.println("MPU6050 ACCELEROMETER");
    Serial.println("------------------------------------");
    Serial.println("Range: ±2g");
    Serial.println("Resolution: 16-bit");
    Serial.println("------------------------------------");
}

/**
 * @brief Initializes the MPU6050 sensor with appropriate settings
 * 
 * Configures the sensor with:
 * - High-pass filter
 * - Motion detection threshold
 * - Motion detection duration
 * - Interrupt settings
 * 
 * @return true if initialization was successful, false otherwise
 */
bool initializeIMU(void) {
    if (!imu.begin()) {
        return false;
    }
    
    // Configure the MPU6050
    imu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    imu.setMotionDetectionThreshold(1);
    imu.setMotionDetectionDuration(20);
    imu.setInterruptPinLatch(true);
    imu.setInterruptPinPolarity(true);
    imu.setMotionInterrupt(true);
    
    return true;
}

/**
 * @brief Clamps a value between a minimum and maximum range
 * @param value The value to clamp
 * @param min The minimum allowed value
 * @param max The maximum allowed value
 * @return The clamped value
 */
float clamp(float value, float min, float max) {
    return std::min(std::max(value, min), max);
}

/**
 * @brief Maps IMU acceleration data to a normalized range
 * @param tiltX Reference to X-axis acceleration
 * @param tiltY Reference to Y-axis acceleration
 * @param tiltZ Reference to Z-axis acceleration
 */
void mapIMUData(float& tiltX, float& tiltY, float& tiltZ) {
    tiltX = clamp(map(tiltX, zeroX-9.8, zeroX+9.8, -1, 1), -1, 1);
    tiltY = clamp(map(tiltY, zeroY-9.8, zeroY+9.8, -1, 1), -1, 1);
}

/**
 * @brief Applies a moving average filter to the IMU data
 * @param newX New X-axis value
 * @param newY New Y-axis value
 * @param newZ New Z-axis value
 * @param filteredX Reference to store filtered X-axis value
 * @param filteredY Reference to store filtered Y-axis value
 * @param filteredZ Reference to store filtered Z-axis value
 */
void applyMovingAverage(float newX, float newY, float newZ, float& filteredX, float& filteredY, float& filteredZ) {
    // Add new values to buffers
    xBuffer[bufferIndex] = newX;
    yBuffer[bufferIndex] = newY;
    zBuffer[bufferIndex] = newZ;
    
    // Calculate moving averages
    float sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sumX += xBuffer[i];
        sumY += yBuffer[i];
        sumZ += zBuffer[i];
    }
    
    filteredX = sumX / FILTER_SIZE;
    filteredY = sumY / FILTER_SIZE;
    filteredZ = sumZ / FILTER_SIZE;
    
    // Update buffer index
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
}

/**
 * @brief Reads acceleration data from the IMU
 * 
 * Reads raw acceleration data from the MPU6050 sensor and stores it
 * in the provided references. The data can then be processed using mapIMUData()
 * to get normalized values.
 * 
 * @param tiltX Reference to store X-axis acceleration
 * @param tiltY Reference to store Y-axis acceleration
 * @param tiltZ Reference to store Z-axis acceleration
 */
void readIMUData(float& tiltX, float& tiltY, float& tiltZ) {
    if(shouldntReadIMU()) return;
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);

    // Apply moving average filter to raw data
    applyMovingAverage(a.acceleration.x, a.acceleration.y, a.acceleration.z, tiltX, tiltY, tiltZ);
    // mapIMUData(tiltX, tiltY, tiltZ);
} 

/**
 * @brief Calibrates the IMU by reading and storing the zero position
 * 
 * This function should be called when the IMU is in a known stable position
 * to establish the reference point for future readings.
 */
void calibrateIMU(void) {
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);
    zeroX = a.acceleration.x;
    zeroY = a.acceleration.y;
}
