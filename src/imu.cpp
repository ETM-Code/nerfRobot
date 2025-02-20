#include "imu.h"
#include "Wire.h"
#include "pins.h"
#include <Adafruit_MPU6050.h>
#include <algorithm>

// Initialize the IMU instance
Adafruit_MPU6050 imu;
static float zeroX = 0;
static float zeroY = 0;

void displaySensorDetails(void) {
    Serial.println("------------------------------------");
    Serial.println("MPU6050 ACCELEROMETER");
    Serial.println("------------------------------------");
    Serial.println("Range: ±2g");
    Serial.println("Resolution: 16-bit");
    Serial.println("------------------------------------");
}

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

float clamp(float value, float min, float max) {
    return std::min(std::max(value, min), max);
}

void mapIMUData(float& tiltX, float& tiltY, float& tiltZ) {
    tiltX = clamp(map(tiltX, zeroX-9.8, zeroX+9.8, -1, 1), -1, 1);
    tiltY = clamp(map(tiltY, zeroY-9.8, zeroY+9.8, -1, 1), -1, 1);
}

void readIMUData(float& tiltX, float& tiltY, float& tiltZ) {
    // delay(100);
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);

    tiltX = a.acceleration.x;
    // Serial.println(a.acceleration.x);
    tiltY = a.acceleration.y;
    tiltZ = a.acceleration.z;
    // mapIMUData(tiltX, tiltY, tiltZ);
} 

void calibrateIMU(void) {
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);
    zeroX = a.acceleration.x;
    zeroY = a.acceleration.y;
}

