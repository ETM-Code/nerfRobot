#include "imu.h"
#include "Wire.h"
#include "pins.h"
#include <Adafruit_FXOS8700.h>
#include <algorithm>

// Initialize the IMU instance
Adafruit_FXOS8700 imu = Adafruit_FXOS8700(0x870000, 0x870001);
static float zeroX = 0;
static float zeroY = 0;

void displaySensorDetails(void) {
    Serial.println("------------------------------------");
    Serial.println("FXOS8700 ACCELEROMETER");
    Serial.println("------------------------------------");
    Serial.println("Range: ±2g");  // Updated to match our configuration
    Serial.println("Resolution: 14-bit");
    Serial.println("------------------------------------");
}

bool initializeIMU(void) {
    return imu.begin();
}

void readIMUData(float& tiltX, float& tiltY, float& tiltZ) {
    sensors_event_t aevent, mevent;
    imu.getEvent(&aevent, &mevent);
    tiltX = aevent.acceleration.x;
    tiltY = aevent.acceleration.y;
    tiltZ = aevent.acceleration.z;
    mapIMUData(tiltX, tiltY, tiltZ);
} 

void mapIMUData(float& tiltX, float& tiltY, float& tiltZ) {
    tiltX = clamp(map(tiltX, zeroX-9.8, zeroX+9.8, -1, 1), -1, 1);
    tiltY = clamp(map(tiltY, zeroY-9.8, zeroY+9.8, -1, 1), -1, 1);
}

void calibrateIMU(void) {
    int16_t x, y, z;
    accelerometer.readAxes(x, y, z);
    zeroX = accelerometer.convertToG(100, x);
    zeroY = accelerometer.convertToG(100, y);
}

float clamp(float value, float min, float max) {
    return std::min(std::max(value, min), max);
}