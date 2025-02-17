#include "imu.h"
#include "Wire.h"
#include "pins.h"

// Initialize the IMU instance
LIS331 accelerometer;

void displaySensorDetails(void) {
    Serial.println("------------------------------------");
    Serial.println("H3LIS331DL ACCELEROMETER");
    Serial.println("------------------------------------");
    Serial.println("Range: ±100g");  // Updated to match our configuration
    Serial.println("Resolution: 12-bit");
    Serial.println("------------------------------------");
}

bool initializeIMU(void) {
    // First, initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); // Set to 100kHz I2C frequency
    
    // Set I2C address BEFORE begin()
    accelerometer.setI2CAddr(0x19); // Default I2C address for H3LIS331DL
    
    // Initialize with I2C communication
    // This will:
    // - Set power mode to normal
    // - Enable axes
    // - Set sampling rate to 50Hz
    // - Reset other registers to 0
    accelerometer.begin(LIS331::USE_I2C);
    
    // Set the full scale range to ±100g (LOW_RANGE for H3LIS331DL)
    accelerometer.setFullScale(LIS331::LOW_RANGE);
    
    // Add a small delay to allow the sensor to stabilize
    delay(100);
    
    // Read multiple samples to verify communication and proper operation
    const int numSamples = 5;
    float maxAccel = 0;
    
    for(int i = 0; i < numSamples; i++) {
        int16_t x, y, z;
        accelerometer.readAxes(x, y, z);
        
        // Convert to g's and find the maximum acceleration on any axis
        // Note: Using 100 for convertToG as we're in LOW_RANGE mode (±100g)
        float gx = abs(accelerometer.convertToG(100, x));
        float gy = abs(accelerometer.convertToG(100, y));
        float gz = abs(accelerometer.convertToG(100, z));
        
        Serial.print("Raw readings - X: "); Serial.print(x);
        Serial.print(" Y: "); Serial.print(y);
        Serial.print(" Z: "); Serial.println(z);
        
        Serial.print("G readings - X: "); Serial.print(gx);
        Serial.print("g Y: "); Serial.print(gy);
        Serial.print("g Z: "); Serial.println(gz);
        
        maxAccel = max(maxAccel, max(max(gx, gy), gz));
        delay(20); // Slightly longer delay between readings
    }
    
    // When stationary, at least one axis should measure close to 1g (gravity)
    if (maxAccel < 0.5 || maxAccel > 1.5) {
        Serial.println("Failed to initialize H3LIS331DL! Invalid readings detected.");
        Serial.print("Maximum acceleration measured: ");
        Serial.print(maxAccel);
        Serial.println("g");
        Serial.println("Expected approximately 1g due to gravity.");
        Serial.println("Please check:");
        Serial.println("- Power connections (3.3V and GND)");
        Serial.println("- I2C connections (SDA and SCL)");
        Serial.println("- Correct I2C address (0x19)");
        
        // Add I2C scanning
        Serial.println("\nScanning I2C bus...");
        byte error, address;
        int nDevices = 0;
        
        for(address = 1; address < 127; address++) {
            Wire.beginTransmission(address);
            error = Wire.endTransmission();
            
            if (error == 0) {
                Serial.print("I2C device found at address 0x");
                if (address < 16) {
                    Serial.print("0");
                }
                Serial.println(address, HEX);
                nDevices++;
            }
        }
        
        if (nDevices == 0) {
            Serial.println("No I2C devices found!");
        }
        
        return false;
    }
    
    displaySensorDetails();
    return true;
}

void readIMUData(float& tiltX, float& tiltY, float& tiltZ, 
                 float& magX, float& magY, float& magZ) {
    // Read raw values
    int16_t x, y, z;
    accelerometer.readAxes(x, y, z);
    
    // Convert raw values to g's using the library's conversion function
    // Using 100 for convertToG as we're in LOW_RANGE mode (±100g)
    tiltX = accelerometer.convertToG(100, x);
    tiltY = accelerometer.convertToG(100, y);
    tiltZ = accelerometer.convertToG(100, z);
    
    // Set magnetic values to 0 as this accelerometer doesn't have magnetometer
    magX = 0;
    magY = 0;
    magZ = 0;
} 