/**
 * @file main.cpp
 * @brief Main program for the Nerf Robot Controller
 * 
 * This file contains the main program flow for the Nerf Robot Controller,
 * including initialization, main loop, and control logic.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <pins.h>
#include "Wire.h"
#include "debug.h"
#include "websocket.h"
#include "imu.h"
#include "joystick.h"
#include "buttons.h"
#include "motors.h"
#include "debounceReading.h"

// WebSocket server instance and control flags
WebSocketsServer webSocket = WebSocketsServer(81);
bool clientConnected = false;
bool enableLocalControl = false;

// Debug print flags
bool PRINT_TILT_VALUES = false;
bool PRINT_JOYSTICK_VALUES = false;
bool PRINT_ROTATION_VALUES = false;
bool STARTUP_DELAY = false;

// Servo objects for turret control
Servo servo1;  // Controls horizontal rotation
Servo servo2;  // Controls vertical rotation

// Control variables for turret positioning
float rotationX = 90;  // Initial horizontal position
float rotationY = 90;  // Initial vertical position
float accel = 0;       // Current acceleration
float accelBiasX = 0;  // X-axis acceleration bias
float accelBiasY = 0;  // Y-axis acceleration bias

// Timestamps for non-blocking debug prints
unsigned long lastJoystickPrint = 0;
unsigned long lastRotationPrint = 0;
unsigned long lastTiltPrint = 0;
const unsigned long PRINT_INTERVAL = 100; // 100ms interval for debug prints

// Sensor reading variables
float tiltX = 0;  // X-axis tilt from IMU
float tiltY = 0;  // Y-axis tilt from IMU
float tiltZ = 0;  // Z-axis tilt from IMU
float magX = 0;   // X-axis magnetic field
float magY = 0;   // Y-axis magnetic field
float magZ = 0;   // Z-axis magnetic field
float joystickX = 0;  // X-axis joystick value
float joystickY = 0;  // Y-axis joystick value

// Calibration variables
float imuOffsetX = 0;      // IMU X-axis offset
float imuOffsetY = 0;      // IMU Y-axis offset
float joystickOffsetX = 0; // Joystick X-axis offset
float joystickOffsetY = 0; // Joystick Y-axis offset
float joystickScaleX = 1.0; // Joystick X-axis scaling
float joystickScaleY = 1.0; // Joystick Y-axis scaling

/**
 * @brief Initial setup function
 * 
 * Performs all necessary initialization including:
 * - Serial communication
 * - WiFi and WebSocket setup
 * - Servo and motor initialization
 * - IMU and joystick setup
 * - Initial calibration
 */
void setup() {
  if(STARTUP_DELAY) delay(3000);  // Startup delay
  Serial.begin(115200);
  Serial.println("Starting WiFi Robot Controller...");

  // Initialize WiFi and WebSocket
  setupWiFi();
  setupWebSocket();
  
  Serial.println("Robot Controller is ready!");

  // Attach the servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Attach the DC Motors
  setupMotors();

  // Attach the Joystick
  setupJoystick();

  // Setup calibration pins
  setupButtons();

  // Initialize IMU
  if (!initializeIMU()) {
    Serial.println("Failed to initialize IMU. Please check connections:");
    Serial.println("- Ensure IMU is properly connected to SDA and SCL pins");
    Serial.println("- Check power connections (3.3V and GND)");
    Serial.println("- Verify IMU address");
  } else {
    Serial.println("IMU initialized successfully");
  }

  // Perform initial calibration
  calibrateJoystick();
  Serial.print("Initial Joystick X reading: ");
  Serial.println(readJoystickX());
  Serial.print("Current Time: ");
  Serial.println(millis());
  Serial.print("Initial Joystick Y reading: ");
  Serial.println(readJoystickY());

  calibrateIMU();
  readIMUData(tiltX, tiltY, tiltZ);
  Serial.print("Initial IMU X reading: ");
  Serial.println(tiltX);
  Serial.print("Initial IMU Y reading: ");
  Serial.println(tiltY);
  Serial.print("Initial IMU Z reading: ");
  Serial.println(tiltZ);
  if(STARTUP_DELAY) delay(5000);
}

/**
 * @brief Main program loop
 * 
 * Handles the continuous operation of the robot including:
 * - Reading sensor inputs
 * - Processing control commands
 * - Updating servo positions
 * - Controlling motor speeds
 * - Debug printing
 */
void loop() {
  // Check for calibration button presses
  imuPressed() ? Serial.println("IMU Calibration Pressed") : 0;
  joystickPressed() ? Serial.println("Joystick Calibration Pressed") : 0;

  // if (enableLocalControl) {
    // Read joystick values (swapped X and Y)
    float joyXin = readJoystickX();
    float joyYin = readJoystickY();
    if(joyXin < 10000) joystickX = joyXin;
    // Serial.println("Reading");
    if(joyYin < 10000) joystickY = joyYin;
    float mappedX = joystickX;
    float mappedY = joystickY;
    if(joystickX > 0.15) {
        mappedX = map(joystickX * 1000, 150, 1000, 0, 1000) / 1000.0f;
    }
    if(joystickY > 0.15) {
        mappedY = map(joystickY * 1000, 150, 1000, 0, 1000) / 1000.0f;
    }
    rotationX += abs(joystickX) > 0.15 ? mappedX * OUTPUT_SCALE : 0;
    if(rotationX > 180) rotationX = 180;
    if(rotationX < 0) rotationX = 0;
    rotationY += abs(joystickY) > 0.15 ? mappedY * OUTPUT_SCALE : 0;
    if(rotationY > 180) rotationY = 180;
    if(rotationY < 0) rotationY = 0;
    
    //read IMU data
    readIMUData(tiltX, tiltY, tiltZ);
    
  // }

  // Update the servos
  servo1.write(rotationX);
  // Serial.println("Rotation X: " + String(rotationX));
  servo2.write(rotationY);
  Serial.println("Rotation Y: " + String(rotationY));

  // Debug printing for joystick values
  if(PRINT_JOYSTICK_VALUES && !shouldntReadJoystick()){
    unsigned long currentTime = millis();
    if (currentTime - lastJoystickPrint >= PRINT_INTERVAL) {
      Serial.printf("Joystick X: %f\n", joystickX);
      Serial.printf("Joystick Y: %f\n", joystickY); 
      lastJoystickPrint = currentTime;
    }
  }

  // Debug printing for rotation values
  if(PRINT_ROTATION_VALUES){
    unsigned long currentTime = millis();
    if (currentTime - lastRotationPrint >= PRINT_INTERVAL) {
      Serial.printf("Rotation X: %f\n", rotationX);
      Serial.printf("Rotation Y: %f\n", rotationY);
      // Serial.printf("Joystick Y: %f\n", joystickY);
      lastRotationPrint = currentTime;
    }
  }

  // Update motor speeds based on IMU tilt
  activateMotors();
  setMotorSpeeds(tiltX, tiltY);

  // Debug printing for tilt values
  if(PRINT_TILT_VALUES && shouldntReadIMU()){
    unsigned long currentTime = millis();
    if (currentTime - lastTiltPrint >= PRINT_INTERVAL) {
      Serial.printf("TiltX: %f\n", tiltX);
      Serial.printf("TiltY: %f\n", tiltY);
      lastTiltPrint = currentTime;
    }
  }

        // Serial.printf("TiltX: %f\n", tiltX);
      // Serial.printf("TiltY: %f\n", tiltY);
  // Print debug information with calibrated values
  // debugPrintJoystickAndTilt(joystickX, joystickY, tiltX, tiltY);
  // Serial.println("Working");
} 