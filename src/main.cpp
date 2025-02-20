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
// WebSocket server instance
WebSocketsServer webSocket = WebSocketsServer(81);
bool clientConnected = false;
bool enableLocalControl = true;

// Servo objects
Servo servo1;
Servo servo2;

// Control variables
float rotationX = 0;
float rotationY = 0;
float accel = 0;
float accelBiasX = 0;
float accelBiasY = 0;

// Joystick and IMU variables
float tiltX = 0;
float tiltY = 0;
float tiltZ = 0;
float magX = 0;
float magY = 0;
float magZ = 0;
float joystickX = 0;
float joystickY = 0;

// Calibration variables
float imuOffsetX = 0;
float imuOffsetY = 0;
float joystickOffsetX = 0;
float joystickOffsetY = 0;
float joystickScaleX = 1.0;
float joystickScaleY = 1.0;

void setup() {
  delay(5000);
  Serial.begin(115200);
  Serial.println("Starting WiFi Robot Controller...");

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
  if (!initializeIMU())
  {
    Serial.println("Failed to initialize IMU. Please check connections:");
    Serial.println("- Ensure IMU is properly connected to SDA and SCL pins");
    Serial.println("- Check power connections (3.3V and GND)");
    Serial.println("- Verify IMU address");
  }
  else
  {
    Serial.println("IMU initialized successfully");
  }

  // Print initial analog readings
  calibrateJoystick();
  Serial.print("Initial Joystick X reading: ");
  Serial.println(readJoystickX());
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
}

void loop() {
  webSocket.loop();

  // Check for IMU calibration
  if (imuPressed()) {
    calibrateIMU();
  }

  // Check for Joystick calibration
  if (joystickPressed()) {
    calibrateJoystick();
  }

  if (enableLocalControl) {
    // Read joystick values (swapped X and Y)
    joystickX = readJoystickX();
    joystickY = readJoystickY();
    rotationX += joystickX/JOYSTICK_SCALE_X * OUTPUT_SCALE;
    rotationY += joystickY/JOYSTICK_SCALE_Y * OUTPUT_SCALE;
    
    //read IMU data
    readIMUData(tiltX, tiltY, tiltZ);
    
  }

  // Update the servos
  servo1.write(rotationX);
  servo2.write(rotationY);
  Serial.printf("Joystick X: %d", joystickX);
  Serial.printf("Joystick Y: %d", joystickY); 

  // Update the DC Motors
  setMotorSpeeds(tiltX, tiltY);
  Serial.printf("TiltX: %f", tiltX);
  Serial.printf("TiltY: %f", tiltY);
  
  // Print debug information with calibrated values
  debugPrintJoystickAndTilt(joystickX, joystickY, tiltX, tiltY);
  // Serial.println("Working");
} 