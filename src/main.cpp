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
// WebSocket server instance
WebSocketsServer webSocket = WebSocketsServer(81);
bool clientConnected = false;
bool enableLocalControl = true;

bool PRINT_TILT_VALUES = false;
bool PRINT_JOYSTICK_VALUES = false;
bool PRINT_ROTATION_VALUES = false;
// Servo objects
Servo servo1;
Servo servo2;

// Control variables
float rotationX = 90;
float rotationY = 90;
float accel = 0;
float accelBiasX = 0;
float accelBiasY = 0;

// Timestamps for non-blocking prints
unsigned long lastJoystickPrint = 0;
unsigned long lastRotationPrint = 0;
unsigned long lastTiltPrint = 0;
const unsigned long PRINT_INTERVAL = 100; // 100ms interval

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
  delay(2000);
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

  // if (enableLocalControl) {
    // Read joystick values (swapped X and Y)
    float joyXin = readJoystickX();
    float joyYin = readJoystickY();
    if(joyXin < 10000) joystickX = joyXin;
    // Serial.println("Reading");
    if(joyYin < 10000) joystickY = joyYin;
    rotationX += abs(joystickX) > 0.15 ? joystickX * OUTPUT_SCALE : 0;
    if(rotationX > 180) rotationX = 180;
    if(rotationX < 0) rotationX = 0;
    rotationY += abs(joystickY) > 0.15 ? joystickY * OUTPUT_SCALE : 0;
    if(rotationY > 180) rotationY = 180;
    if(rotationY < 0) rotationY = 0;
    
    //read IMU data
    readIMUData(tiltX, tiltY, tiltZ);
    
  // }

  // Update the servos
  servo1.write(rotationX);
  servo2.write(rotationY);

  if(PRINT_JOYSTICK_VALUES && !shouldntReadJoystick()){
    unsigned long currentTime = millis();
    if (currentTime - lastJoystickPrint >= PRINT_INTERVAL) {
      Serial.printf("Joystick X: %f\n", joystickX);
      Serial.printf("Joystick Y: %f\n", joystickY); 
      lastJoystickPrint = currentTime;
    }
  }

  if(PRINT_ROTATION_VALUES){
    unsigned long currentTime = millis();
    if (currentTime - lastRotationPrint >= PRINT_INTERVAL) {
      Serial.printf("Rotation X: %f\n", rotationX);
      Serial.printf("Rotation Y: %f\n", rotationY);
      // Serial.printf("Joystick Y: %f\n", joystickY);
      lastRotationPrint = currentTime;
    }
  }
  // Update the DC Motors
  setMotorSpeeds(tiltX, tiltY);
  if(PRINT_TILT_VALUES && shouldntReadIMU()){
    unsigned long currentTime = millis();
    if (currentTime - lastTiltPrint >= PRINT_INTERVAL) {
      Serial.printf("TiltX: %f\n", tiltX);
      Serial.printf("TiltY: %f\n", tiltY);
      lastTiltPrint = currentTime;
    }
  }
  // Print debug information with calibrated values
  // debugPrintJoystickAndTilt(joystickX, joystickY, tiltX, tiltY);
  // Serial.println("Working");
} 