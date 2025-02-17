#include <Arduino.h>
#include <ESP32Servo.h>
#include <pins.h>
#include "Wire.h"
#include "debug.h"
#include "websocket.h"
#include "imu.h"

// WebSocket server instance
WebSocketsServer webSocket = WebSocketsServer(81);
bool clientConnected = false;

// Servo objects
Servo servo1;
Servo servo2;

// Control variables
float rotationX = 0;
float rotationY = 0;
float accel = 0;
float accelBiasX = 0;
float accelBiasY = 0;
bool enableLocalControl = true;

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
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PIN, OUTPUT);

  // Attach the Joystick
  pinMode(STICK_X, INPUT);
  pinMode(STICK_Y, INPUT);

  // Setup calibration pins
  pinMode(IMU_CALIBRATION_PIN, INPUT);
  pinMode(JOYSTICK_CALIBRATION_PIN, INPUT);

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  
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
  Serial.print("Initial Joystick X reading: ");
  Serial.println(analogRead(STICK_X));
  Serial.print("Initial Joystick Y reading: ");
  Serial.println(analogRead(STICK_Y));
}

void loop() {
  webSocket.loop();

  // Check for IMU calibration
  if (digitalRead(IMU_CALIBRATION_PIN) == HIGH) {
    float sumTiltX = 0, sumTiltY = 0;
    const int numSamples = 10;
    for (int i = 0; i < numSamples; i++) {
      readIMUData(tiltX, tiltY, tiltZ, magX, magY, magZ);
      sumTiltX += tiltX;
      sumTiltY += tiltY;
      delay(10);
    }
    imuOffsetX = sumTiltX / numSamples;
    imuOffsetY = sumTiltY / numSamples;
    Serial.println("IMU calibrated!");
    Serial.print("X offset: "); Serial.println(imuOffsetX);
    Serial.print("Y offset: "); Serial.println(imuOffsetY);
    delay(500); // Debounce
  }

  // Check for Joystick calibration
  if (digitalRead(JOYSTICK_CALIBRATION_PIN) == HIGH) {
    long sumX = 0, sumY = 0;
    const int numSamples = 10;
    for (int i = 0; i < numSamples; i++) {
      sumX += analogRead(STICK_Y);  // Note: X and Y are swapped as per original code
      sumY += analogRead(STICK_X);
      delay(10);
    }
    int rawX = sumX / numSamples;
    int rawY = sumY / numSamples;
    joystickOffsetX = rawX;
    joystickOffsetY = rawY;
    // Calculate maximum deviation from the center reading for each axis
    int maxDevX = max(rawX, 4095 - rawX);
    int maxDevY = max(rawY, 4095 - rawY);
    // Use the maximum deviation to generate a scale factor to map the deflection to [-1,1]
    joystickScaleX = (maxDevX != 0) ? (1.0 / maxDevX) : 1.0;
    joystickScaleY = (maxDevY != 0) ? (1.0 / maxDevY) : 1.0;
    Serial.println("Joystick calibrated!");
    Serial.print("X offset: "); Serial.println(joystickOffsetX);
    Serial.print("Y offset: "); Serial.println(joystickOffsetY);
    Serial.print("X scale: "); Serial.println(joystickScaleX);
    Serial.print("Y scale: "); Serial.println(joystickScaleY);
    delay(500); // Debounce
  }

  if (enableLocalControl) {
    // Read joystick values (swapped X and Y)
    int rawJoystickY = analogRead(STICK_X);
    int rawJoystickX = analogRead(STICK_Y);
    
    // Apply calibration: subtract the offset and scale the deviation to normalize to [-1, 1]
    joystickX = (rawJoystickX - joystickOffsetX) * joystickScaleX;
    joystickY = -1 * (rawJoystickY - joystickOffsetY) * joystickScaleY;
    
    // Directly scale the normalized values to set the rotation increment (max increment around 8)
    float scaledJoystickX = joystickX * 8.0;
    float scaledJoystickY = joystickY * 8.0;
    
    // Update rotation values with constraint so the servo positions remain within [-90,90]
    rotationX = constrain(rotationX + scaledJoystickX, -90, 90);
    rotationY = constrain(rotationY - scaledJoystickY, -90, 90);
    
    // Read IMU values for tilt-based control
    readIMUData(tiltX, tiltY, tiltZ, magX, magY, magZ);
    
    // Apply the IMU calibration offsets
    tiltX -= imuOffsetX;
    tiltY -= imuOffsetY;
    
    // Apply tilt controls similar to websocket logic
    if (abs(tiltY) > 0.5) {
      if (tiltY > 0) {
        accelBiasY = tiltY;
      } else {
        accelBiasX = -tiltY;
      }
    } else {
      accelBiasX = 0;
      accelBiasY = 0;
    }
  }

  // Update the servos
  servo1.write(rotationX);
  servo2.write(rotationY);

  // Update the DC Motors
  analogWrite(MOTOR_RIGHT_PIN, accel + accelBiasX - accelBiasY);
  analogWrite(MOTOR_LEFT_PIN, accel - accelBiasX + accelBiasY);
  
  // Print debug information with calibrated values
  debugPrintJoystickAndTilt(joystickX, joystickY, tiltX, tiltY);
  // Serial.println("Working");
} 