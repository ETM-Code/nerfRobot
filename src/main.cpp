#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <pins.h>
#include "Wire.h"
#include "SparkFunLSM6DS3.h"

#define DEBUG_MODE 1 // 0 = off, 1 = on

// WiFi credentials for Access Point mode
const char* ssid = "Robot_Controller";
const char* password = "12345678";

// WiFi configuration
const int WIFI_CHANNEL = 1;  // Use channel 1 for better iOS compatibility
const int MAX_CONNECTIONS = 1;

// Initialize WebSocket server on port 81
WebSocketsServer webSocket = WebSocketsServer(81);

bool clientConnected = false;
unsigned long lastStatusPrint = 0;  // Variable to track last status print time
const unsigned long STATUS_PRINT_INTERVAL = 2000;  // Print every 2000ms (2 seconds)
unsigned long lastHeartbeatPrint = 0;  // Variable to track last heartbeat print time
const unsigned long HEARTBEAT_INTERVAL = 5000;  // Print every 5000ms (5 seconds)
unsigned long lastWiFiStatusPrint = 0;  // Variable to track last WiFi status print time
const unsigned long WIFI_STATUS_INTERVAL = 3000;  // Print every 3000ms (3 seconds)

// Servo objects
Servo servo1;
Servo servo2;


// Rotation Values
float rotationX = 0;
float rotationY = 0;

// DC Motor Values
float accel = 0;
float accelBiasX = 0;
float accelBiasY = 0;

// Joystick and IMU
LSM6DS3 controlIMU;
float tiltX = 0;
float tiltY = 0;
float joystickX = 0;
float joystickY = 0;



// Enabling/Disabling Features
bool enableLocalControl = true;


// Callback function for WebSocket events
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
      case WStype_DISCONNECTED:
        enableLocalControl = true; // Enable local control when disconnected from the app
        clientConnected = false;
        Serial.println("Client disconnected!");
        break;
      
      case WStype_CONNECTED:
        enableLocalControl = false; // Disable local control when connected to the app
        clientConnected = true;
        Serial.println("Client connected!");
        break;
      
      case WStype_TEXT:
        // Handle text data (if needed)
        break;
      
      case WStype_BIN:
        if (length == 17) { // 4 floats (4 bytes each) + 1 byte for buttons
          float joystickX, joystickY, tiltX, tiltY;
          uint8_t buttons;
          
          // Extract the floats and buttons from the data
          memcpy(&joystickX, payload, sizeof(float));
          memcpy(&joystickY, payload + 4, sizeof(float));
          memcpy(&tiltX, payload + 8, sizeof(float));
          memcpy(&tiltY, payload + 12, sizeof(float));
          buttons = payload[16];

          rotationX = constrain(rotationX + joystickX * 8, -90, 90);
          rotationY = constrain(rotationY + joystickY * 8, -90, 90);

          accel = tiltX;
          if(abs(tiltY) > 0.5) {
            if (tiltY > 0) {
              accelBiasY = tiltY;
            }
            else {
              accelBiasX = -tiltY;
            }
          }
          else {
            accelBiasX = 0;
            accelBiasY = 0;
          }

          // Print the received values
          if (DEBUG_MODE == 1) {
            Serial.print("Joystick X: "); Serial.println(joystickX);
            Serial.print("Joystick Y: "); Serial.println(joystickY);
            Serial.print("Tilt X: "); Serial.println(tiltX);
            Serial.print("Tilt Y: "); Serial.println(tiltY);
            Serial.print("Buttons: 0b"); Serial.println(buttons, BIN);
          Serial.println("---");
        }
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting WiFi Robot Controller...");

  // Set WiFi to max power
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // Configure AP with explicit settings
  WiFi.softAP(ssid, password, WIFI_CHANNEL, 0, MAX_CONNECTIONS);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("WebSocket server started");
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

  // Attach the IMU
  if(controlIMU.begin() != 0) {
    Serial.println("Failed to initialize IMU");
  }
  else {
    Serial.println("IMU initialized successfully");
  }
  
}

void loop() {
  webSocket.loop();

  // Update the servos
  servo1.write(rotationX);
  servo2.write(rotationY);

  // Update the DC Motors
  analogWrite(MOTOR_RIGHT_PIN, accel + accelBiasX - accelBiasY);
  analogWrite(MOTOR_LEFT_PIN, accel - accelBiasX + accelBiasY);
  
  unsigned long currentMillis = millis();

  if (DEBUG_MODE == 1) {
      // Print status every 2 seconds if client is connected
      if (clientConnected && (currentMillis - lastStatusPrint >= STATUS_PRINT_INTERVAL)) {
        Serial.println("Client is connected!");
        lastStatusPrint = currentMillis;
      }
    
    // Print heartbeat every 5 seconds regardless of connection status
    if (currentMillis - lastHeartbeatPrint >= HEARTBEAT_INTERVAL) {
      Serial.println("Serial communication heartbeat - Robot controller running");
      lastHeartbeatPrint = currentMillis;
    }
    
    // Print WiFi status every 3 seconds if someone is connected to WiFi but not WebSocket
    if (currentMillis - lastWiFiStatusPrint >= WIFI_STATUS_INTERVAL) {
      int stationCount = WiFi.softAPgetStationNum();
      if (stationCount > 0 && !clientConnected) {
        Serial.println("Device connected to WiFi but WebSocket connection not established");
      }
      lastWiFiStatusPrint = currentMillis;
    }
  }
} 