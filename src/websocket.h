/**
 * @file websocket.h
 * @brief WebSocket communication interface for the Nerf Robot Controller
 * 
 * This file provides WebSocket functionality for remote control of the robot
 * through a WiFi connection. It handles client connections, data transmission,
 * and command processing.
 * Later this file was partially neglected to meet the requirement of controlling
 * the robot through physical components.
 */

#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <WebSocketsServer.h>
#include <WiFi.h>
#include "debug.h"

// WiFi configuration
const char* WIFI_SSID = "Robot_Controller";  // Network name
const char* WIFI_PASSWORD = "12345678";       // Network password
const int WIFI_CHANNEL = 1;                   // WiFi channel (1 for better iOS compatibility)
const int MAX_CONNECTIONS = 1;                // Maximum number of simultaneous connections

// WebSocket server instance and connection status
extern WebSocketsServer webSocket;
extern bool clientConnected;

// Control variables updated via WebSocket
extern float rotationX;      // Horizontal rotation of turret
extern float rotationY;      // Vertical rotation of turret
extern float accel;          // Acceleration value
extern float accelBiasX;     // X-axis acceleration bias
extern float accelBiasY;     // Y-axis acceleration bias
extern bool enableLocalControl;  // Flag to enable/disable local control

/**
 * @brief WebSocket event handler for processing client messages
 * 
 * Handles various WebSocket events including:
 * - Client connections/disconnections
 * - Text messages
 * - Binary data (control commands)
 * 
 * @param num Client number
 * @param type Event type
 * @param payload Message payload
 * @param length Payload length
 */
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

/**
 * @brief Sets up the WiFi access point
 * 
 * Configures the ESP32 as a WiFi access point with the specified
 * credentials and settings.
 */
inline void setupWiFi() {
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, 0, MAX_CONNECTIONS);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
}

/**
 * @brief Sets up the WebSocket server
 * 
 * Initializes the WebSocket server and registers the event handler.
 */
inline void setupWebSocket() {
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started");
}

/**
 * @brief Implementation of the WebSocket event handler
 * 
 * Processes incoming WebSocket events and updates control variables
 * based on received commands.
 */
inline void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            enableLocalControl = true;
            clientConnected = false;
            Serial.println("Client disconnected!");
            break;
        
        case WStype_CONNECTED:
            enableLocalControl = false;
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

                // Update control variables based on received data
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

                debugPrintJoystickAndTilt(joystickX, joystickY, tiltX, tiltY, buttons);
            }
            break;
    }
}

#endif // WEBSOCKET_H 