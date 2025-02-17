#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <WebSocketsServer.h>
#include <WiFi.h>
#include "debug.h"

// WiFi credentials and configuration
const char* WIFI_SSID = "Robot_Controller";
const char* WIFI_PASSWORD = "12345678";
const int WIFI_CHANNEL = 1;  // Use channel 1 for better iOS compatibility
const int MAX_CONNECTIONS = 1;

// WebSocket server instance
extern WebSocketsServer webSocket;
extern bool clientConnected;

// Control variables that will be updated by WebSocket
extern float rotationX;
extern float rotationY;
extern float accel;
extern float accelBiasX;
extern float accelBiasY;
extern bool enableLocalControl;

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

// WiFi setup function
inline void setupWiFi() {
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, 0, MAX_CONNECTIONS);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
}

// WebSocket setup function
inline void setupWebSocket() {
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started");
}

// Implementation of the WebSocket event handler
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