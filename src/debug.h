#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

#define DEBUG_MODE 0 // 0 = off, 1 = on

// Debug print intervals
const unsigned long STATUS_PRINT_INTERVAL = 2000;  // Print every 2000ms (2 seconds)
const unsigned long HEARTBEAT_INTERVAL = 5000;     // Print every 5000ms (5 seconds)
const unsigned long WIFI_STATUS_INTERVAL = 3000;   // Print every 3000ms (3 seconds)

// Debug print timestamps
static unsigned long lastStatusPrint = 0;
static unsigned long lastHeartbeatPrint = 0;
static unsigned long lastWiFiStatusPrint = 0;

// Debug print functions
inline void debugPrintJoystickAndTilt(float joystickX, float joystickY, float tiltX, float tiltY, uint8_t buttons = 0) {
    if (DEBUG_MODE == 1) {
        Serial.print("Joystick X: "); Serial.println(joystickX);
        Serial.print("Joystick Y: "); Serial.println(joystickY);
        Serial.print("Tilt X: "); Serial.println(tiltX);
        Serial.print("Tilt Y: "); Serial.println(tiltY);
        if (buttons > 0) {
            Serial.print("Buttons: 0b"); Serial.println(buttons, BIN);
        }
        // Serial.println("---");
    }
}

inline void debugPrintConnectionStatus(bool clientConnected, int stationCount) {
    if (DEBUG_MODE == 1) {
        unsigned long currentMillis = millis();
        
        // Print status if client is connected
        if (clientConnected && (currentMillis - lastStatusPrint >= STATUS_PRINT_INTERVAL)) {
            Serial.println("Client is connected!");
            lastStatusPrint = currentMillis;
        }
        
        // Print heartbeat
        if (currentMillis - lastHeartbeatPrint >= HEARTBEAT_INTERVAL) {
            Serial.println("Serial communication heartbeat - Robot controller running");
            lastHeartbeatPrint = currentMillis;
        }
        
        // Print WiFi status
        if (currentMillis - lastWiFiStatusPrint >= WIFI_STATUS_INTERVAL) {
            if (stationCount > 0 && !clientConnected) {
                Serial.println("Device connected to WiFi but WebSocket connection not established");
            }
            lastWiFiStatusPrint = currentMillis;
        }
    }
}

#endif // DEBUG_H 