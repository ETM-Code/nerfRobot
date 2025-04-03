/**
 * @file debug.h
 * @brief Debugging functionality for the Nerf Robot Controller
 * 
 * This file provides debugging functions and utilities for monitoring
 * the robot's operation and troubleshooting issues.
 * Later the debug mode was partially neglected, as it was more convenient 
 * to use the Serial Monitor for certain elements.
 */

#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

// Debug mode toggle (0 = off, 1 = on)
#define DEBUG_MODE 0

// Debug print intervals (in milliseconds)
const unsigned long STATUS_PRINT_INTERVAL = 2000;  // Print every 2000ms (2 seconds)
const unsigned long HEARTBEAT_INTERVAL = 5000;     // Print every 5000ms (5 seconds)
const unsigned long WIFI_STATUS_INTERVAL = 3000;   // Print every 3000ms (3 seconds)

// Timestamps for tracking last debug prints
static unsigned long lastStatusPrint = 0;
static unsigned long lastHeartbeatPrint = 0;
static unsigned long lastWiFiStatusPrint = 0;

/**
 * @brief Prints joystick and tilt values for debugging
 * 
 * Prints the current values of joystick and tilt inputs when debug mode
 * is enabled. This helps in monitoring the input values and their changes.
 * 
 * @param joystickX Current X-axis joystick value
 * @param joystickY Current Y-axis joystick value
 * @param tiltX Current X-axis tilt value
 * @param tiltY Current Y-axis tilt value
 * @param buttons Current button state (optional)
 */
inline void debugPrintJoystickAndTilt(float joystickX, float joystickY, float tiltX, float tiltY, uint8_t buttons = 0) {
    if (DEBUG_MODE == 1) {
        Serial.print("Joystick X: "); Serial.println(joystickX);
        Serial.print("Joystick Y: "); Serial.println(joystickY);
        Serial.print("Tilt X: "); Serial.println(tiltX);
        Serial.print("Tilt Y: "); Serial.println(tiltY);
        if (buttons > 0) {
            Serial.print("Buttons: 0b"); Serial.println(buttons, BIN);
        }
    }
}

/**
 * @brief Prints connection status and system heartbeat
 * 
 * Monitors and prints various system status information including:
 * - Client connection status
 * - System heartbeat
 * - WiFi connection status
 * 
 * @param clientConnected Current WebSocket client connection status
 * @param stationCount Number of connected WiFi stations
 */
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