# Nerf Robot Controller

This project implements a controller for a Nerf robot using an ESP32-based Arduino Nano. The controller features both manual control via joystick and remote control via WiFi, along with IMU-based movement control.

## Features

- Manual control using analog joystick
- Remote control via WiFi using WebSocket communication
- IMU-based movement control
- Servo-controlled turret for aiming
- DC motor control for movement
- Calibration capabilities for both IMU and joystick
- Debug mode for monitoring system operation

## Hardware Components

- ESP32-based Arduino Nano
- MPU6050 IMU sensor
- Analog joystick
- Two servo motors (for turret control)
- Two DC motors (for movement)
- Push buttons for calibration and firing
- WiFi capability

## Pin Assignments

### Servo Motors
- SERVO1_PIN (2): Controls horizontal rotation of turret
- SERVO2_PIN (3): Controls vertical rotation of turret

### DC Motors
- MOTOR_RIGHT_PIN (5): Right motor control
- MOTOR_LEFT_PIN (7): Left motor control
- MOTOR_ENABLE_RIGHT_PIN (11): Right motor enable
- MOTOR_ENABLE_LEFT_PIN (10): Left motor enable

### IMU (MPU6050)
- SDA_PIN (A4): I2C Data pin
- SCL_PIN (A5): I2C Clock pin

### Joystick
- STICK_X (A6): X-axis input
- STICK_Y (A3): Y-axis input

### Control Buttons
- IMU_CALIBRATION_PIN (7): IMU calibration button
- JOYSTICK_CALIBRATION_PIN (12): Joystick calibration button
- FIRE_BUTTON_PIN (13): Firing mechanism control

## Software Structure

The project is organized into several modules:

- `main.cpp`: Main program flow and control logic
- `pins.h`: Pin definitions and hardware mapping
- `buttons.h/cpp`: Button handling and debouncing
- `motors.h/cpp`: DC motor control
- `joystick.h/cpp`: Joystick input processing
- `imu.h/cpp`: IMU sensor interface
- `websocket.h`: WiFi and WebSocket communication
- `debug.h`: Debugging utilities
- `debounceReading.h`: Input debouncing functionality

## Control Modes

### Manual Control
- Joystick controls turret rotation
- IMU tilt controls robot movement
- Buttons for calibration and firing

### Remote Control
- WiFi-based WebSocket communication
- Remote client can control turret and movement
- Automatic switching between local and remote control

## Calibration

The system includes calibration capabilities for both the IMU and joystick:

1. IMU Calibration:
   - Press the IMU calibration button
   - Keep the robot stationary during calibration
   - System will store the zero position

2. Joystick Calibration:
   - Press the joystick calibration button
   - Keep the joystick centered during calibration
   - System will store the center position

## Debug Mode

Debug mode can be enabled by setting `DEBUG_MODE` to 1 in `debug.h`. When enabled, the system will print:
- Joystick values
- IMU tilt values
- Servo rotation values
- Connection status
- System heartbeat

## Dependencies

- Arduino.h
- ESP32Servo.h
- Wire.h
- WebSocketsServer.h
- WiFi.h
- Adafruit_MPU6050.h
- Adafruit_Sensor.h

## Building and Uploading

1. Install the required libraries in your Arduino IDE
2. Select the appropriate board (ESP32-based Arduino Nano)
3. Compile and upload the code

