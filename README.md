# Robot Controller Project

## Overview
This project implements a robot controller using an Arduino Nano ESP32 board. The system features dual servo control, DC motor control, and supports both local control via a joystick and remote control through a WebSocket interface. The robot includes accelerometer-based tilt control.

## Hardware Requirements
- Arduino Nano ESP32
- Accelerometer (specific model TBD)
- 2x Servo Motors
- 2x DC Motors
- Analog Joystick
- 2x Calibration Buttons

## Project Structure
### Core Files
- `src/main.cpp` - Main program loop and setup
- `src/pins.h` - Pin definitions and hardware configuration
- `src/debug.h` - Debugging utilities and serial output configuration

### Control Systems
- `src/imu.h` & `src/imu.cpp` - Accelerometer interface and tilt sensing
- `src/websocket.h` - WebSocket server and remote control implementation

### Configuration
- `platformio.ini` - PlatformIO project configuration and dependencies

## Pin Configuration
### Servos
- Servo 1: Pin 2
- Servo 2: Pin 3

### DC Motors
- Right Motor: Pin A2
- Left Motor: Pin A1

### IMU (I2C)
- SDA: Pin A4 (GPIO 8)
- SCL: Pin A5 (GPIO 9)

### Joystick
- X-axis: Pin A3
- Y-axis: Pin A7

### Calibration Buttons
- IMU Calibration: Pin 7
- Joystick Calibration: Pin 12

## Software Dependencies
The project uses PlatformIO for development. Required libraries:
- WebSockets (v2.4.1)
- ArduinoJson (v6.21.3)
- ESP32Servo (v3.0.6)
- Accelerometer library (TBD based on final sensor choice)

## Setup and Configuration
1. Install PlatformIO in your development environment
2. Clone this repository
3. Open the project in PlatformIO
4. Install the required dependencies
5. Connect the hardware according to the pin configuration
6. Upload the code to your Arduino Nano ESP32

## Features
- Dual control modes:
  - Local control via joystick
  - Remote control via WebSocket interface
- Tilt-based control using accelerometer
- Servo position control
- DC motor speed control
- Calibration system for both accelerometer and joystick
- WiFi Access Point for remote connection
- Debug output system

## Network Configuration
The robot creates a WiFi Access Point with:
- SSID: "Robot_Controller"
- Password: "12345678"
- Channel: 1 (optimized for iOS compatibility)
- WebSocket port: 81

## Current Development Status and Planned Changes

### Accelerometer Implementation
The current accelerometer setup is under review with the following considerations:
1. Need to select an appropriate accelerometer with a suitable g-range for robot control
2. Planning to implement more robust tilt sensing algorithms
3. Improve calibration procedures for more accurate readings

### Areas for Improvement
1. Accelerometer selection criteria:
   - Appropriate g-range for robot control applications
   - Good sensitivity for tilt detection
   - Reliable I2C communication
2. Enhanced tilt detection algorithms
3. More robust calibration procedures

## Development Guide
### Making Changes
- For accelerometer modifications: Focus on `src/imu.cpp` and `src/imu.h`
- For control logic changes: Modify `src/main.cpp`
- For pin reassignment: Update `src/pins.h`
- For remote control modifications: Edit `src/websocket.h`
- For debugging: Configure options in `src/debug.h`

## Debugging
- Serial output is available at 115200 baud
- Debug mode can be enabled/disabled in `debug.h`
- Various debug print functions are available for monitoring:
  - Joystick and tilt values
  - Connection status
  - WiFi status
  - Heartbeat messages

## Contributing
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License
MIT License

Copyright (c) 2025

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. 