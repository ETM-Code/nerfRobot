/**
 * @file joystick.h
 * @brief Joystick control interface for the Nerf Robot Controller
 * 
 * This file contains function declarations and constants for handling
 * the analog joystick input used for manual control of the robot.
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

// Scaling constants for joystick input processing
#define JOYSTICK_SCALE_X 512.0  // Scaling factor for X-axis input to make controls responsive
#define JOYSTICK_SCALE_Y 512.0  // Scaling factor for Y-axis input to make controls responsive
#define OUTPUT_SCALE 0.1        // Scaling factor for final output values

/**
 * @brief Initializes the joystick analog input pins
 */
void setupJoystick(void);

/**
 * @brief Reads and processes the X-axis value from the joystick
 * @return Processed X-axis value (-1.0 to 1.0)
 */
float readJoystickX(void);

/**
 * @brief Reads and processes the Y-axis value from the joystick
 * @return Processed Y-axis value (-1.0 to 1.0)
 */
float readJoystickY(void);

/**
 * @brief Calibrates the joystick by reading and storing the center position
 */
void calibrateJoystick(void);

#endif // JOYSTICK_H