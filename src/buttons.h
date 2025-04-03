/**
 * @file buttons.h
 * @brief Button handling functions for the Nerf Robot Controller
 * 
 * This file contains function declarations for handling various buttons
 * used for calibration and control of the robot.
 */

#ifndef BUTTONS_H
#define BUTTONS_H

/**
 * @brief Initializes all button pins with appropriate configurations
 */
void setupButtons();

/**
 * @brief Checks if the IMU calibration button is pressed
 * @return true if button is pressed and debounced, false otherwise
 */
bool imuPressed();

/**
 * @brief Checks if the joystick calibration button is pressed
 * @return true if button is pressed and debounced, false otherwise
 */
bool joystickPressed();

/**
 * @brief Checks if the fire button is pressed
 * @return true if button is pressed and debounced, false otherwise
 */
bool firePressed();

// Debounce timing constants (in milliseconds)
#define IMU_BUTTON_PRESS_INTERVAL 100      // Debounce time between IMU button presses
#define JOYSTICK_BUTTON_PRESS_INTERVAL 100 // Debounce time between joystick button presses
#define FIRE_BUTTON_PRESS_INTERVAL 100     // Debounce time between fire button presses

#endif