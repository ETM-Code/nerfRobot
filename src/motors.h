/**
 * @file motors.h
 * @brief Motor control interface for the Nerf Robot Controller
 * 
 * This file contains function declarations for controlling the DC motors
 * that drive the robot's movement.
 */

#ifndef MOTORS_H
#define MOTORS_H

/**
 * @brief Initializes the motor control pins and sets up PWM
 */
void setupMotors();

/**
 * @brief Sets the speed and direction of both motors based on tilt values
 * @param tiltX Tilt value for X-axis (controls left/right movement)
 * @param tiltY Tilt value for Y-axis (controls forward/backward movement)
 */
void setMotorSpeeds(float tiltX, float tiltY);

/**
 * @brief Activates the motor enable pins to allow motor movement
 */
void activateMotors();

#endif
