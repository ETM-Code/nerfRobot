#ifndef BUTTONS_H
#define BUTTONS_H

void setupButtons();
bool imuPressed();
bool joystickPressed();
#define IMU_BUTTON_PRESS_INTERVAL 100
#define JOYSTICK_BUTTON_PRESS_INTERVAL 100
#endif