#ifndef JOYSTICK_H
#define JOYSTICK_H

#define JOYSTICK_SCALE_X 512.0
#define JOYSTICK_SCALE_Y 512.0
#define OUTPUT_SCALE 8.0

void setupJoystick(void);
int readJoystickX(void);
int readJoystickY(void);
void calibrateJoystick(void);

#endif // JOYSTICK_H