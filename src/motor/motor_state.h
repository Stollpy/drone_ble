#ifndef MOTOR_STATE
#define MOTOR_STATE

typedef struct {
    int x;
    int y;
} motors_joystick_state_t;

static motors_joystick_state_t joystick_state = {
    .x = -1,
    .y = -1
};
#endif