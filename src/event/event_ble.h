// event_ble.h
#ifndef EVENT_BLE_H
#define EVENT_BLE_H

#include <stdint.h>

typedef enum {
    MOTOR_CMD_STOP,
    MOTOR_CMD_START,
    MOTOR_CMD_SPEED,
    MOTOR_CMD_DIRECTION
} motor_command_type_t;

typedef struct {
    motor_command_type_t type;
    uint8_t motor_id;
    uint8_t value;
} event_ble_motors_command_data_t;

typedef struct {
    char axe;
    int position;
} event_ble_joystick_direction_data_t;

#endif 