#ifndef EVENT_BLE_H
#define EVENT_BLE_H

#include <stdint.h>

typedef struct {
    uint8_t connected;
} event_ble_connected_data_t;

typedef struct {
    uint8_t disconnected;
} event_ble_disconnected_data_t;

typedef enum {
    MOTOR_CMD_STOP = 0,
    MOTOR_CMD_START = 1,
    // MOTOR_CMD_REVERSE = 2
} event_ble_motor_command_type_t;

typedef struct {
    event_ble_motor_command_type_t type;
} event_ble_motors_command_data_t;

#endif