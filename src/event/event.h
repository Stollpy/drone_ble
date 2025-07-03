// events.h
#ifndef EVENT_H
#define EVENT_H

#include <stdint.h>
#include <string.h>

#include "event_motor.h"
#include "event_ble.h"

typedef enum {
    EVENT_MOTOR_UPDATE,
    EVENT_BLE_CONNECTED,
    EVENT_BLE_DISCONNECTED,
    EVENT_BLE_MOTORS_COMMAND,
    EVENT_UNKNOWN
} event_type_t;

typedef struct {
    event_type_t type;
    union {
        event_motor_data_t motor;
        event_ble_motors_command_data_t ble_motors_command;
    } data;
} event_t;

typedef void (*event_handler_t)(event_t *event);

void event_bus_subscribe(event_type_t type, event_handler_t handler);
void event_bus_publish(event_t *event);

#endif
