#ifndef EVENT_MOTOR_H
#define EVENT_MOTOR_H

#include <stdint.h>

typedef struct {
    uint8_t motor_id;
    uint8_t direction;
    uint8_t speed;
} event_motor_data_t;

#endif