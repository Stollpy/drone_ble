// event_motor.h
#ifndef EVENT_MOTOR_H
#define EVENT_MOTOR_H

#include <stdint.h>

typedef struct {
    uint8_t motor_id;
    uint8_t speed;
    uint8_t direction;
} event_motor_data_t;

#endif 