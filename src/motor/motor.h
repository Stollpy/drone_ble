#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "config.h"
#include "event/event.h"
#include "motor_state.h"

void motors_init(void);
void motors_command_handler(event_t *event);
void motors_direction_handler(event_t *event);

#endif 