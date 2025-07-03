#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "config.h"
#include "event/event.h"


void motors_init();
void motors_command_handler(event_t *event);
// void motor_set(uint8_t id, uint8_t direction, uint8_t speed); // direction: 0 stop, 1 avant, 2 arrière

#endif