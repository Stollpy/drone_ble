#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "config.h"
#include "event/event.h"
#include "motor_state.h"

// MOTOR 1
// #define MOTOR_1_PIN GPIO_NUM_4
#define MOTOR_1_PIN_1 GPIO_NUM_4
#define MOTOR_1_PIN_2 GPIO_NUM_17
#define MOTOR_1_ENABLE_PIN GPIO_NUM_16

// MOTOR 2
#define MOTOR_2_PIN_1 GPIO_NUM_25
#define MOTOR_2_PIN_2 GPIO_NUM_26
#define MOTOR_2_ENABLE_PIN GPIO_NUM_33

void motors_init(void);
void motors_command_handler(event_t *event);
void motors_direction_handler(event_t *event);

#endif 