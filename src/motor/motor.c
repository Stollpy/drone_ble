#include "motor/motor.h"

ledc_timer_config_t ledc_timer_motor_1 = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .timer_num        = LEDC_TIMER_0,
    .duty_resolution  = LEDC_TIMER_8_BIT, // Résolution sur 8 bits (0-255)
    .freq_hz          = 5000,             // Fréquence PWM (ex: 5 kHz)
    .clk_cfg          = LEDC_AUTO_CLK
};

ledc_channel_config_t ledc_channel_motor_1 = {
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_0,
    .timer_sel      = LEDC_TIMER_0,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = MOTOR_1_ENABLE_PIN,
    .duty           = 0, // Valeur initiale du duty cycle (0 = éteint)
    .hpoint         = 0
};

// Ajout pour le deuxième moteur
ledc_timer_config_t ledc_timer_motor_2 = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,
    .timer_num        = LEDC_TIMER_1,
    .duty_resolution  = LEDC_TIMER_8_BIT,
    .freq_hz          = 5000,
    .clk_cfg          = LEDC_AUTO_CLK
};

ledc_channel_config_t ledc_channel_motor_2 = {
    .speed_mode     = LEDC_LOW_SPEED_MODE,
    .channel        = LEDC_CHANNEL_1,
    .timer_sel      = LEDC_TIMER_1,
    .intr_type      = LEDC_INTR_DISABLE,
    .gpio_num       = MOTOR_2_ENABLE_PIN,
    .duty           = 0,
    .hpoint         = 0
};

void motors_command_handler(event_t *event) {
    if (event->type == EVENT_BLE_MOTORS_COMMAND) {
        if (event->data.ble_motors_command.type == MOTOR_CMD_START) {
            gpio_set_level(MOTOR_1_PIN_1, 1);
            gpio_set_level(MOTOR_2_PIN_1, 1);
        } else if (event->data.ble_motors_command.type == MOTOR_CMD_STOP) {
            gpio_set_level(MOTOR_1_PIN_1, 0);
            gpio_set_level(MOTOR_2_PIN_1, 0);
        }
    }
}

static int normalize_joystick_value(int raw) {
    int delta = raw - JOYSTICK_ADC_CENTER;
    if (abs(delta) < JOYSTICK_DEADZONE) return 0;
    return delta;
}

static void update_motor_speed_from_joystick() {
    int y = normalize_joystick_value(joystick_state.y);
    int x = normalize_joystick_value(joystick_state.x);

    int base_speed = y;
    int diff = x;

    int motor1_speed = base_speed - diff;
    int motor2_speed = base_speed + diff;

    motor1_speed = MIN(MAX(motor1_speed, 0), 255);
    motor2_speed = MIN(MAX(motor2_speed, 0), 255);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, motor1_speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, motor2_speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void motors_direction_handler(event_t *event) {
    if (event->type != EVENT_BLE_JOYSTICK_DIRECTION) return;

    if (event->data.ble_joystick_direction.axe == 'x') {
        joystick_state.x = event->data.ble_joystick_direction.position;
    } else if (event->data.ble_joystick_direction.axe == 'y') {
        joystick_state.y = event->data.ble_joystick_direction.position;
    }

    update_motor_speed_from_joystick();
}


void motors_init() {
    gpio_set_direction(MOTOR_1_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_1_PIN_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_1_ENABLE_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(MOTOR_2_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_PIN_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_ENABLE_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(MOTOR_1_PIN_1, 0);  // IN1 = 1
    gpio_set_level(MOTOR_1_PIN_2, 0);  // IN2 = 0

    gpio_set_level(MOTOR_2_PIN_1, 0);  // IN1 = 1
    gpio_set_level(MOTOR_2_PIN_2, 0);  // IN2 = 0

    ledc_timer_config(&ledc_timer_motor_1);
    ledc_channel_config(&ledc_channel_motor_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 256);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_timer_config(&ledc_timer_motor_2); // Ajout pour moteur 2
    ledc_channel_config(&ledc_channel_motor_2); // Ajout pour moteur 2
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 256); // Ajout pour moteur 2
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); // Ajout pour moteur 2

    event_bus_subscribe(EVENT_BLE_MOTORS_COMMAND, motors_command_handler);
    event_bus_subscribe(EVENT_BLE_JOYSTICK_DIRECTION, motors_direction_handler);
}
