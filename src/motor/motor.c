#include "motor/motor.h"
#include "config.h"

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

void motors_init() {
    gpio_set_direction(MOTOR_1_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_1_PIN_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_1_ENABLE_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(MOTOR_2_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_PIN_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_2_ENABLE_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(MOTOR_1_PIN_1, 1);  // IN1 = 1
    gpio_set_level(MOTOR_1_PIN_2, 0);  // IN2 = 0

    gpio_set_level(MOTOR_2_PIN_1, 1);  // IN1 = 1
    gpio_set_level(MOTOR_2_PIN_2, 0);  // IN2 = 0

    ledc_timer_config(&ledc_timer_motor_1);
    ledc_channel_config(&ledc_channel_motor_1);

    ledc_timer_config(&ledc_timer_motor_2); // Ajout pour moteur 2
    ledc_channel_config(&ledc_channel_motor_2); // Ajout pour moteur 2

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 256);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 256); // Ajout pour moteur 2
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); // Ajout pour moteur 2
}