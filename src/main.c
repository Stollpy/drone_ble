#include "motor/motor.h"
#include "ble/ble.h"
#include "joystick/joystick.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main() 
{
    printf("START ...\n");

    motors_init();
    printf("Motors initialized\n");
    vTaskDelay(pdMS_TO_TICKS(100));

    ble_init();
    printf("BLE initialized\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    joystick_init();
    printf("Joystick initialized\n");

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}