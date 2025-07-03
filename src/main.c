#include "motor/motor.h"
#include "ble/ble.h"

void app_main() 
{
    printf("START ...\n");

    ble_init();
    motors_init();

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
