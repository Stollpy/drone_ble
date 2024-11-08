#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BLE_SERVER"
#define SERVICE_UUID 0x00FF
#define CHARACTERISTIC_UUID 0xFF01

static uint8_t char_value = 0;

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_WRITE_EVT:
            if (param->write.handle == CHARACTERISTIC_UUID) {
                char_value = param->write.value[0];
                if (char_value == 1) {
                    ESP_LOGI(TAG, "Appui sur le bouton détecté !");
                }
            }
            break;
        default:
            break;
    }
}

// MAC: d0:ef:76:1e:5d:ec
// Bluetooth MAC: d0:ef:76:1e:5d:ee
void app_main() {
    printf("START ...\n");

   // CONFIG BLUETOOTH
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_set_device_name("stollpy_drone");

    ESP_LOGI(TAG, "Serveur BLE prêt !");

    // printf("ENABLE PIN %d ...\n", GPIO_NUM_4);
    
    // gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    
    // printf("%d \n", gpio_set_level(GPIO_NUM_4, 1));

    // vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // printf("DISABLE PIN %d ...\n", GPIO_NUM_4);

    // printf("%d \n", gpio_set_level(GPIO_NUM_4, 0));

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
