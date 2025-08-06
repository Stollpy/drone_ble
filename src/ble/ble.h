// BLE Server - Configuration générale
#ifndef BLE_H
#define BLE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "config.h"
#include "event/event.h"
#include "ble_motor.h"
#include "ble_joystick.h"

// BLE Server Configuration
#define BLE_SERVER_TAG "BLE_SERVER"
#define BLE_DEVICE_NAME "stollpy_drone"
#define BLE_APP_NUM 2

// Advertising configuration flags
#define BLE_ADV_CONFIG_FLAG (1 << 0)
#define BLE_SCAN_RSP_CONFIG_FLAG (1 << 1)

// Number of services to advertise
#define BLE_ADV_SERVICE_COUNT 2

// BLE Server Functions
void ble_server_init(void);
void ble_server_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void ble_server_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// Connection management (deprecated - handled centrally now)
void ble_server_on_disconnect(void);

#endif // BLE_H 