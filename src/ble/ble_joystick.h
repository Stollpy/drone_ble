// BLE Joystick Profile
#ifndef BLE_JOYSTICK_H
#define BLE_JOYSTICK_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "config.h"
#include "event/event.h"

// Joystick BLE Profile constants
#define BLE_JOYSTICK_APP_ID 1
#define BLE_JOYSTICK_CHAR_VAL_LEN_MAX 0x08  // 4 bytes pour X + 4 bytes pour Y
#define BLE_JOYSTICK_SERVICE_UUID 0x00FE
#define BLE_JOYSTICK_X_CHARACTERISTIC_UUID 0xFE01
#define BLE_JOYSTICK_Y_CHARACTERISTIC_UUID 0xFE02
#define BLE_JOYSTICK_HANDLE 0x08

// Joystick profile structure
struct ble_joystick_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    // Caractéristique X
    uint16_t char_x_handle;
    esp_bt_uuid_t char_x_uuid;
    // Caractéristique Y
    uint16_t char_y_handle;
    esp_bt_uuid_t char_y_uuid;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

// Function declarations
void ble_joystick_init(void);
void ble_joystick_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void ble_joystick_update_value(char axis, int32_t value);
void ble_joystick_on_direction_event(event_t *event);
esp_err_t ble_joystick_register_app(void);
uint16_t ble_joystick_get_service_uuid(void);

#endif // BLE_JOYSTICK_H