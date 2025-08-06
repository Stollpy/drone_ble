// BLE Motor Profile
#ifndef BLE_MOTOR_H
#define BLE_MOTOR_H

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

// Motor BLE Profile constants
#define BLE_MOTOR_APP_ID 0
#define BLE_MOTOR_CHAR_VAL_LEN_MAX 0x01
#define BLE_MOTOR_HANDLE 0x04

// Motor 128-bit UUIDs
extern uint8_t motor_service_uuid[16];
extern uint8_t motor_char_uuid[16];

// Motor states
#define BLE_MOTOR_STATE_START 0x01
#define BLE_MOTOR_STATE_STOP 0x00

// Motor profile structure
struct ble_motor_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

// Function declarations
void ble_motor_init(void);
void ble_motor_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
esp_err_t ble_motor_register_app(void);
uint8_t* ble_motor_get_service_uuid(void);

#endif // BLE_MOTOR_H