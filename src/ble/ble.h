// BLE
#ifndef BLE_H
#define BLE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

// BLE
#define GATTS_TAG "BLE_SERVER"
#define APP_NUM 2

// MOTOR BLE
#define APP_MOTOR_ID 0
#define APP_JOYSTICK_ID 1
#define APP_MOTOR_CHAR_VAL_LEN_MAX 0x01
#define MOTOR_SERVICE_UUID 0x00FF
#define MOTOR_CHARACTERISTIC_UUID 0xFF01
#define MOTOR_DESCR_UUID 0x3333
#define MOTOR_HANDLE 0x04

// JOYSTICK BLE
#define APP_JOYSTICK_CHAR_VAL_LEN_MAX 0x08  // 4 bytes pour X + 4 bytes pour Y
#define JOYSTICK_SERVICE_UUID 0x00FE
#define JOYSTICK_X_CHARACTERISTIC_UUID 0xFE01
#define JOYSTICK_Y_CHARACTERISTIC_UUID 0xFE02
#define JOYSTICK_HANDLE 0x08
#define MOTOR_STATE_START 0x01
#define MOTOR_STATE_STOP 0x00
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

void gatts_app_motor_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gatts_app_joystick_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void ble_update_joystick_value(char axis, int32_t value);
void ble_joystick_event_handler(event_t *event);
void ble_init(void);

#endif 