#ifndef CONFIG_H
#define CONFIG_H

#define DEVICE_NAME "stollpy_drone"
#define GATTS_TAG "BLE_SERVER"
#define APP_NUM 1

// BLE
#define APP_MOTOR_ID 0
#define APP_MOTOR_CHAR_VAL_LEN_MAX 0x01
#define MOTOR_SERVICE_UUID 0x00FF
#define MOTOR_CHARACTERISTIC_UUID 0xFF01
#define MOTOR_DESCR_UUID 0x3333
#define MOTOR_HANDLE 0x04
#define MOTOR_STATE_START 0x01
#define MOTOR_STATE_STOP 0x00
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

// MOTOR 1
// #define MOTOR_1_PIN GPIO_NUM_4
#define MOTOR_1_PIN_1 GPIO_NUM_4
#define MOTOR_1_PIN_2 GPIO_NUM_17
#define MOTOR_1_ENABLE_PIN GPIO_NUM_16

// MOTOR 2
#define MOTOR_2_PIN_1 GPIO_NUM_25
#define MOTOR_2_PIN_2 GPIO_NUM_26
#define MOTOR_2_ENABLE_PIN GPIO_NUM_33

#endif