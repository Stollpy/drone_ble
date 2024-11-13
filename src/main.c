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
#include <string.h>

#define DEVICE_NAME "stollpy_drone"
#define GATTS_TAG "BLE_SERVER"
#define APP_NUM 1

#define APP_MOTOR_ID 0
#define APP_MOTOR_CHAR_VAL_LEN_MAX 0x40
#define MOTOR_SERVICE_UUID 0x00FF
#define MOTOR_CHARACTERISTIC_UUID 0xFF01
#define MOTOR_DESCR_UUID 0x3333
#define MOTOR_HANDLE 4
#define MOTOR_STATE_START 1
#define MOTOR_STATE_STOP 0


#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

void gatts_app_motor_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static uint8_t adv_config_done = 0;
static uint8_t motor_char_base_state = MOTOR_STATE_STOP;
static esp_gatt_char_prop_t motor_property = 0;

static esp_attr_value_t motor_char_val = {
    .attr_max_len = APP_MOTOR_CHAR_VAL_LEN_MAX,
    .attr_len = sizeof(MOTOR_STATE_STOP),
    .attr_value = MOTOR_STATE_STOP
};

// Prefix 128 bits: 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00
static uint8_t adv_service_uuid128[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00
};

struct gatts_profile_inst
{
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

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    // .min_interval = 0x0006,
    // .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
};

static struct gatts_profile_inst gl_profile_tab[APP_NUM] = {
    [APP_MOTOR_ID] = {
        .gatts_cb = gatts_app_motor_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    }
};


void gatts_app_motor_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) 
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT: status %d, app_id %d", param->reg.status, param->reg.app_id);
            
            gl_profile_tab[APP_MOTOR_ID].service_id.is_primary = true;
            gl_profile_tab[APP_MOTOR_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[APP_MOTOR_ID].service_id.id.uuid.uuid.uuid16 = MOTOR_SERVICE_UUID;
            
            esp_ble_gap_set_device_name(DEVICE_NAME);

            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) {
                ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            
            adv_config_done |= adv_config_flag;
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[APP_MOTOR_ID].service_id, MOTOR_HANDLE);
            break;
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(GATTS_TAG, "CREATE SERVICE EVT: status %d, service handle: %d", param->create.status, param->create.service_handle);
            
            gl_profile_tab[APP_MOTOR_ID].service_handle = param->create.service_handle;
            gl_profile_tab[APP_MOTOR_ID].char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[APP_MOTOR_ID].char_uuid.uuid.uuid16 = MOTOR_CHARACTERISTIC_UUID;

            esp_ble_gatts_start_service(gl_profile_tab[APP_MOTOR_ID].service_handle);

            int motor_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE; // | ESP_GATT_CHAR_PROP_BIT_NOTIFY
            
            esp_err_t add_char_ret = esp_ble_gatts_add_char(
                gl_profile_tab[APP_MOTOR_ID].service_handle,
                &gl_profile_tab[APP_MOTOR_ID].char_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                motor_property,
                &motor_char_val,
                NULL
            );

            if (add_char_ret) {
                ESP_LOGE(GATTS_TAG, "add char failed, error code = %x", add_char_ret);
            }
            break;
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t length = 0;
            const uint8_t *prf_char;

            ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d, attr_handle %d, servive_handle %d", param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            gl_profile_tab[APP_MOTOR_ID].char_handle = param->add_char.attr_handle;
            gl_profile_tab[APP_MOTOR_ID].service_handle = param->add_char.service_handle;
            gl_profile_tab[APP_MOTOR_ID].char_uuid.uuid.uuid16 = param->add_char.char_uuid.uuid.uuid16;
            
            esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
            if (get_attr_ret == ESP_FAIL) {
                ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
            }

            ESP_LOGI(GATTS_TAG, "The gatts demo char length = %x", length);

            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
                gl_profile_tab[APP_MOTOR_ID].service_handle,
                &gl_profile_tab[APP_MOTOR_ID].descr_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL, NULL
            );

            if (add_descr_ret) {
                ESP_LOGE(GATTS_TAG, "Add char descr failed, error code = %x", add_descr_ret);
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d", param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        case ESP_GATTS_CONNECT_EVT: {
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            
            conn_params.latency = 0;
            conn_params.max_int = 0x30;
            conn_params.min_int = 0x10;
            conn_params.timeout = 400;

            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x",
                param->connect.conn_id,
                param->connect.remote_bda[0],
                param->connect.remote_bda[1],
                param->connect.remote_bda[2],
                param->connect.remote_bda[3],
                param->connect.remote_bda[4],
                param->connect.remote_bda[5]
            );

            gl_profile_tab[APP_MOTOR_ID].conn_id = param->connect.conn_id;

            esp_ble_gap_update_conn_params(&conn_params);
            break;
        }
        default:
            break;
    }
}


void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) 
{
    if (ESP_GATTS_REG_EVT == event) {
        if (ESP_GATT_OK == param->reg.status) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for(idx = 0; idx < APP_NUM; idx++) {
            if (ESP_GATT_IF_NONE == gatts_if || gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
    
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) 
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~adv_config_flag);
            if (0 == adv_config_done) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~scan_rsp_config_flag);
            if (0 == adv_config_done) {
                esp_ble_gap_start_advertising(&adv_params);
            }
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (ESP_BT_STATUS_SUCCESS != param->adv_start_cmpl.status) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed");
            } else {
                ESP_LOGE(GATTS_TAG, "Advertising started with successfully");
            }
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d", 
                param->update_conn_params.status, 
                param->update_conn_params.conn_int, 
                param->update_conn_params.latency, 
                param->update_conn_params.timeout);
        default:
            break;
    }
}

// MAC: d0:ef:76:1e:5d:ec
// Bluetooth MAC: d0:ef:76:1e:5d:ee
void app_main() 
{
    printf("START ...\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s  enable controller bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gatts register handler failed. error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "gap register handler failed. error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(APP_MOTOR_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "cannot of register app motor. error code = %x", ret);
        return;
    }

    ESP_LOGI(GATTS_TAG, "Serveur BLE is ready!");


    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

    // printf("ENABLE PIN %d ...\n", GPIO_NUM_4);
    
    // gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    
    // printf("%d \n", gpio_set_level(GPIO_NUM_4, 1));

    // vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // printf("DISABLE PIN %d ...\n", GPIO_NUM_4);

    // printf("%d \n", gpio_set_level(GPIO_NUM_4, 0));
