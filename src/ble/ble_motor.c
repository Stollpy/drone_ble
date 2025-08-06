#include "ble_motor.h"

static const char* BLE_MOTOR_TAG = "BLE_MOTOR";

// Motor 128-bit UUIDs based on custom base
uint8_t motor_service_uuid[16] = {
    0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00,
    0x6d, 0x9f, 0xf0, 0xe0
};

uint8_t motor_char_uuid[16] = {
    0x01, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00,
    0x6d, 0x9f, 0xf0, 0xe0
};

// Motor state and characteristics
static uint8_t motor_state = BLE_MOTOR_STATE_STOP;
static esp_attr_value_t motor_char_val = {
    .attr_max_len = BLE_MOTOR_CHAR_VAL_LEN_MAX,
    .attr_len = sizeof(motor_state),
    .attr_value = &motor_state
};

// Motor profile instance
static struct ble_motor_profile_inst motor_profile = {
    .gatts_cb = ble_motor_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,
    .app_id = BLE_MOTOR_APP_ID
};

void ble_motor_init(void)
{
    ESP_LOGI(BLE_MOTOR_TAG, "Initializing Motor BLE profile");
}

esp_err_t ble_motor_register_app(void)
{
    esp_err_t ret = esp_ble_gatts_app_register(BLE_MOTOR_APP_ID);
    if (ret) {
        ESP_LOGE(BLE_MOTOR_TAG, "Motor app register failed, error code = %x", ret);
        return ret;
    }
    ESP_LOGI(BLE_MOTOR_TAG, "Motor app registration initiated");
    return ESP_OK;
}

uint8_t* ble_motor_get_service_uuid(void)
{
    return motor_service_uuid;
}

void ble_motor_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(BLE_MOTOR_TAG, "REGISTER_APP_EVT: status %d, app_id %d", param->reg.status, param->reg.app_id);
            
            motor_profile.gatts_if = gatts_if;
            motor_profile.service_id.is_primary = true;
            motor_profile.service_id.id.uuid.len = ESP_UUID_LEN_128;
            memcpy(motor_profile.service_id.id.uuid.uuid.uuid128, motor_service_uuid, ESP_UUID_LEN_128);
            
            esp_ble_gatts_create_service(gatts_if, &motor_profile.service_id, BLE_MOTOR_HANDLE);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(BLE_MOTOR_TAG, "CREATE SERVICE EVT: status %d, service handle: %d", param->create.status, param->create.service_handle);
            
            motor_profile.service_handle = param->create.service_handle;
            motor_profile.char_uuid.len = ESP_UUID_LEN_128;
            memcpy(motor_profile.char_uuid.uuid.uuid128, motor_char_uuid, ESP_UUID_LEN_128);

            esp_ble_gatts_start_service(motor_profile.service_handle);
           
            ESP_LOGI(BLE_MOTOR_TAG, "Motor service started");
            
            esp_err_t add_char_ret = esp_ble_gatts_add_char(
                motor_profile.service_handle,
                &motor_profile.char_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                &motor_char_val,
                NULL);

            if (add_char_ret) {
                ESP_LOGE(BLE_MOTOR_TAG, "add char failed, error code = %x", add_char_ret);
            }
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t length = 0;
            const uint8_t *prf_char;

            ESP_LOGI(BLE_MOTOR_TAG, "ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d", 
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
                
            motor_profile.char_handle = param->add_char.attr_handle;
            motor_profile.service_handle = param->add_char.service_handle;
            // motor_profile.char_uuid.uuid.uuid16 = param->add_char.char_uuid.uuid.uuid16;
            
            esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
            if (get_attr_ret == ESP_FAIL) {
                ESP_LOGE(BLE_MOTOR_TAG, "ILLEGAL HANDLE");
            }

            ESP_LOGI(BLE_MOTOR_TAG, "Motor char length = %x", length);

            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
                motor_profile.service_handle,
                &motor_profile.descr_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL, NULL
            );

            if (add_descr_ret) {
                ESP_LOGE(BLE_MOTOR_TAG, "Add char descr failed, error code = %x", add_descr_ret);
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            motor_profile.descr_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(BLE_MOTOR_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d", 
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(BLE_MOTOR_TAG, "Motor profile connected, conn_id %d", param->connect.conn_id);
            motor_profile.conn_id = param->connect.conn_id;
            break;
            
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(BLE_MOTOR_TAG, "GATT_READ_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16, 
                param->read.conn_id, param->read.trans_id, param->read.handle);

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = BLE_MOTOR_CHAR_VAL_LEN_MAX;
            rsp.attr_value.value[0] = motor_state;
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(BLE_MOTOR_TAG, "GATT_WRITE_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16, 
                param->write.conn_id, param->write.trans_id, param->write.handle);

            if (!param->write.is_prep) {
                ESP_LOGI(BLE_MOTOR_TAG, "Motor write value len %d", param->write.len);

                if (param->write.handle == motor_profile.char_handle && param->write.len == 1) {
                    motor_state = param->write.value[0];

                    if (motor_state == 0) {
                        ESP_LOGI(BLE_MOTOR_TAG, "Motor command: STOP");
                    } else if (motor_state == 1) {
                        ESP_LOGI(BLE_MOTOR_TAG, "Motor command: START");
                    } else {
                        ESP_LOGW(BLE_MOTOR_TAG, "Invalid motor command: %d", motor_state);
                    }

                    // Publier l'événement moteur
                    event_t event = {
                        .type = EVENT_BLE_MOTORS_COMMAND,
                        .data = {
                            .ble_motors_command = {
                                .type = motor_state
                            }
                        }
                    };

                    event_bus_publish(&event);
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                } else {
                    ESP_LOGW(BLE_MOTOR_TAG, "Write event not targeting correct handle or invalid length");
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_INVALID_HANDLE, NULL);
                }
            }
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_MOTOR_TAG, "Motor profile disconnected");
            break;
            
        default:
            break;
    }
}