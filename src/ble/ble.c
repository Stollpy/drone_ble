#include "ble/ble.h"

static uint8_t adv_config_done = 0;

static uint8_t motor_state = MOTOR_STATE_STOP;
static esp_attr_value_t motor_char_val = {
    .attr_max_len = APP_MOTOR_CHAR_VAL_LEN_MAX,
    .attr_len = sizeof(motor_state),
    .attr_value = &motor_state
};

// Variables pour le joystick
static int32_t joystick_x = 0;
static int32_t joystick_y = 0;
static esp_attr_value_t joystick_x_char_val = {
    .attr_max_len = sizeof(int32_t),
    .attr_len = sizeof(int32_t),
    .attr_value = (uint8_t*)&joystick_x
};
static esp_attr_value_t joystick_y_char_val = {
    .attr_max_len = sizeof(int32_t),
    .attr_len = sizeof(int32_t),
    .attr_value = (uint8_t*)&joystick_y
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

// Structure étendue pour le joystick avec 2 caractéristiques
struct gatts_joystick_profile_inst
{
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

// static esp_ble_adv_data_t scan_rsp_data = {
//     .set_scan_rsp = true,
//     .include_name = true,
//     .include_txpower = true,
//     // .min_interval = 0x0006,
//     // .max_interval = 0x0010,
//     .appearance = 0x00,
//     .manufacturer_len = 0,
//     .p_manufacturer_data = NULL,
//     .service_data_len = 0,
//     .p_service_data = NULL,
//     .service_uuid_len = sizeof(adv_service_uuid128),
//     .p_service_uuid = adv_service_uuid128,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
// };

static struct gatts_profile_inst gl_profile_tab[APP_NUM] = {
    [APP_MOTOR_ID] = {
        .gatts_cb = gatts_app_motor_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
    [APP_JOYSTICK_ID] = {
        .gatts_cb = gatts_app_joystick_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    }
};

// Profil spécifique pour le joystick
static struct gatts_joystick_profile_inst gl_joystick_profile;


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
           
            ESP_LOGI(GATTS_TAG, "CREATE SERVICE EVT: service started");
            
            esp_err_t add_char_ret = esp_ble_gatts_add_char(
                gl_profile_tab[APP_MOTOR_ID].service_handle,
                &gl_profile_tab[APP_MOTOR_ID].char_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE, // | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                &motor_char_val,
                NULL);

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
            gl_profile_tab[APP_MOTOR_ID].descr_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d", param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            break;
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
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16, param->read.conn_id, param->read.trans_id, param->read.handle);

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = APP_MOTOR_CHAR_VAL_LEN_MAX;
            rsp.attr_value.value[0] = motor_state;
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16, param->write.conn_id, param->write.trans_id, param->write.handle);

            if (!param->write.is_prep) {
                ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);

                if (param->write.handle == gl_profile_tab[APP_MOTOR_ID].char_handle && param->write.len == 1) {
                    motor_state = param->write.value[0];

                    if (motor_state == 0) {
                        ESP_LOGI(GATTS_TAG, "Received value: 0 - Motor stopped");
                    } else if (motor_state == 1) {
                        ESP_LOGI(GATTS_TAG, "Received value: 1 - Motor started");
                    } else {
                        ESP_LOGW(GATTS_TAG, "Invalid value received: %d", motor_state);
                    }

                    // TODO: Implement motor control by Bus
                    // gpio_set_level(MOTOR_1_PIN_1, motor_state);

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
                    ESP_LOGW(GATTS_TAG, "Write event not targeting the correct handle or invalid length");
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_INVALID_HANDLE, NULL);

                }
            }
            break;
        }
        default:
            break;
    }
}

void gatts_app_joystick_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "JOYSTICK REGISTER_APP_EVT: status %d, app_id %d", param->reg.status, param->reg.app_id);
            
            gl_joystick_profile.service_id.is_primary = true;
            gl_joystick_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_joystick_profile.service_id.id.uuid.uuid.uuid16 = JOYSTICK_SERVICE_UUID;
            
            esp_ble_gatts_create_service(gatts_if, &gl_joystick_profile.service_id, JOYSTICK_HANDLE);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(GATTS_TAG, "JOYSTICK CREATE SERVICE EVT: status %d, service handle: %d", param->create.status, param->create.service_handle);
            
            gl_joystick_profile.service_handle = param->create.service_handle;
            
            // Configuration de la caractéristique X
            gl_joystick_profile.char_x_uuid.len = ESP_UUID_LEN_16;
            gl_joystick_profile.char_x_uuid.uuid.uuid16 = JOYSTICK_X_CHARACTERISTIC_UUID;

            esp_ble_gatts_start_service(gl_joystick_profile.service_handle);
            
            // Ajouter la caractéristique X
            esp_err_t add_char_x_ret = esp_ble_gatts_add_char(
                gl_joystick_profile.service_handle,
                &gl_joystick_profile.char_x_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                &joystick_x_char_val,
                NULL);

            if (add_char_x_ret) {
                ESP_LOGE(GATTS_TAG, "add char X failed, error code = %x", add_char_x_ret);
            }
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(GATTS_TAG, "JOYSTICK ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d", 
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            
            // Identifier quelle caractéristique a été ajoutée
            if (param->add_char.char_uuid.uuid.uuid16 == JOYSTICK_X_CHARACTERISTIC_UUID) {
                gl_joystick_profile.char_x_handle = param->add_char.attr_handle;
                ESP_LOGI(GATTS_TAG, "Caractéristique X ajoutée, handle: %d", gl_joystick_profile.char_x_handle);
                
                // Ajouter maintenant la caractéristique Y
                gl_joystick_profile.char_y_uuid.len = ESP_UUID_LEN_16;
                gl_joystick_profile.char_y_uuid.uuid.uuid16 = JOYSTICK_Y_CHARACTERISTIC_UUID;
                
                esp_err_t add_char_y_ret = esp_ble_gatts_add_char(
                    gl_joystick_profile.service_handle,
                    &gl_joystick_profile.char_y_uuid,
                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                    &joystick_y_char_val,
                    NULL);

                if (add_char_y_ret) {
                    ESP_LOGE(GATTS_TAG, "add char Y failed, error code = %x", add_char_y_ret);
                }
            } else if (param->add_char.char_uuid.uuid.uuid16 == JOYSTICK_Y_CHARACTERISTIC_UUID) {
                gl_joystick_profile.char_y_handle = param->add_char.attr_handle;
                ESP_LOGI(GATTS_TAG, "Caractéristique Y ajoutée, handle: %d", gl_joystick_profile.char_y_handle);
            }
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "JOYSTICK ESP_GATTS_CONNECT_EVT, conn_id %d", param->connect.conn_id);
            gl_joystick_profile.conn_id = param->connect.conn_id;
            break;
            
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(GATTS_TAG, "JOYSTICK GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", 
                param->read.conn_id, param->read.trans_id, param->read.handle);

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            
            rsp.attr_value.handle = param->read.handle;
            
            if (param->read.handle == gl_joystick_profile.char_x_handle) {
                rsp.attr_value.len = sizeof(int32_t);
                memcpy(rsp.attr_value.value, &joystick_x, sizeof(int32_t));
                ESP_LOGI(GATTS_TAG, "Lecture X: %" PRIx32 "\n", joystick_x);
            } else if (param->read.handle == gl_joystick_profile.char_y_handle) {
                rsp.attr_value.len = sizeof(int32_t);
                memcpy(rsp.attr_value.value, &joystick_y, sizeof(int32_t));
                ESP_LOGI(GATTS_TAG, "Lecture %" PRIx32 "\n", joystick_y);
            }
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
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
    } else if (ESP_GATTS_DISCONNECT_EVT == event) {
        ESP_LOGI(GATTS_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
        ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        return;
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
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (ESP_BT_STATUS_SUCCESS != param->adv_start_cmpl.status) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed");
            } else {
                ESP_LOGE(GATTS_TAG, "Advertising started with successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d", 
                param->update_conn_params.status, 
                param->update_conn_params.conn_int, 
                param->update_conn_params.latency, 
                param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

// Fonction pour mettre à jour et notifier les valeurs du joystick
void ble_update_joystick_value(char axis, int32_t value)
{
    if (axis == 'x') {
        joystick_x = value;
        
        // Envoyer notification si connecté
        if (gl_joystick_profile.gatts_if != ESP_GATT_IF_NONE && gl_joystick_profile.conn_id != 0) {
            esp_ble_gatts_send_indicate(
                gl_joystick_profile.gatts_if,
                gl_joystick_profile.conn_id,
                gl_joystick_profile.char_x_handle,
                sizeof(int32_t),
                (uint8_t*)&joystick_x,
                false
            );
        }
    } else if (axis == 'y') {
        joystick_y = value;
        
        // Envoyer notification si connecté
        if (gl_joystick_profile.gatts_if != ESP_GATT_IF_NONE && gl_joystick_profile.conn_id != 0) {
            esp_ble_gatts_send_indicate(
                gl_joystick_profile.gatts_if,
                gl_joystick_profile.conn_id,
                gl_joystick_profile.char_y_handle,
                sizeof(int32_t),
                (uint8_t*)&joystick_y,
                false
            );
        }
    }
}

// Handler d'événements pour traiter les événements du joystick
void ble_joystick_event_handler(event_t *event)
{
    if (event->type == EVENT_BLE_JOYSTICK_DIRECTION) {
        ble_update_joystick_value(
            event->data.ble_joystick_direction.axe,
            event->data.ble_joystick_direction.position
        );
    }
}

// MAC: d0:ef:76:1e:5d:ec
// Bluetooth MAC: d0:ef:76:1e:5d:ee
void ble_init() 
{
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

    ret = esp_ble_gatts_app_register(APP_JOYSTICK_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "cannot of register app joystick. error code = %x", ret);
        return;
    }

    // // Enregistrer le handler d'événements pour les mises à jour du joystick
    // event_bus_subscribe(EVENT_BLE_JOYSTICK_DIRECTION, ble_joystick_event_handler);

    ESP_LOGI(GATTS_TAG, "Serveur BLE is ready!");
}