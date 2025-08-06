#include "ble.h"

static uint8_t adv_config_done = 0;

// UUID 16-bit pour service principal (Motor en priorité)
// Alternative : utiliser un seul service en advertising pour éviter l'erreur 102
static uint8_t adv_service_uuid16[2] = {
    // Service Motor (0x00FF) - little endian
    0xFF, 0x00
};

static uint8_t adv_uuid128[16] = {
    0xe7, 0xa1, 0x32, 0x91,
    0xe8, 0x55, 0x29, 0x83,
    0x2a, 0x4f, 0x34, 0x12,
    0xe6, 0x38, 0x7c, 0xf9
};

// Configuration minimale des données d'advertising
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
    .service_uuid_len = sizeof(adv_uuid128),
    .p_service_uuid = adv_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
};

// // Données de scan response pour le service Joystick
// static uint8_t scan_rsp_service_uuid16[2] = {
//     0xFE, 0x00  // Service Joystick (0x00FE)
// };

// static esp_ble_adv_data_t scan_rsp_data = {
//     .set_scan_rsp = true,
//     .include_name = false,
//     .include_txpower = false,
//     .service_uuid_len = sizeof(scan_rsp_service_uuid16),
//     .p_service_uuid = scan_rsp_service_uuid16,
//     .flag = 0
// };

// Paramètres d'advertising
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

// Tableau des profils GATT
static struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
} gl_profile_tab[BLE_APP_NUM] = {
    [BLE_MOTOR_APP_ID] = {
        .gatts_cb = ble_motor_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
    [BLE_JOYSTICK_APP_ID] = {
        .gatts_cb = ble_joystick_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    }
};

void ble_server_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // Gestion centralisée des événements globaux
    switch (event) {
        case ESP_GATTS_REG_EVT:
            if (ESP_GATT_OK == param->reg.status) {
                gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
            } else {
                ESP_LOGI(BLE_SERVER_TAG, "Reg app failed app_id %04x, status %d", param->reg.app_id, param->reg.status);
                return;
            }
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(BLE_SERVER_TAG, "Client connected, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x",
                param->connect.conn_id,
                param->connect.remote_bda[0], param->connect.remote_bda[1],
                param->connect.remote_bda[2], param->connect.remote_bda[3],
                param->connect.remote_bda[4], param->connect.remote_bda[5]);
                
            // Configurer les paramètres de connexion
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x30;
            conn_params.min_int = 0x10;
            conn_params.timeout = 400;
            esp_ble_gap_update_conn_params(&conn_params);
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_SERVER_TAG, "Client disconnected, remote %02x:%02x:%02x:%02x:%02x:%02x, reason 0x%02x",
                param->disconnect.remote_bda[0], param->disconnect.remote_bda[1],
                param->disconnect.remote_bda[2], param->disconnect.remote_bda[3],
                param->disconnect.remote_bda[4], param->disconnect.remote_bda[5],
                param->disconnect.reason);
            
            // Redémarrer l'advertising centralisé ici
            ble_server_on_disconnect();
            
            // Ne pas dispatcher cet événement aux profils pour éviter la duplication
            return;
            
        default:
            // Autres événements continuent vers les profils
            break;
    }

    // Dispatch des événements vers les handlers spécifiques des profils
    do {
        int idx;
        for(idx = 0; idx < BLE_APP_NUM; idx++) {
            if (ESP_GATT_IF_NONE == gatts_if || gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void ble_server_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~BLE_ADV_CONFIG_FLAG);
            if (0 == adv_config_done) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~BLE_SCAN_RSP_CONFIG_FLAG);
            if (0 == adv_config_done) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (ESP_BT_STATUS_SUCCESS != param->adv_start_cmpl.status) {
                ESP_LOGE(BLE_SERVER_TAG, "Advertising start failed");
            } else {
                ESP_LOGI(BLE_SERVER_TAG, "Advertising started successfully - Services: Motor(0x00FF), Joystick(0x00FE)");
            }
            break;
            
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(BLE_SERVER_TAG, "Connection params update: status=%d, conn_int=%d, latency=%d, timeout=%d", 
                param->update_conn_params.status, 
                param->update_conn_params.conn_int, 
                param->update_conn_params.latency, 
                param->update_conn_params.timeout);
            break;
            
        default:
            break;
    }
}

// Handler global pour les déconnexions - redémarre l'advertising
static void ble_server_handle_disconnect(void)
{
    ESP_LOGI(BLE_SERVER_TAG, "Device disconnected - restarting advertising");
    esp_ble_gap_start_advertising(&adv_params);
}

// Wrapper pour gérer les déconnexions depuis les profils
void ble_server_on_disconnect(void)
{
    ble_server_handle_disconnect();
}

void ble_server_init(void)
{
    ESP_LOGI(BLE_SERVER_TAG, "Initializing BLE Server with %d services", BLE_ADV_SERVICE_COUNT);
    
    // Initialisation NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialisation du contrôleur Bluetooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_SERVER_TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BLE_SERVER_TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Initialisation de la stack Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_SERVER_TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BLE_SERVER_TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // Enregistrement des callbacks
    ret = esp_ble_gatts_register_callback(ble_server_gatts_event_handler);
    if (ret) {
        ESP_LOGE(BLE_SERVER_TAG, "GATTS register callback failed, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(ble_server_gap_event_handler);
    if (ret) {
        ESP_LOGE(BLE_SERVER_TAG, "GAP register callback failed, error code = %x", ret);
        return;
    }

    // Configuration du nom du périphérique
    ret = esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
    if (ret) {
        ESP_LOGE(BLE_SERVER_TAG, "Set device name failed, error = 0x%x", ret);
        return;
    }

    // Initialisation des profils
    ble_motor_init();
    ble_joystick_init();

    // Enregistrement des applications GATT
    ret = ble_motor_register_app();
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SERVER_TAG, "Motor app registration failed");
        return;
    }

    ret = ble_joystick_register_app();
    if (ret != ESP_OK) {
        ESP_LOGE(BLE_SERVER_TAG, "Joystick app registration failed");
        return;
    }

    // Configuration des données d'advertising avec fallback
    ESP_LOGI(BLE_SERVER_TAG, "Configuring advertising data...");
    
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret) {
        ESP_LOGE(BLE_SERVER_TAG, "Config adv data failed, error code = 0x%x (%d)", ret, ret);
        return;
        // // Fallback: configuration minimale sans UUIDs
        // ESP_LOGW(BLE_SERVER_TAG, "Trying fallback configuration (name only)...");
        // esp_ble_adv_data_t fallback_adv_data = {
        //     .set_scan_rsp = false,
        //     .include_name = true,
        //     .include_txpower = false,
        //     .appearance = 0x00,
        //     .manufacturer_len = 0,
        //     .p_manufacturer_data = NULL,
        //     .service_data_len = 0,
        //     .p_service_data = NULL,
        //     .service_uuid_len = 0,  // Pas d'UUIDs
        //     .p_service_uuid = NULL,
        //     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT)
        // };
        
        // ret = esp_ble_gap_config_adv_data(&fallback_adv_data);
        // if (ret) {
        //     ESP_LOGE(BLE_SERVER_TAG, "Fallback config also failed, error code = 0x%x", ret);
        //     return;
        // } else {
        //     ESP_LOGW(BLE_SERVER_TAG, "Fallback advertising configured (no service UUIDs)");
        // }
    } else {
        ESP_LOGI(BLE_SERVER_TAG, "Advertising data configured successfully");
    }
    adv_config_done |= BLE_ADV_CONFIG_FLAG;

    // // Configuration des données de scan response (pour le service Joystick) - optionnel
    // ESP_LOGI(BLE_SERVER_TAG, "Configuring scan response data...");
    // ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
    // if (ret) {
    //     ESP_LOGW(BLE_SERVER_TAG, "Config scan response data failed, error code = 0x%x", ret);
    //     ESP_LOGW(BLE_SERVER_TAG, "Continuing without scan response data...");
    //     // Ne pas retourner, continuer sans scan response
    // } else {
    //     ESP_LOGI(BLE_SERVER_TAG, "Scan response data configured successfully");
    //     adv_config_done |= BLE_SCAN_RSP_CONFIG_FLAG;
    // }

    // MOTOR UUID TODO
    // static uint8_t service1_uuid[16] = {
    //     0xe0, 0xf0, 0x9f, 0x6d,
    //     0xda, 0x1b, 0xa1, 0x84,
    //     0x0b, 0x4a, 0x96, 0x84,
    //     0xfc, 0xd3, 0xe8, 0x31
    // };
    // static uint8_t service1_uuid[16] = {
    //     0xe0, 0xf0, 0x9f, 0x6d,
    //     0xda, 0x1b, 0xa1, 0x84,
    //     0x0b, 0x4a, 0x96, 0x84,
    //     0xfc, 0xd3, 0xe8, 0x31
    // };

    // JOYSTICK UUID TODO
    //static uint8_t service2_uuid[16] = {
    //     0x7b, 0x7d, 0xaa, 0x59,
    //     0xf1, 0xe5, 0xb6, 0x83,
    //     0x54, 0x40, 0x95, 0x67,
    //     0xd7, 0x68, 0x15, 0x7c
    // };
    // static uint8_t char2_x_uuid[16] = {
    //     0x11, 0xc0, 0x36, 0x2c,
    //     0xc7, 0x41, 0xdb, 0xb5,
    //     0x09, 0x43, 0x5d, 0x78,
    //     0xfc, 0x3b, 0x1b, 0xb4
    // };
    // static uint8_t char2_y_uuid[16] = {
    //     0xa9, 0xc0, 0x2d, 0x6e,
    //     0x28, 0x96, 0xb1, 0x9d,
    //     0x74, 0x4b, 0x6e, 0x51,
    //     0x6e, 0xab, 0xe1, 0xdb
    // };
    

    ESP_LOGI(BLE_SERVER_TAG, "BLE Server initialization complete!");
    ESP_LOGI(BLE_SERVER_TAG, "Available services: Motor (0x%04X), Joystick (0x%04X)", ble_motor_get_service_uuid(), ble_joystick_get_service_uuid());
}