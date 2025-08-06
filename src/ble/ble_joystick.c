#include "ble_joystick.h"

static const char* BLE_JOYSTICK_TAG = "BLE_JOYSTICK";

// Joystick 128-bit UUIDs based on custom base
uint8_t joystick_service_uuid[16] = {
    0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x02, 0x00,
    0x6d, 0x9f, 0xf0, 0xe0
};

uint8_t joystick_char_x_uuid[16] = {
    0x01, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x02, 0x00,
    0x6d, 0x9f, 0xf0, 0xe0
};

uint8_t joystick_char_y_uuid[16] = {
    0x02, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x02, 0x00,
    0x6d, 0x9f, 0xf0, 0xe0
};

// Joystick state and characteristics
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

// Joystick profile instance
static struct ble_joystick_profile_inst joystick_profile = {
    .gatts_cb = ble_joystick_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,
    .app_id = BLE_JOYSTICK_APP_ID
};

void ble_joystick_init(void)
{
    ESP_LOGI(BLE_JOYSTICK_TAG, "Initializing Joystick BLE profile");
    
    // S'abonner aux événements de direction du joystick
    event_bus_subscribe(EVENT_BLE_JOYSTICK_DIRECTION, ble_joystick_on_direction_event);
}

esp_err_t ble_joystick_register_app(void)
{
    esp_err_t ret = esp_ble_gatts_app_register(BLE_JOYSTICK_APP_ID);
    if (ret) {
        ESP_LOGE(BLE_JOYSTICK_TAG, "Joystick app register failed, error code = %x", ret);
        return ret;
    }
    ESP_LOGI(BLE_JOYSTICK_TAG, "Joystick app registration initiated");
    return ESP_OK;
}

uint8_t* ble_joystick_get_service_uuid(void)
{
    return joystick_service_uuid;
}

void ble_joystick_update_value(char axis, int32_t value)
{
    if (axis == 'x') {
        joystick_x = value;
        
        // Envoyer notification si connecté
        if (joystick_profile.gatts_if != ESP_GATT_IF_NONE && joystick_profile.conn_id != 0) {
            esp_ble_gatts_send_indicate(
                joystick_profile.gatts_if,
                joystick_profile.conn_id,
                joystick_profile.char_x_handle,
                sizeof(int32_t),
                (uint8_t*)&joystick_x,
                false
            );
        }
    } else if (axis == 'y') {
        joystick_y = value;
        
        // Envoyer notification si connecté
        if (joystick_profile.gatts_if != ESP_GATT_IF_NONE && joystick_profile.conn_id != 0) {
            esp_ble_gatts_send_indicate(
                joystick_profile.gatts_if,
                joystick_profile.conn_id,
                joystick_profile.char_y_handle,
                sizeof(int32_t),
                (uint8_t*)&joystick_y,
                false
            );
        }
    }
}

void ble_joystick_on_direction_event(event_t *event)
{
    if (event->type == EVENT_BLE_JOYSTICK_DIRECTION) {
        ble_joystick_update_value(
            event->data.ble_joystick_direction.axe,
            event->data.ble_joystick_direction.position
        );
    }
}

void ble_joystick_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(BLE_JOYSTICK_TAG, "REGISTER_APP_EVT: status %d, app_id %d", param->reg.status, param->reg.app_id);
            
            joystick_profile.gatts_if = gatts_if;
            joystick_profile.service_id.is_primary = true;
            joystick_profile.service_id.id.uuid.len = ESP_UUID_LEN_128;
            memcpy(joystick_profile.service_id.id.uuid.uuid.uuid128, joystick_service_uuid, ESP_UUID_LEN_128);
            
            esp_ble_gatts_create_service(gatts_if, &joystick_profile.service_id, BLE_JOYSTICK_HANDLE);
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(BLE_JOYSTICK_TAG, "CREATE SERVICE EVT: status %d, service handle: %d", param->create.status, param->create.service_handle);
            
            joystick_profile.service_handle = param->create.service_handle;
            
            // Configuration de la caractéristique X
            joystick_profile.char_x_uuid.len = ESP_UUID_LEN_128;
            memcpy(joystick_profile.char_x_uuid.uuid.uuid128, joystick_char_x_uuid, ESP_UUID_LEN_128);

            esp_ble_gatts_start_service(joystick_profile.service_handle);
            
            // Ajouter la caractéristique X
            esp_err_t add_char_x_ret = esp_ble_gatts_add_char(
                joystick_profile.service_handle,
                &joystick_profile.char_x_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                &joystick_x_char_val,
                NULL);

            if (add_char_x_ret) {
                ESP_LOGE(BLE_JOYSTICK_TAG, "add char X failed, error code = %x", add_char_x_ret);
            }
            break;
            
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(BLE_JOYSTICK_TAG, "ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d", 
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            
            // Identifier quelle caractéristique a été ajoutée
            if (memcmp(param->add_char.char_uuid.uuid.uuid128, joystick_char_x_uuid, ESP_UUID_LEN_128) == 0) {
                joystick_profile.char_x_handle = param->add_char.attr_handle;
                ESP_LOGI(BLE_JOYSTICK_TAG, "Caractéristique X ajoutée, handle: %d", joystick_profile.char_x_handle);
                
                // Ajouter maintenant la caractéristique Y
                joystick_profile.char_y_uuid.len = ESP_UUID_LEN_128;
                memcpy(joystick_profile.char_y_uuid.uuid.uuid128, joystick_char_y_uuid, ESP_UUID_LEN_128);
                
                esp_err_t add_char_y_ret = esp_ble_gatts_add_char(
                    joystick_profile.service_handle,
                    &joystick_profile.char_y_uuid,
                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                    &joystick_y_char_val,
                    NULL);

                if (add_char_y_ret) {
                    ESP_LOGE(BLE_JOYSTICK_TAG, "add char Y failed, error code = %x", add_char_y_ret);
                }
            } else if (memcmp(param->add_char.char_uuid.uuid.uuid128, joystick_char_y_uuid, ESP_UUID_LEN_128) == 0) {
                joystick_profile.char_y_handle = param->add_char.attr_handle;
                ESP_LOGI(BLE_JOYSTICK_TAG, "Caractéristique Y ajoutée, handle: %d", joystick_profile.char_y_handle);
            }
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(BLE_JOYSTICK_TAG, "Joystick profile connected, conn_id %d", param->connect.conn_id);
            joystick_profile.conn_id = param->connect.conn_id;
            break;
            
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(BLE_JOYSTICK_TAG, "GATT_READ_EVT, conn_id %" PRIu16 ", trans_id %" PRIu32 ", handle %" PRIu16, 
                param->read.conn_id, param->read.trans_id, param->read.handle);

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            
            rsp.attr_value.handle = param->read.handle;
            
            if (param->read.handle == joystick_profile.char_x_handle) {
                rsp.attr_value.len = sizeof(int32_t);
                memcpy(rsp.attr_value.value, &joystick_x, sizeof(int32_t));
                ESP_LOGI(BLE_JOYSTICK_TAG, "Lecture X: %" PRId32, joystick_x);
            } else if (param->read.handle == joystick_profile.char_y_handle) {
                rsp.attr_value.len = sizeof(int32_t);
                memcpy(rsp.attr_value.value, &joystick_y, sizeof(int32_t));
                ESP_LOGI(BLE_JOYSTICK_TAG, "Lecture Y: %" PRId32, joystick_y);
            }
            
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_JOYSTICK_TAG, "Joystick profile disconnected");
            // La déconnexion est maintenant gérée centralement dans ble.c
            break;
            
        default:
            break;
    }
}