#include "joystick/joystick.h"

// #if CONFIG_IDF_TARGET_ESP32
// static adc_channel_t channel[2] = {ADC_CHANNEL_6, ADC_CHANNEL_7};
// #else
// static adc_channel_t channel[2] = {ADC_CHANNEL_2, ADC_CHANNEL_3};
// #endif

static adc_channel_t channel[2] = {ADC_CHANNEL_0, ADC_CHANNEL_3}; // X et Y

// static const char *TAG = "JOYSTICK";
static TaskHandle_t s_task_handle = NULL;
static adc_continuous_handle_t s_adc_handle = NULL;
static bool adc_initialized = false;

int x_raw = -1;
int y_raw = -1;
// int z_raw = 0;

// gpio_config_t io_axe_z_conf = {
//     .pin_bit_mask = (1ULL << JOYSTICK_AXE_Z),
//     .mode = GPIO_MODE_INPUT,
//     .pull_up_en = GPIO_PULLUP_ENABLE,
//     .pull_down_en = GPIO_PULLDOWN_DISABLE,
//     .intr_type = GPIO_INTR_NEGEDGE
// };

// static TaskHandle_t s_joystick_btn_task_handle = NULL;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    // Vérifier que la tâche existe avant de la notifier
    if (s_task_handle != NULL) {
        vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    }
    return mustYield == pdTRUE;
}

// static void joystick_btn_task(void *arg)
// {
//     int last_state = gpio_get_level(JOYSTICK_AXE_Z);
    
//     while (1) {
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
//         int current_state = gpio_get_level(JOYSTICK_AXE_Z);
        
//         // Ne traite que si l'état a vraiment changé
//         if (current_state != last_state) {
//             z_raw ^= 1;
//             ESP_LOGI(TAG, "AXE Z: %d", z_raw);
//             // event_bus_publish(&event);
//             last_state = current_state;
//         }
//     }
// }

// static void IRAM_ATTR joystick_isr_handler(void *arg) 
// {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     vTaskNotifyGiveFromISR(s_joystick_btn_task_handle, &xHigherPriorityTaskWoken);
//     if (xHigherPriorityTaskWoken) {
//         portYIELD_FROM_ISR();
//     }
// }

static void joystick_adc_init(void)
{
    // Configuration plus conservative de l'ADC
    adc_continuous_handle_cfg_t cfg = {
        .max_store_buf_size = 512,  // Réduit de 1024 à 512
        .conv_frame_size = JOYSTICK_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&cfg, &s_adc_handle));

    adc_digi_pattern_config_t patterns[2] = {0};
    for (int i = 0; i < 2; i++) {
        patterns[i].atten = JOYSTICK_ADC_ATTEN;
        patterns[i].channel = channel[i];
        patterns[i].unit = JOYSTICK_ADC_UNIT;
        patterns[i].bit_width = JOYSTICK_ADC_BIT_WIDTH;
    }

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20000,  // Réduit de 20kHz à 10kHz
        .conv_mode = JOYSTICK_ADC_CONV_MODE,
        .format = JOYSTICK_ADC_OUTPUT_TYPE,
        .pattern_num = 2,
        .adc_pattern = patterns,
    };

    ESP_ERROR_CHECK(adc_continuous_config(s_adc_handle, &dig_cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(s_adc_handle, &cbs, NULL));
    
    // Attendre un peu avant de démarrer l'ADC
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(adc_continuous_start(s_adc_handle));
    
    adc_initialized = true;
}

static void joystick_publish_event_direction(char axe, int position) 
{
    // Vérifier que l'ADC est initialisé avant de publier
    if (!adc_initialized) {
        return;
    }
    
    event_t event = {
        .type = EVENT_BLE_JOYSTICK_DIRECTION,
        .data = {
            .ble_joystick_direction = {
                .axe = axe,
                .position = position
            }
        }
    };

    event_bus_publish(&event);
}

static void joystick_task(void *arg)
{
    s_task_handle = xTaskGetCurrentTaskHandle();
    uint8_t result[JOYSTICK_READ_LEN];
    uint32_t ret_num;

    // Attendre que l'ADC soit complètement initialisé
    while (!adc_initialized) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (adc_continuous_read(s_adc_handle, result, JOYSTICK_READ_LEN, &ret_num, 0) == ESP_OK) {
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                int chan = JOYSTICK_ADC_GET_CHANNEL(p);
                int val = JOYSTICK_ADC_GET_DATA(p);
                
                // ESP_LOGI(TAG, "Channel %d = %d", chan, val);

                if (chan == ADC_CHANNEL_0) {
                    x_raw = val;
                    joystick_publish_event_direction('x', x_raw);
                }

                if (chan == ADC_CHANNEL_3) {
                    y_raw = val;
                    joystick_publish_event_direction('y', y_raw);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));  // Augmenté de 1ms à 10ms
        }
    }
}

void joystick_init(void)
{
    // Créer la tâche d'abord
    xTaskCreatePinnedToCore(
        joystick_task,
        "joystick_task",
        4096, // Réduit de 8192 à 4096
        NULL,
        3,    // Réduit la priorité de 5 à 3
        NULL,
        tskNO_AFFINITY
    );

    // Attendre que la tâche soit créée
    vTaskDelay(pdMS_TO_TICKS(50));

    // Initialiser l'ADC après la création de la tâche
    joystick_adc_init();

    // xTaskCreatePinnedToCore(
    //     joystick_btn_task,
    //     "joystick_btn_task",
    //     8192,
    //     NULL,
    //     5,
    //     &s_joystick_btn_task_handle,
    //     tskNO_AFFINITY
    // );

    // gpio_config(&io_axe_z_conf);
    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(JOYSTICK_AXE_Z, joystick_isr_handler, (void*) JOYSTICK_AXE_Z);
}