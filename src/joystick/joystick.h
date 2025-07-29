// JOYSTICK
#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include "event/event.h"

#define JOYSTICK_ADC_UNIT                    ADC_UNIT_1
#define _JOYSTICK_ADC_UNIT_STR(unit)         #unit
#define JOYSTICK_ADC_UNIT_STR(unit)          _JOYSTICK_ADC_UNIT_STR(unit)
#define JOYSTICK_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define JOYSTICK_ADC_ATTEN                   ADC_ATTEN_DB_12
#define JOYSTICK_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define JOYSTICK_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define JOYSTICK_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define JOYSTICK_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define JOYSTICK_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define JOYSTICK_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define JOYSTICK_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define JOYSTICK_READ_LEN                    256

#define JOYSTICK_AXE_Z GPIO_NUM_5

void joystick_init(void);
#endif