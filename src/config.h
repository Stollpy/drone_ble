#ifndef CONFIG_H
#define CONFIG_H

#include <inttypes.h>

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#define DEVICE_NAME "stollpy_drone"

// JOYSTICK
#define JOYSTICK_ADC_MIN     0
#define JOYSTICK_ADC_MAX     4095
#define JOYSTICK_ADC_CENTER  ((JOYSTICK_ADC_MAX - JOYSTICK_ADC_MIN) / 2)
#define JOYSTICK_DEADZONE    200
#endif