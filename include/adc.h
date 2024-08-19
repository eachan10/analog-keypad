#ifndef __ADC_STUFF__
#define __ADC_STUFF__

#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/sync.h"

#define KEY_PIN_L         26      // ADC 0 pin
#define KEY_ADC_INPUT_L   0       // ADC Input to select for right key
#define KEY_PIN_R         27      // ADC 1 pin
#define KEY_ADC_INPUT_R   1       // ADC Input to select for left key
#define ADC_BUF_SHIFT     3       // bits needed to shift to average the buffer instead of doing division
#define ADC_BUF_SIZE      0x1 << ADC_BUF_SHIFT      // 2^3

// configurable options
#define THRESHOLD_MULTIPLIER  3 / 4

// value the buffer is set to when key is set
#define MAX_KEY_BUFFER 20

typedef struct {
  int32_t threshold;
  int32_t reset;
  int32_t max;
  int32_t min;
} AdcConfig;

// buffers to debounce adc
// this is shared between the hid_task and process_keys
typedef struct {
  uint8_t left;
  uint8_t right;
} KeyBuffers;

typedef struct {
  int32_t min;        // lowest adc reading the key has reached while set
  int32_t max;        // highest adc reading the key has reached while unset
} AdcRange;

typedef struct {
  AdcRange left;
  AdcRange right;
} AdcRanges;

typedef struct {
  int32_t left;
  int32_t right;
} AdcAverage;

void adc_task(mutex_t *key_buf_mut, mutex_t *config_mutex, KeyBuffers *key_buf, AdcAverage *adc_average, AdcRanges *adc_ranges, AdcConfig *left_config, AdcConfig *right_config);
// void adc_task(mutex_t *key_buf_mut, KeyBuffers *key_buf, AdcAverage *adc_average, AdcRanges *adc_ranges, AdcConfig *left_config, AdcConfig *right_config);

#endif