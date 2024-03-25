#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "adc.h"

#include <math.h>



// offset caused by other key being pressed all the way down to key sensor value
#define KEY_INTERFERENCE_L 300 
#define KEY_INTERFERENCE_R 300 


//--------------------------------------------------------------------+
// ADC
//--------------------------------------------------------------------+

static void __not_in_flash_func(adc_capture)(uint16_t *buf, size_t buf_size) {
  adc_fifo_setup(true, false, 0, false, false);
  adc_run(true);
  for (int i = 0; i < buf_size; i++) {
    buf[i] = adc_fifo_get_blocking();
  }
  adc_run(false);
  adc_fifo_drain();
}


// Helper func to average my adc capture buffers
static int32_t average_buffer(const uint16_t *buf, size_t buf_size, uint8_t div_shift) {
  uint32_t avg = 0;
  for (int i = 0; i < buf_size; i++) {
    avg += buf[i];
  }
  avg >>= div_shift;
  return (int32_t)avg;
}


static void process_key(uint8_t *key_buf, AdcRange *adc_range, const int32_t adc_val, const AdcConfig *adc_config) {
  // map inverse square distance to linear function from 0 to 100
  double percentage = 100 - 100 * sqrt((adc_config->max - (double)adc_val) / (adc_config->max - adc_config->min));

  if (*key_buf == 0) {                                            // currently unset
    if (percentage > adc_range->max) {
      adc_range->max = percentage;
    }
    else if (percentage < adc_range->max - adc_config->reset      // moved down enuff from max
      && percentage < adc_config->threshold) {                    // below threshold
      *key_buf = MAX_KEY_BUFFER;
      adc_range->min = percentage;
    }
  } else {                                                        // currently set
    if (percentage < adc_range->min) {
      adc_range->min = percentage;
    }
    else if (percentage > adc_range->min + adc_config->reset) {
      (*key_buf)--;
      adc_range->max = percentage;
    }
  }
}

// ADC Task
void adc_task(SemaphoreHandle_t key_buf_mut, SemaphoreHandle_t config_mutex, KeyBuffers *key_buf, AdcAverage *adc_average, AdcRanges *adc_ranges, const AdcConfig *left_config, const AdcConfig *right_config) {
    uint16_t adc_buf1[ADC_BUF_SIZE];
    uint16_t adc_buf2[ADC_BUF_SIZE];
    // left key read + average
    adc_select_input(KEY_ADC_INPUT_L);
    adc_capture(adc_buf1, ADC_BUF_SIZE);

    // right key read + average
    adc_select_input(KEY_ADC_INPUT_R);
    adc_capture(adc_buf2, ADC_BUF_SIZE);

    xSemaphoreTake(config_mutex, portMAX_DELAY);
    // process adc values into key buffers
    adc_average->left = average_buffer(adc_buf1, ADC_BUF_SIZE, ADC_BUF_SHIFT);
    adc_average->right = average_buffer(adc_buf2, ADC_BUF_SIZE, ADC_BUF_SHIFT);

    int32_t l_off, r_off, l_range, r_range, l_gap, r_gap;
    l_range = left_config->max - left_config->min;
    r_range = right_config->max - right_config->min;

    // distance from max height to current
    l_gap = left_config->max - adc_average->left;

    // constrain gap to maximum
    if (l_gap < 0) {
      l_gap = 0;
    }
    r_gap = right_config->max - adc_average->right;
    if (r_gap < 0) {
      r_gap = 0;
    }
    l_off = KEY_INTERFERENCE_L * r_gap / r_range;     // increase offset as other key goes down
    r_off = KEY_INTERFERENCE_R * l_gap / l_range;
    // if (l_off > adc_average->left) l_off = adc_average->left;
    // if (r_off > adc_average->right) r_off = adc_average->right;
    
    // constrain to set adc min and max values
    adc_average->left -= l_off;
    if (adc_average->left < left_config->min) {
      adc_average->left = left_config->min;
    } else if (adc_average->left > left_config->max) {
      adc_average->left = left_config->max;
    }
    adc_average->right -= r_off;
    if (adc_average->right < right_config->min) {
      adc_average->right = right_config->min;
    } else if (adc_average->right > right_config->max) {
      adc_average->right = right_config->max;
    }

    xSemaphoreTake(key_buf_mut, portMAX_DELAY);
    process_key(&(key_buf->left), &(adc_ranges->left), adc_average->left, left_config);
    process_key(&(key_buf->right), &(adc_ranges->right), adc_average->right, right_config);
    xSemaphoreGive(key_buf_mut);
    xSemaphoreGive(config_mutex);
}