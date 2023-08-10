#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "adc.h"



// offset caused by other key being pressed all the way down to key sensor value
#define KEY_INTERFERENCE_L 270 
#define KEY_INTERFERENCE_R 500 


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
static uint32_t average_buffer(const uint16_t *buf, size_t buf_size, uint8_t div_shift) {
  uint32_t avg = 0;
  for (int i = 0; i < buf_size; i++) {
    avg += buf[i];
  }
  avg >>= div_shift;
  return avg;
}


static void process_key(uint8_t *key_buf, AdcRange *adc_range, uint32_t adc_val) {
  if (*key_buf == 0) {                                                               // currently unset
    if (adc_val > adc_range->max) {
      adc_range->max = adc_val;
    }
    else if (adc_val < adc_range->max - adc_range->reset_gap      // moved down enuff from max
      && adc_val < adc_range->threshold) {                                 // below threshold
      *key_buf = MAX_KEY_BUFFER;
      adc_range->min = adc_val;
    }
  } else {                                                                                   // currently set
    if (adc_val < adc_range->min) {
      adc_range->min = adc_val;
    }
    else if (adc_val > adc_range->min + adc_range->reset_gap) {
      (*key_buf)--;
      adc_range->max = adc_val;
    }
  }
}

// ADC Task
void adc_task(SemaphoreHandle_t key_buf_mut, KeyBuffers *key_buf, AdcAverage *adc_average, AdcRanges *adc_ranges, AdcConfig *adc_config) {
    uint16_t adc_buf1[ADC_BUF_SIZE];
    uint16_t adc_buf2[ADC_BUF_SIZE];
    // left key read + average
    adc_select_input(KEY_ADC_INPUT_L);
    adc_capture(adc_buf1, ADC_BUF_SIZE);

    // right key read + average
    adc_select_input(KEY_ADC_INPUT_R);
    adc_capture(adc_buf2, ADC_BUF_SIZE);

    // process adc values into key buffers
    xSemaphoreTake(key_buf_mut, portMAX_DELAY);
    adc_average->left = average_buffer(adc_buf1, ADC_BUF_SIZE, ADC_BUF_SHIFT);
    adc_average->right = average_buffer(adc_buf2, ADC_BUF_SIZE, ADC_BUF_SHIFT);

    int32_t l_off, r_off, l_range, r_range, l_gap, r_gap;
    l_range = adc_config->left_max - adc_config->left_min;
    r_range = adc_config->right_max - adc_config->right_min;

    // distance from max height to current
    l_gap = (int32_t)adc_config->left_max - (int32_t)adc_average->left;
    if (l_gap < 0) l_gap = 0;
    else if (l_gap > l_range) l_gap = l_range;
    r_gap = (int32_t)adc_config->right_max - (int32_t)adc_average->right;
    if (r_gap < 0) r_gap = 0;
    else if (r_gap > r_range) r_gap = r_range;
    l_off = KEY_INTERFERENCE_L * r_gap / r_range;         // increase offset as other key goes down
    l_off = l_off * (l_range - l_gap / 2) / l_range;    // decrease offset as current key goes down
    r_off = KEY_INTERFERENCE_R * l_gap / l_range / 2;
    r_off = r_off * (r_range - r_gap / 2) / r_range;
    if (l_off > adc_average->left) l_off = adc_average->left;
    if (r_off > adc_average->right) r_off = adc_average->right;
    adc_average->left -= l_off;
    adc_average->right -= r_off;

    process_key(&(key_buf->left), &(adc_ranges->left), adc_average->left);
    process_key(&(key_buf->right), &(adc_ranges->right), adc_average->right);
    xSemaphoreGive(key_buf_mut);
}