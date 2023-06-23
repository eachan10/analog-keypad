#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "adc.h"


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
static void average_buffer(const uint16_t *buf, size_t buf_size, uint32_t *average, uint8_t div_shift) {
  *average = 0;
  for (int i = 0; i < buf_size; i++) {
    *average += buf[i];
  }
  *average >>= div_shift;
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
void adc_task(SemaphoreHandle_t key_buf_mut, KeyBuffers *key_buf, AdcAverage *adc_average, AdcRanges *adc_ranges) {
    uint16_t adc_buf[ADC_BUF_SIZE];
    // left key read + average
    adc_select_input(KEY_ADC_INPUT_L);
    adc_capture(adc_buf, ADC_BUF_SIZE);
    average_buffer(adc_buf, ADC_BUF_SIZE, &adc_average->left, ADC_BUF_SHIFT);

    // right key read + average
    adc_select_input(KEY_ADC_INPUT_R);
    adc_capture(adc_buf, ADC_BUF_SIZE);
    average_buffer(adc_buf, ADC_BUF_SIZE, &adc_average->right, ADC_BUF_SHIFT);

    // process adc values into key buffers
    // mutex_enter_blocking(key_buf_mut);
    xSemaphoreTake(key_buf_mut, portMAX_DELAY);
    process_key(&(key_buf->left), &(adc_ranges->left), adc_average->left);
    process_key(&(key_buf->right), &(adc_ranges->right), adc_average->right);
    // mutex_exit(key_buf_mut);
    xSemaphoreGive(key_buf_mut);
}