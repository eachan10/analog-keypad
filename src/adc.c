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
void adc_task(SemaphoreHandle_t key_buf_mut, KeyBuffers *key_buf, AdcAverage *adc_average, AdcRanges *adc_ranges) {
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
    process_key(&(key_buf->left), &(adc_ranges->left), adc_average->left);
    process_key(&(key_buf->right), &(adc_ranges->right), adc_average->right);
    xSemaphoreGive(key_buf_mut);
}