#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "bsp/board.h"
#include "tusb.h"


#include "usb_descriptors.h"


#define KEY_PIN_R         26      // ADC 0 pin
#define KEY_ADC_INPUT_R   0       // ADC Input to select for right key
#define KEY_PIN_L         27      // ADC 1 pin
#define KEY_ADC_INPUT_L   1       // ADC Input to select for left key
#define ADC_BUF_SHIFT     5       // bits needed to shift to average the buffer instead of doing division
#define ADC_BUF_SIZE      0x1 << ADC_BUF_SHIFT      // 2^5
#define THRESHOLD_MULTIPLIER  3 / 4


// keycodes to send for each key
struct {
  uint8_t left;
  uint8_t right;
} keys;

// buffers to debounce adc
struct {
  uint8_t left;
  uint8_t right;
} key_buffers;


// value the buffer
#define MAX_KEY_BUFFER 100


typedef struct {
  uint16_t min;        // lowest adc reading the key has reached while set
  uint16_t max;        // highest adc reading the key has reached while unset
  uint16_t threshold;  // maximum adc reading for the key to be set
  uint16_t reset_gap;  // gap between max and current reading to set
} AdcRange;

struct {
  AdcRange left;
  AdcRange right;
} key_adc_ranges;

struct {
  uint32_t left;
  uint32_t right;
} adc_average;


static void hid_task();
void hid_task_empty();

// have to set adc pin with adc_select_input before running this
void adc_capture(uint16_t *buf, size_t buf_size);
static void average_buffer(const uint16_t *buf, size_t buf_size, uint32_t *average, uint8_t div_shift);
static void process_key(uint8_t *key_buf, AdcRange *adc_range, uint32_t adc_val);


int main() {
  stdio_init_all();
  board_init();

  // usb
  tusb_init();

  // keypad config
  keys.left = HID_KEY_Z;
  keys.right = HID_KEY_X;
  key_adc_ranges.left.max = 1873;
  key_adc_ranges.left.min = 1020;
  key_adc_ranges.left.threshold = (1873 - 1020) * THRESHOLD_MULTIPLIER + 1020;
  key_adc_ranges.left.reset_gap = (1873 - 1020) / 50;
  key_adc_ranges.right.max = 1892;
  key_adc_ranges.right.min = 1069;
  key_adc_ranges.right.threshold = (1892 - 1069) * THRESHOLD_MULTIPLIER + 1069;
  key_adc_ranges.right.reset_gap = (1892 - 1069) / 50;
  key_buffers.left = 0;
  key_buffers.right = 0;

  // adc
  uint16_t adc_buf[ADC_BUF_SIZE];

  adc_init();
  adc_gpio_init(KEY_PIN_L);
  adc_gpio_init(KEY_PIN_R);

  while (1) {
    tud_task();
    hid_task();

    // left key
    adc_select_input(KEY_ADC_INPUT_L);
    adc_capture(adc_buf, ADC_BUF_SIZE);
    average_buffer(adc_buf, ADC_BUF_SIZE, &adc_average.left, ADC_BUF_SHIFT);

    // if (key_buffers.left == 0) {                                                               // currently unset
    //   if (adc_average.left > key_adc_ranges.left.max) {
    //     key_adc_ranges.left.max = adc_average.left;
    //   }
    //   else if (adc_average.left < key_adc_ranges.left.max - key_adc_ranges.left.reset_gap      // moved down enuff from max
    //     && adc_average.left < key_adc_ranges.left.threshold) {                                 // below threshold
    //     key_buffers.left = MAX_KEY_BUFFER;
    //     key_adc_ranges.left.min = adc_average.left;
    //   }
    // } else {                                                                                   // currently set
    //   if (adc_average.left < key_adc_ranges.left.min) {
    //     key_adc_ranges.left.min = adc_average.left;
    //   }
    //   else if (adc_average.left > key_adc_ranges.left.min + key_adc_ranges.left.reset_gap) {
    //     key_buffers.left--;
    //     key_adc_ranges.left.max = adc_average.left;
    //   }
    // }
    process_key(&key_buffers.left, &key_adc_ranges.left, adc_average.left);

    // right key
    adc_select_input(KEY_ADC_INPUT_R);
    adc_capture(adc_buf, ADC_BUF_SIZE);
    average_buffer(adc_buf, ADC_BUF_SIZE, &adc_average.right, ADC_BUF_SHIFT);

    // if (key_buffers.right == 0) {                                                               // currently unset
    //   if (adc_average.right > key_adc_ranges.right.max) {
    //     key_adc_ranges.right.max = adc_average.right;
    //   }
    //   else if (adc_average.right < key_adc_ranges.right.max - key_adc_ranges.right.reset_gap      // moved down enuff from max
    //     && adc_average.right < key_adc_ranges.right.threshold) {                                 // below threshold
    //     key_buffers.right = MAX_KEY_BUFFER;
    //     key_adc_ranges.right.min = adc_average.right;
    //   }
    // } else {                                                                                   // currently set
    //   if (adc_average.right < key_adc_ranges.right.min) {
    //     key_adc_ranges.right.min = adc_average.right;
    //   }
    //   else if (adc_average.right > key_adc_ranges.right.min + key_adc_ranges.right.reset_gap) {
    //     key_buffers.right--;
    //     key_adc_ranges.right.max = adc_average.right;
    //   }
    // }
    process_key(&key_buffers.right, &key_adc_ranges.right, adc_average.right);
  }
}


//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void hid_task()
{
  // Keyboard is at interface 0
  static absolute_time_t last_report_time = 0;
  absolute_time_t current_report_time = get_absolute_time();
  if (absolute_time_diff_us(last_report_time, current_report_time) < 1000) return;   // report rate of ~1000Hz so don't need to constantly process keys
  last_report_time = current_report_time;

  if ( tud_hid_n_ready(0) )
  {
    // wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    if ( tud_suspended()) {
      tud_remote_wakeup();
    }
    // use to avoid send multiple consecutive zero report for keyboard
    static bool has_key = false;

    if ( key_buffers.left || key_buffers.right )
    {
      uint8_t keycode[6] = { 0 };

      if (key_buffers.left) {
        keycode[0] = keys.left;
      }
      if (key_buffers.right) {
        keycode[1] = keys.right;
      }

      tud_hid_n_keyboard_report(0, REPORT_ID_KEYBOARD, 0, keycode);

      has_key = true;
    }else
    {
      // send empty key report if previously has key pressed
      if (has_key) tud_hid_n_keyboard_report(0, REPORT_ID_KEYBOARD, 0, NULL);
      has_key = false;
    }
  }
}


// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;
  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  // bufsize is 1 for some reason when doing bufsize == 1
  // but bufsize passed into memcpy still has it copy all 16 bytes
  // I really don't know why it does this, but I will just ignore it and assume it will
  // always be 16 bytes because that what it seems to do fine
  // and i can make sure that on application side I always send 16 bytes
  // TODO set LED based on CAPLOCK, NUMLOCK etc...

  // Interface 1 is the IO interface -> buffer should be 16 bytes

  uint8_t new_buf[64] = { 0 };
  // first byte of the report will be used to indicate which 'command' will be given to the keypad
  if (itf == 1 && report_type == 0 && report_id == 0 && *buffer == 0x11) {
    memcpy(new_buf, buffer, bufsize);
    memcpy(&new_buf[2], &adc_average.left, sizeof(adc_average.left));
    memcpy(&new_buf[2+sizeof(adc_average.left)], &adc_average.right, sizeof(adc_average.right));
    tud_hid_n_report(1, 0, new_buf, bufsize);
  }
}



//--------------------------------------------------------------------+
// ADC
//--------------------------------------------------------------------+

void __not_in_flash_func(adc_capture)(uint16_t *buf, size_t buf_size) {
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
