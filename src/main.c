#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "bsp/board.h"
#include "tusb.h"


#include "usb_descriptors.h"
#include "adc.h"
#include "usb.h"


Keys keys;
KeyBuffers key_buffers;

auto_init_mutex(key_buffers_mutex);


AdcAverage adc_average;
AdcRanges adc_ranges;


void hid_task_empty();

static bool is_valid_hid_key_code(uint8_t keycode);

void core1_entry();


int main() {
  stdio_init_all();
  board_init();

  // usb
  tusb_init();

  // keypad config/setup
  adc_ranges.left.max = 3054;
  adc_ranges.left.min = 1283;
  adc_ranges.right.max = 3124;
  adc_ranges.right.min = 1392;
  key_buffers.left = 0;
  key_buffers.right = 0;

  // configurable options
  keys.left = HID_KEY_PERIOD;
  keys.right = HID_KEY_SLASH;
  adc_ranges.left.threshold = (adc_ranges.left.max - adc_ranges.left.min) * THRESHOLD_MULTIPLIER + adc_ranges.left.min;
  adc_ranges.left.reset_gap = (adc_ranges.left.max - adc_ranges.left.min) / 50;
  adc_ranges.right.threshold = (adc_ranges.right.max - adc_ranges.right.min) * THRESHOLD_MULTIPLIER + adc_ranges.right.min;
  adc_ranges.right.reset_gap = (adc_ranges.right.max - adc_ranges.right.min) / 50;

  adc_init();
  adc_gpio_init(KEY_PIN_L);
  adc_gpio_init(KEY_PIN_R);
  
  multicore_launch_core1(core1_entry);

  while (1) {
    usb_task(&key_buffers_mutex, &key_buffers, &keys);
  }
}


//--------------------------------------------------------------------+
// Tasks
//--------------------------------------------------------------------+

void core1_entry() {
  while (1) {
    adc_task(&key_buffers_mutex, &key_buffers, &adc_average, &adc_ranges);
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
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
  // Interface 1 is the IO interface -> buffer should be 16 bytes

  uint8_t new_buf[64] = { 0 };
  uint8_t k1, k2;
  // first byte of the report will be used to indicate which 'command' will be given to the keypad
  if (itf == 1 && report_type == 0 && report_id == 0) {
    switch (*buffer) {
    case 0x11:
      // return values read by adc
      memcpy(new_buf, buffer, bufsize);
      memcpy(&new_buf[2], &adc_average.left, sizeof(adc_average.left));
      memcpy(&new_buf[2+sizeof(adc_average.left)], &adc_average.right, sizeof(adc_average.right));
      break;
    case 0x12:
      // set keys left and right should be in bytes 1 and 2
      k1 = buffer[1];
      k2 = buffer[2];
      if (is_valid_hid_key_code(k1) && is_valid_hid_key_code(k2)) {
        keys.left = k1;
        keys.right = k2;
        memcpy(new_buf, buffer, bufsize);  // echo on success
      }
      // returned report is all 0 if invalid keycodes
      break;
    }
    tud_hid_n_report(1, 0, new_buf, bufsize);
  }
}


static bool is_valid_hid_key_code(uint8_t keycode) {
  return (0x04 <= keycode && keycode <= 0xA4) || (0xE0 <= keycode && keycode <= 0xE7);
}