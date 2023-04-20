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
struct {
  uint8_t threshold_percentage;
  uint8_t reset_percentage;
} adc_config;


void hid_task_empty();

static bool is_valid_hid_key_code(uint8_t keycode);
static void recalculate_threshold();
static void recalculate_reset_distance();

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
  adc_config.threshold_percentage = 75;
  adc_config.reset_percentage = 2;
  keys.left = HID_KEY_PERIOD;
  keys.right = HID_KEY_SLASH;
  adc_ranges.left.threshold = (adc_ranges.left.max - adc_ranges.left.min) * adc_config.threshold_percentage / 100 + adc_ranges.left.min;
  adc_ranges.left.reset_gap = (adc_ranges.left.max - adc_ranges.left.min) * adc_config.reset_percentage / 100;
  adc_ranges.right.threshold = (adc_ranges.right.max - adc_ranges.right.min) * adc_config.threshold_percentage / 100 + adc_ranges.right.min;
  adc_ranges.right.reset_gap = (adc_ranges.right.max - adc_ranges.right.min) * adc_config.reset_percentage / 100;

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
      if (is_valid_hid_key_code(buffer[1]) && is_valid_hid_key_code(buffer[2])) {
        keys.left = buffer[1];
        keys.right = buffer[2];
        memcpy(new_buf, buffer, bufsize);  // echo on success
      }
      // returned report is all 0 if invalid keycodes
      break;
    case 0x13:
      // set threshold for actuation
      if (buffer[1] <= 95u) {
        mutex_enter_blocking(&key_buffers_mutex);
        adc_config.threshold_percentage = buffer[1];
        recalculate_threshold();
        mutex_exit(&key_buffers_mutex);
        memcpy(new_buf, buffer, bufsize);  // echo on success
      }
      // returned report is all 0 if invalid percentage value
      break;
    case 0x14:
      // set reset distance
      if (buffer[1] >= 2u && buffer[1] <= 80u) {
        mutex_enter_blocking(&key_buffers_mutex);
        adc_config.reset_percentage = buffer[1];
        recalculate_reset_distance();
        mutex_exit(&key_buffers_mutex);
        memcpy(new_buf, buffer, bufsize);
      }
      break;
    case 0x15:
      // callibrate max and recalculate all values
      mutex_enter_blocking(&key_buffers_mutex);
      memcpy(&adc_ranges.left.max, &buffer[1], sizeof(uint16_t));
      memcpy(&adc_ranges.right.max, &buffer[3], sizeof(uint16_t));
      recalculate_reset_distance();
      recalculate_threshold();
      mutex_exit(&key_buffers_mutex);
      memcpy(new_buf, buffer, bufsize);  // echo on success
      break;
    case 0x16:
      // callibrate min and recalculate all values;
      mutex_enter_blocking(&key_buffers_mutex);
      memcpy(&adc_ranges.left.min, &buffer[1], sizeof(uint16_t));
      memcpy(&adc_ranges.right.min, &buffer[3], sizeof(uint16_t));
      recalculate_reset_distance();
      recalculate_threshold();
      mutex_exit(&key_buffers_mutex);
      memcpy(new_buf, buffer, bufsize);  // echo on success
      break;
    case 0x01:
      // get current config settings
      mutex_enter_blocking(&key_buffers_mutex);
      // byte 1 -> left keycode
      // byte 2 -> right keycode
      // byte 3 -> threshold percentage
      // byte 4 -> reset percentage
      // byte 5-6 -> left max
      // byte 7-8 -> left min
      // byte 9-10 -> right max
      // byte 11-12 -> right min
      {
        size_t i = 1;
        memcpy(&new_buf[i], &keys.left, sizeof(keys.left));
        i += sizeof(keys.left);
        memcpy(&new_buf[i], &keys.right, sizeof(keys.right));
        i += sizeof(keys.right);
        memcpy(&new_buf[i], &adc_config.threshold_percentage, sizeof(adc_config.threshold_percentage));
        i += sizeof(adc_config.threshold_percentage);
        memcpy(&new_buf[i], &adc_config.reset_percentage, sizeof(adc_config.reset_percentage));
        i += sizeof(adc_config.reset_percentage);
        memcpy(&new_buf[i], &adc_ranges.left.max, sizeof(adc_ranges.left.max));
        i += sizeof(adc_ranges.left.max);
        memcpy(&new_buf[i], &adc_ranges.left.min, sizeof(adc_ranges.left.min));
        i += sizeof(adc_ranges.left.min);
        memcpy(&new_buf[i], &adc_ranges.right.max, sizeof(adc_ranges.right.max));
        i += sizeof(adc_ranges.right.max);
        memcpy(&new_buf[i], &adc_ranges.right.min, sizeof(adc_ranges.right.min));
      }
      mutex_exit(&key_buffers_mutex);
    }
    tud_hid_n_report(1, 0, new_buf, bufsize);
  }
}


static bool is_valid_hid_key_code(uint8_t keycode) {
  return (0x04 <= keycode && keycode <= 0xA4) || (0xE0 <= keycode && keycode <= 0xE7);
}

static void recalculate_threshold() {
  adc_ranges.left.threshold = (adc_ranges.left.max - adc_ranges.left.min) * adc_config.threshold_percentage / 100 + adc_ranges.left.min;
  adc_ranges.right.threshold = (adc_ranges.right.max - adc_ranges.right.min) * adc_config.threshold_percentage / 100 + adc_ranges.right.min;
}

static void recalculate_reset_distance() {
  adc_ranges.left.reset_gap = (adc_ranges.left.max - adc_ranges.left.min) * adc_config.reset_percentage / 100;
  adc_ranges.right.reset_gap = (adc_ranges.right.max - adc_ranges.right.min) * adc_config.reset_percentage / 100;
}