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

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


Keys keys;
KeyBuffers key_buffers;

// auto_init_mutex(key_buffers_mutex);

SemaphoreHandle_t key_buffers_mutex;


AdcAverage adc_average;
AdcRanges adc_ranges;
struct {
  uint8_t threshold_percentage;
  uint8_t reset_percentage;
  uint16_t left_max;
  uint16_t left_min;
  uint16_t right_max;
  uint16_t right_min;
} adc_config;


void hid_task_empty();

static bool is_valid_hid_key_code(uint8_t keycode);
static void recalculate_threshold();
static void recalculate_reset_distance();

void adc_task_entry(void *pvParameters);
void usb_task_entry(void *pvParameters);


int main() {
  stdio_init_all();
  board_init();

  // usb
  tusb_init();

  // keypad config/setup
  adc_config.left_max = 3054;
  adc_config.left_min = 1283;
  adc_config.right_max = 3124;
  adc_config.right_min = 1392;
  key_buffers.left = 0;
  key_buffers.right = 0;

  // configurable options
  adc_config.threshold_percentage = 65;
  adc_config.reset_percentage = 4;
  keys.left = HID_KEY_PERIOD;
  keys.right = HID_KEY_SLASH;
  adc_ranges.left.threshold = (adc_config.left_max - adc_config.left_min) * adc_config.threshold_percentage / 100 + adc_config.left_min;
  adc_ranges.left.reset_gap = (adc_config.left_max - adc_config.left_min) * adc_config.reset_percentage / 100;
  adc_ranges.right.threshold = (adc_config.right_max - adc_config.right_min) * adc_config.threshold_percentage / 100 + adc_config.right_min;
  adc_ranges.right.reset_gap = (adc_config.right_max - adc_config.right_min) * adc_config.reset_percentage / 100;

  adc_init();
  adc_gpio_init(KEY_PIN_L);
  adc_gpio_init(KEY_PIN_R);

  key_buffers_mutex = xSemaphoreCreateMutex();
  
  // multicore_launch_core1(core1_entry);

  xTaskCreate(
    adc_task_entry,
    "ADC_TASK",
    256,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    usb_task_entry,
    "USB_TASK",
    256,
    NULL,
    2,
    NULL
  );

  vTaskStartScheduler();

  while (1) {
    // usb_task(&key_buffers_mutex, &key_buffers, &keys);
  }
}


//--------------------------------------------------------------------+
// Tasks
//--------------------------------------------------------------------+

void adc_task_entry(void *pvParameters) {
  TickType_t LastWakeTime;
  const TickType_t xFrequency = 1;
  LastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&LastWakeTime, xFrequency);
    adc_task(key_buffers_mutex, &key_buffers, &adc_average, &adc_ranges);
  }
}

void usb_task_entry(void *pvParameters) {
  TickType_t LastWakeTime;
  const TickType_t xFrequency = 10;
  LastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&LastWakeTime, xFrequency);
    usb_task(key_buffers_mutex, &key_buffers, &keys);
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
        // mutex_enter_blocking(&key_buffers_mutex);
        xSemaphoreTake(key_buffers_mutex, portMAX_DELAY);
        adc_config.threshold_percentage = buffer[1];
        recalculate_threshold();
        // mutex_exit(&key_buffers_mutex);
        xSemaphoreGive(key_buffers_mutex);
        memcpy(new_buf, buffer, bufsize);  // echo on success
      }
      // returned report is all 0 if invalid percentage value
      break;
    case 0x14:
      // set reset distance
      if (buffer[1] >= 2u && buffer[1] <= 80u) {
        // mutex_enter_blocking(&key_buffers_mutex);
        xSemaphoreTake(key_buffers_mutex, portMAX_DELAY);
        adc_config.reset_percentage = buffer[1];
        recalculate_reset_distance();
        // mutex_exit(&key_buffers_mutex);
        xSemaphoreGive(key_buffers_mutex);
        memcpy(new_buf, buffer, bufsize);
      }
      break;
    case 0x15:
      // callibrate max and recalculate all values
      // mutex_enter_blocking(&key_buffers_mutex);
      xSemaphoreTake(key_buffers_mutex, portMAX_DELAY);
      memcpy(&adc_ranges.left.max, &buffer[1], sizeof(uint16_t));
      memcpy(&adc_ranges.right.max, &buffer[3], sizeof(uint16_t));
      adc_config.left_max = adc_ranges.left.max;
      adc_config.right_max = adc_ranges.right.max;
      recalculate_reset_distance();
      recalculate_threshold();
      // mutex_exit(&key_buffers_mutex);
      xSemaphoreGive(key_buffers_mutex);
      memcpy(new_buf, buffer, bufsize);  // echo on success
      break;
    case 0x16:
      // callibrate min and recalculate all values;
      // mutex_enter_blocking(&key_buffers_mutex);
      xSemaphoreTake(key_buffers_mutex, portMAX_DELAY);
      memcpy(&adc_ranges.left.min, &buffer[1], sizeof(uint16_t));
      memcpy(&adc_ranges.right.min, &buffer[3], sizeof(uint16_t));
      adc_config.left_min = adc_ranges.left.min;
      adc_config.right_min = adc_ranges.right.min;
      recalculate_reset_distance();
      recalculate_threshold();
      // mutex_exit(&key_buffers_mutex);
      xSemaphoreGive(key_buffers_mutex);
      memcpy(new_buf, buffer, bufsize);  // echo on success
      break;
    case 0x01:
      // get current config settings
      // mutex_enter_blocking(&key_buffers_mutex);
      xSemaphoreTake(key_buffers_mutex, portMAX_DELAY);
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
        memcpy(&new_buf[i], &adc_config.left_max, sizeof(adc_config.left_max));
        i += sizeof(adc_config.left_max);
        memcpy(&new_buf[i], &adc_config.left_min, sizeof(adc_config.left_min));
        i += sizeof(adc_config.left_min);
        memcpy(&new_buf[i], &adc_config.right_max, sizeof(adc_config.right_max));
        i += sizeof(adc_config.right_max);
        memcpy(&new_buf[i], &adc_config.right_min, sizeof(adc_config.right_min));
      }
      // mutex_exit(&key_buffers_mutex);
      xSemaphoreGive(key_buffers_mutex);
      break;
    }
    tud_hid_n_report(1, 0, new_buf, bufsize);
  }
}


static bool is_valid_hid_key_code(uint8_t keycode) {
  return (0x04 <= keycode && keycode <= 0xA4) || (0xE0 <= keycode && keycode <= 0xE7);
}

static void recalculate_threshold() {
  adc_ranges.left.threshold = (adc_config.left_max - adc_config.left_min) * adc_config.threshold_percentage / 100 + adc_config.left_min;
  adc_ranges.right.threshold = (adc_config.right_max - adc_config.right_min) * adc_config.threshold_percentage / 100 + adc_config.right_min;
}

static void recalculate_reset_distance() {
  adc_ranges.left.reset_gap = (adc_config.left_max - adc_config.left_min) * adc_config.reset_percentage / 100;
  adc_ranges.right.reset_gap = (adc_config.right_max - adc_config.right_min) * adc_config.reset_percentage / 100;
}
