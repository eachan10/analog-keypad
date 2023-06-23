#include <time.h>
#include "pico/stdlib.h"

#include "adc.h"
#include "usb.h"
#include "usb_descriptors.h"


//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void hid_task(SemaphoreHandle_t key_buf_mut, KeyBuffers *key_buf, Keys *keys)
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

    // mutex_enter_blocking(key_buf_mut);
    xSemaphoreTake(key_buf_mut, portMAX_DELAY);
    if ( key_buf->left || key_buf->right )
    {
      uint8_t keycode[6] = { 0 };

      if (key_buf->left) {
        keycode[0] = keys->left;
      }
      if (key_buf->right) {
        keycode[1] = keys->right;
      }
      // mutex_exit(key_buf_mut);
      xSemaphoreGive(key_buf_mut);

      tud_hid_n_keyboard_report(0, REPORT_ID_KEYBOARD, 0, keycode);

      has_key = true;
    }else
    {
      // mutex_exit(key_buf_mut);
      xSemaphoreGive(key_buf_mut);
      // send empty key report if previously has key pressed
      if (has_key) tud_hid_n_keyboard_report(0, REPORT_ID_KEYBOARD, 0, NULL);
      has_key = false;
    }
  }
}

// USB Task
void usb_task(SemaphoreHandle_t key_buf_mut, KeyBuffers *key_buf, Keys *keys) {
  hid_task(key_buf_mut, key_buf, keys);
  tud_task();
}