#include <time.h>
#include "pico/stdlib.h"

#include "adc.h"
#include "usb.h"
#include "usb_descriptors.h"


//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void hid_task(mutex_t *key_buf_mut, const KeyBuffers *key_buf, const Keys *keys)
{
  // Keyboard is at interface 0
  if ( tud_hid_n_ready(0) )
  {
    // wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    if ( tud_suspended()) {
      tud_remote_wakeup();
    }
    // use to avoid send multiple consecutive zero report for keyboard
    static bool has_key = false;

    mutex_enter_blocking(key_buf_mut);
    if ( key_buf->left || key_buf->right )
    {
      uint8_t keycode[6] = { 0 };

      if (key_buf->left) {
        keycode[0] = keys->left;
      }
      if (key_buf->right) {
        keycode[1] = keys->right;
      }
      mutex_exit(key_buf_mut);

      tud_hid_n_keyboard_report(0, REPORT_ID_KEYBOARD, 0, keycode);

      has_key = true;
    }else
    {
      mutex_exit(key_buf_mut);
      // send empty key report if previously has key pressed
      if (has_key) tud_hid_n_keyboard_report(0, REPORT_ID_KEYBOARD, 0, NULL);
      has_key = false;
    }
  }
}

// USB Task
void usb_task(mutex_t *key_buf_mut, const KeyBuffers *key_buf, const Keys *keys) {
  hid_task(key_buf_mut, key_buf, keys);
  tud_task();
}