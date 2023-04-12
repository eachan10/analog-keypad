#include <stdint.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"

#include "bsp/board.h"
#include "tusb.h"

#include "adc.h"


#ifndef USB_STUFF
#define USB_STUFF

// keycodes to send for each key
typedef struct {
  uint8_t left;
  uint8_t right;
} Keys;

void usb_task(mutex_t *key_buf_mut, KeyBuffers *key_buf, Keys *keys);

#endif