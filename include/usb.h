#include <stdint.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"

#include "bsp/board.h"
#include "tusb.h"

#include "adc.h"

#include "FreeRTOS.h"
#include "semphr.h"


#ifndef USB_STUFF
#define USB_STUFF

// keycodes to send for each key
typedef struct {
  uint8_t left;
  uint8_t right;
} Keys;

void usb_task(SemaphoreHandle_t key_buf_mut, const KeyBuffers *key_buf, const Keys *keys);

#endif