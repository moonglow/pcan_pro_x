#pragma once
#include <stdint.h>

enum e_pcan_led
{
  LED_CH0_RX,
  LED_CH1_RX,
  LED_CH0_TX,
  LED_CH1_TX,
  LED_STAT,

  LED_TOTAL,
};

enum e_pcan_led_mode
{
  LED_MODE_DEVICE = 0, /* we can control led */
  LED_MODE_BLINK_FAST,
  LED_MODE_BLINK_SLOW,
  LED_MODE_ON,
  LED_MODE_OFF,
};

void pcan_led_init( void );
void pcan_led_set_mode( int led, int mode, uint32_t arg );
void pcan_led_poll( void );
