#include <assert.h>
#include "io_macro.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_led.h"

static struct
{
  uint32_t arg;
  uint32_t delay;
  uint32_t timestamp;
  uint16_t mode;
  uint8_t  state;
}
led_mode_array[LED_TOTAL] = { 0 };


#define IO_LED_STAT C, 10, MODE_OUTPUT_PP, NOPULL, SPEED_FREQ_MEDIUM, NOAF
#define IO_LED_TX0 A, 2, MODE_OUTPUT_PP, NOPULL, SPEED_FREQ_MEDIUM, NOAF
#define IO_LED_RX0 A, 3, MODE_OUTPUT_PP, NOPULL, SPEED_FREQ_MEDIUM, NOAF
#define IO_LED_TX1 C, 6, MODE_OUTPUT_PP, NOPULL, SPEED_FREQ_MEDIUM, NOAF
#define IO_LED_RX1 C, 7, MODE_OUTPUT_PP, NOPULL, SPEED_FREQ_MEDIUM, NOAF

void pcan_led_init( void )
{
  PORT_ENABLE_CLOCK( PIN_PORT( IO_LED_TX0 ), PIN_PORT( IO_LED_RX0 ) );
  PORT_ENABLE_CLOCK( PIN_PORT( IO_LED_TX1 ), PIN_PORT( IO_LED_RX1 ) );
  PORT_ENABLE_CLOCK( PIN_PORT( IO_LED_STAT ), PIN_PORT( IO_LED_STAT ) );
  
  PIN_INIT( IO_LED_STAT );
  PIN_INIT( IO_LED_TX0 );
  PIN_INIT( IO_LED_RX0 );
  PIN_INIT( IO_LED_TX1 );
  PIN_INIT( IO_LED_RX1 );
}

void pcan_led_set_mode( int led, int mode, uint32_t arg )
{
  assert( led < LED_TOTAL );

  if( led_mode_array[led].mode == mode )
    return;

  led_mode_array[led].mode = mode;
  led_mode_array[led].timestamp = pcan_timestamp_millis();
  led_mode_array[led].delay = 0;

  /* set guard time */
  if( mode == LED_MODE_BLINK_FAST || mode == LED_MODE_BLINK_SLOW )
  {
    led_mode_array[led].delay = ( mode == LED_MODE_BLINK_FAST ) ? 50: 200;
    if( arg != 0xFFFFFFFF )
    {
      /* update to absolute */
      arg += led_mode_array[led].timestamp;
    }
  }

  led_mode_array[led].arg  = arg;
}

static void _led_update_state( int led, uint8_t state )
{
  switch( led )
  {
    case LED_STAT:
      if( state )
        PIN_HI( IO_LED_STAT );
      else
        PIN_LOW( IO_LED_STAT );
    break;
    case LED_CH0_TX:
      if( state )
        PIN_HI( IO_LED_TX0 );
      else
        PIN_LOW( IO_LED_TX0 );
    break;
    case LED_CH0_RX:
      if( state )
        PIN_HI( IO_LED_RX0 );
      else
        PIN_LOW( IO_LED_RX0 );
    break;
    case LED_CH1_TX:
      if( state )
        PIN_HI( IO_LED_TX1 );
      else
        PIN_LOW( IO_LED_TX1 );
    break;
    case LED_CH1_RX:
      if( state )
        PIN_HI( IO_LED_RX1 );
      else
        PIN_LOW( IO_LED_RX1 );
    break;
  }
}

void pcan_led_poll( void )
{
  uint32_t ts_ms = pcan_timestamp_millis();

  for( int i = 0; i < LED_TOTAL; i++ )
  {
    if( !led_mode_array[i].timestamp  )
      continue;
    if( (uint32_t)( ts_ms - led_mode_array[i].timestamp ) < led_mode_array[i].delay )
      continue;

    switch( led_mode_array[i].mode )
    {
      default:
      case LED_MODE_DEVICE:
        led_mode_array[i].timestamp = 0;
      break;
      case LED_MODE_OFF:
      case LED_MODE_ON:
        led_mode_array[i].state = ( led_mode_array[i].mode == LED_MODE_ON );
        led_mode_array[i].timestamp = 0;
      break;
      case LED_MODE_BLINK_FAST:
      case LED_MODE_BLINK_SLOW:
        led_mode_array[i].state ^= 1;
        led_mode_array[i].timestamp += led_mode_array[i].delay;
        if( ( led_mode_array[i].arg != 0xFFFFFFFF ) && ( led_mode_array[i].arg <= ts_ms ) )
        {
          pcan_led_set_mode( i, LED_MODE_OFF, 0 );
        }
      break;
    }

    _led_update_state( i, led_mode_array[i].state );
  }
}
