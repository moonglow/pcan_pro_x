#include <assert.h>
#include "io_macro.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_led.h"
#include "pcanpro_variant.h"

static struct
{
  uint32_t arg;
  uint32_t delay;
  uint32_t timestamp;
  uint16_t mode;
  uint8_t  state;
}
led_mode_array[LED_TOTAL] = { 0 };

void pcan_led_init( void )
{
  IO_HW_INIT();

#ifdef IO_LED_STAT
  PORT_ENABLE_CLOCK( PIN_PORT( IO_LED_STAT ), PIN_PORT( IO_LED_STAT ) );
#endif
#ifdef IO_LED_TX0
  PORT_ENABLE_CLOCK( PIN_PORT( IO_LED_TX0 ), PIN_PORT( IO_LED_TX0 ) );
#endif
#ifdef IO_LED_TX1
  PORT_ENABLE_CLOCK( PIN_PORT( IO_LED_TX1 ), PIN_PORT( IO_LED_TX1 ) );
#endif
#ifdef IO_LED_RX0
  PORT_ENABLE_CLOCK( PIN_PORT( IO_LED_RX0 ), PIN_PORT( IO_LED_RX0 ) );
#endif
#ifdef IO_LED_RX1
  PORT_ENABLE_CLOCK( PIN_PORT( IO_LED_RX1 ), PIN_PORT( IO_LED_RX1 ) );
#endif
  
#ifdef IO_LED_STAT
  IO_LED_LOW( IO_LED_STAT );
  PIN_INIT( IO_LED_STAT );
#endif
#ifdef IO_LED_TX0
  IO_LED_LOW( IO_LED_TX0 );
  PIN_INIT( IO_LED_TX0 );
#endif
#ifdef IO_LED_RX0
  IO_LED_LOW( IO_LED_RX0 );
  PIN_INIT( IO_LED_RX0 );
#endif
#ifdef IO_LED_TX1
  IO_LED_LOW( IO_LED_TX1 );
  PIN_INIT( IO_LED_TX1 );
#endif
#ifdef IO_LED_RX1
  IO_LED_LOW( IO_LED_RX1 );
  PIN_INIT( IO_LED_RX1 );
#endif
}

void pcan_led_set_mode( int led, int mode, uint32_t arg )
{
  assert( led < LED_TOTAL );

  led_mode_array[led].mode = mode;
  if( !led_mode_array[led].timestamp )
  {
    led_mode_array[led].timestamp = pcan_timestamp_millis();
  }
  led_mode_array[led].delay = 0;

  /* set guard time */
  if( mode == LED_MODE_BLINK_FAST || mode == LED_MODE_BLINK_SLOW )
  {
    led_mode_array[led].delay = ( mode == LED_MODE_BLINK_FAST ) ? 50: 200;
    if( arg != 0xFFFFFFFF )
    {
      /* update to absolute */
      arg = pcan_timestamp_millis() + arg;
    }
  }

  led_mode_array[led].arg  = arg;
}

static void _led_update_state( int led, uint8_t state )
{
  switch( led )
  {
    case LED_STAT:
#ifdef IO_LED_STAT
      if( state )
        IO_LED_HI( IO_LED_STAT );
      else
        IO_LED_LOW( IO_LED_STAT );
#endif
    break;
    case LED_CH0_TX:
#ifdef IO_LED_TX0
      if( state )
        IO_LED_HI( IO_LED_TX0 );
      else
        IO_LED_LOW( IO_LED_TX0 );
#endif
    break;
    case LED_CH0_RX:
#ifdef IO_LED_RX0
      if( state )
        IO_LED_HI( IO_LED_RX0 );
      else
        IO_LED_LOW( IO_LED_RX0 );
#endif
    break;
    case LED_CH1_TX:
#ifdef IO_LED_TX1
      if( state )
        IO_LED_HI( IO_LED_TX1 );
      else
        IO_LED_LOW( IO_LED_TX1 );
#endif
    break;
    case LED_CH1_RX:
#ifdef IO_LED_RX1
      if( state )
        IO_LED_HI( IO_LED_RX1 );
      else
        IO_LED_LOW( IO_LED_RX1 );
#endif
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
