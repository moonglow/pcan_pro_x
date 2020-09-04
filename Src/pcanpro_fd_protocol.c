#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "pcanpro_can.h"
#include "pcan_usbpro_fw.h"
#include "pcanfd_usb_fw.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_protocol.h"
#include "pcanpro_led.h"
#include "pcanpro_usbd.h"
#include "usb_device.h"

#define CAN_CHANNEL_MAX     (2)

struct pcan_usbfd_fw_info
{
  uint16_t  size_of;        /* sizeof this */
  uint16_t  type;            /* type of this structure */
  uint8_t    hw_type;        /* Type of hardware (HW_TYPE_xxx) */
  uint8_t    bl_version[3];  /* Bootloader version */
  uint8_t    hw_version;      /* Hardware version (PCB) */
  uint8_t    fw_version[3];  /* Firmware version */
  uint32_t  dev_id[2];      /* "device id" per CAN */
  uint32_t  ser_no;          /* S/N */
  uint32_t  flags;          /* special functions */
  uint8_t   unk[8];
} __attribute__ ((packed));

static struct
{
  uint32_t device_nr;
  uint32_t last_time_sync;
  uint8_t  can_drv_loaded;
  uint8_t  lin_drv_loaded;

  struct
  {
    /* config */
    uint8_t   silient;
    uint8_t   bus_active;
    uint8_t   loopback;
    uint8_t   err_mask;
    uint32_t   channel_nr;

    uint8_t   led_is_busy;

    /* slow speed */
    struct ucan_timing_slow slow_br;
    /* can fd , data fast speed */
    struct ucan_timing_fast fast_br;
    /* clock */
    uint32_t can_clock;
  }
  can[CAN_CHANNEL_MAX];
}
pcan_device =
{
  .device_nr = 0xFFFFFFFF,

  .can[0] = 
  {
    .channel_nr = 0xFFFFFFFF,
    .can_clock = 80000000u
  },
  .can[1] = 
  {
    .channel_nr = 0xFFFFFFFF,
    .can_clock = 80000000u
  },
};

#define PCAN_USB_DATA_BUFFER_SIZE   1024
static uint8_t resp_buffer[2][PCAN_USB_DATA_BUFFER_SIZE];
static uint8_t drv_load_packet[16];

static uint16_t data_pos = 0;
static uint8_t   data_buffer[PCAN_USB_DATA_BUFFER_SIZE];

void *pcan_data_alloc_buffer( uint16_t type, uint16_t size )
{
  uint16_t aligned_size = (size+(4-1))&(~(4-1));
  if( sizeof( data_buffer ) < (aligned_size+data_pos+4) )
    return (void*)0;
  struct ucan_msg *pmsg = (void*)&data_buffer[data_pos];

  pmsg->size = aligned_size;
  pmsg->type = type;
  pmsg->ts_low = pcan_timestamp_us();
  pmsg->ts_high = 0;

  data_pos += aligned_size;
  return pmsg;
}

static struct t_m2h_fsm resp_fsm[2] = 
{
  [0] = {
    .state = 0,
    .ep_addr = PCAN_USB_EP_CMDIN,
    .pdbuf = resp_buffer[0],
    .dbsize = PCAN_USB_DATA_BUFFER_SIZE,
  },
  [1] = {
    .state = 0,
    .ep_addr = PCAN_USB_EP_MSGIN_CH1,
    .pdbuf = resp_buffer[1],
    .dbsize = PCAN_USB_DATA_BUFFER_SIZE,
  }
};

/* low level requests */
uint8_t pcan_protocol_device_setup( USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req )
{
  switch( req->bRequest )
  {
    case USB_VENDOR_REQUEST_INFO:
      switch( req->wValue )
      {
        case USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE:
        {
          static struct pcan_usbfd_fw_info fwi =
          {
            .size_of = sizeof( struct pcan_usbfd_fw_info ),
            .type = 2,
            .hw_type = 1,
            .bl_version = { 2, 1, 0 }, /* bootloader v > 2 support massstorage mode */
            .hw_version = 2,
            .fw_version = { 3, 2, 0 },
            .dev_id[0] = 0xFFFFFFFF,
            .dev_id[1] = 0xFFFFFFFF,
            .ser_no = 0xFFFFFFFF,
            .flags = 0x00000000,
            .unk = { 0x01, 0x81, 0x02, 0x03, 0x82, 0x00, 0x00, 0x00 }
          };
          /* windows/linux has different struct size */
          fwi.size_of = req->wLength;
          fwi.dev_id[0] = pcan_device.can[0].channel_nr;
          fwi.dev_id[1] = pcan_device.can[1].channel_nr;
          fwi.ser_no = pcan_device.device_nr;
          return USBD_CtlSendData( pdev,  (void*)&fwi, fwi.size_of );
        }
        default:
          assert(0);
        break;
      }
      break;
    case USB_VENDOR_REQUEST_FKT:
      switch( req->wValue )
      {
        case USB_VENDOR_REQUEST_wVALUE_SETFKT_BOOT:
          break;
        case USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED:
        {
          USBD_CtlPrepareRx( pdev, drv_load_packet, 16 );
          return USBD_OK;
        }
        break;
        default:
          assert(0);
          break;
      }
      break;
    case USB_VENDOR_REQUEST_ZERO:
      break;
    default:
      USBD_CtlError( pdev, req );
      return USBD_FAIL; 
  }

  return USBD_FAIL;
}

void pcan_ep0_receive( void )
{
  if( drv_load_packet[0] == 0 )
  {
    pcan_flush_ep( PCAN_USB_EP_MSGIN_CH1 );
    pcan_flush_ep( PCAN_USB_EP_CMDIN );
    pcan_device.can_drv_loaded = drv_load_packet[1];
  }
  else
    pcan_device.lin_drv_loaded = drv_load_packet[1];
}

int pcan_protocol_rx_frame( uint8_t channel, struct t_can_msg *pmsg )
{
  struct ucan_rx_msg *pcan_msg = pcan_data_alloc_buffer( UCAN_MSG_CAN_RX, sizeof(struct ucan_rx_msg) + pmsg->size );
  if( !pcan_msg )
    return -1;

  if( !pcan_device.can[channel].led_is_busy && !(pmsg->flags & MSG_FLAG_ECHO) )
  {
    pcan_led_set_mode( channel ? LED_CH1_RX:LED_CH0_RX, LED_MODE_BLINK_FAST, 237 );
  }

  pcan_msg->channel_dlc = UCAN_MSG_CHANNEL_DLC( channel, pmsg->size );
  pcan_msg->client = pmsg->dummy;
  pcan_msg->flags = 0;
  pcan_msg->tag_low = 0;
  pcan_msg->tag_high = 0;

  /* we support only regular frames */
  if( pmsg->flags & MSG_FLAG_RTR )
    pcan_msg->flags |= UCAN_MSG_RTR;
  if( pmsg->flags & MSG_FLAG_EXT )
    pcan_msg->flags |= UCAN_MSG_EXT_ID;
  if( pmsg->flags & MSG_FLAG_ECHO )
  {
    pcan_msg->flags |= UCAN_MSG_API_SRR | UCAN_MSG_HW_SRR;
  }
  else
  {
    pcan_msg->ts_low = pmsg->timestamp;
  }

  pcan_msg->can_id = pmsg->id;
  memcpy( pcan_msg->d, pmsg->data, pmsg->size );
  return 0;
}

int pcan_protocol_tx_frame_cb( uint8_t channel, struct t_can_msg *pmsg )
{
  if( pmsg->flags & MSG_FLAG_ECHO )
  {
    (void)pcan_protocol_rx_frame( channel, pmsg );
  }

  if( !pcan_device.can[channel].led_is_busy )
  {
    pcan_led_set_mode( channel ? LED_CH1_TX:LED_CH0_TX, LED_MODE_BLINK_FAST, 237 );
  }
  return 0;
}

int pcan_protocol_tx_frame( struct ucan_tx_msg *pmsg )
{
  struct t_can_msg msg = { 0 };
  uint8_t channel;

  channel = UCAN_MSG_CHANNEL(pmsg);

  if( channel >= CAN_CHANNEL_MAX )
    return -1;

  msg.id = pmsg->can_id;

  /* CAN-FD frame */
  if( pmsg->flags & UCAN_MSG_EXT_DATA_LEN )
  {
    /* TODO: .... */
    return -1;
  }
  else
  {
    msg.size = UCAN_MSG_DLC(pmsg);
  }

  if( msg.size > sizeof( msg.data ) )
    return -1;

  /* TODO: process UCAN_MSG_SINGLE_SHOT, UCAN_MSG_HW_SRR, UCAN_MSG_ERROR_STATE_IND */
  if( pmsg->flags & UCAN_MSG_RTR )
    msg.flags |= MSG_FLAG_RTR;
  if( pmsg->flags & UCAN_MSG_EXT_ID )
    msg.flags |= MSG_FLAG_EXT;
  if( pmsg->flags & (UCAN_MSG_API_SRR|UCAN_MSG_HW_SRR) )
  {
    msg.flags |= MSG_FLAG_ECHO;
    msg.dummy = pmsg->client;
  }

  memcpy( msg.data, pmsg->d, msg.size );

  msg.timestamp = pcan_timestamp_us();

  if( pcan_can_write( channel, &msg ) < 0 )
  {
    /* TODO: tx queue overflow ? */
    ;
  }
  return 0;
}

static int pcan_protocol_send_status( uint8_t channel, uint8_t status )
{
  struct ucan_status_msg *ps = pcan_data_alloc_buffer( UCAN_MSG_STATUS, sizeof( struct ucan_status_msg ) );
  if( !ps )
    return -1;

  ps->channel_p_w_b = channel&0x0f;
  ps->channel_p_w_b |= (status&0x0f)<<4;
  return 0;
}

int pcan_protocol_set_baudrate( uint8_t channel, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw, uint8_t tsam )
{
#define PCAN_STM32_SYSCLK_HZ    (24000000u)

  uint32_t   bitrate, pcan_bitrate, pcan_brp;
  
  pcan_brp = ((PCAN_STM32_SYSCLK_HZ/1000000u)*brp)/(pcan_device.can[channel].can_clock/1000000u);

  bitrate = (((pcan_device.can[channel].can_clock)/brp)/(1/*tq*/ + tseg1 + tseg2 ));
  pcan_bitrate = (((PCAN_STM32_SYSCLK_HZ)/pcan_brp)/(1/*tq*/ + tseg1 + tseg2 ));

  if( bitrate != pcan_bitrate )
  {
    pcan_can_set_bitrate( channel, bitrate );
  }
  else
  {
    pcan_can_set_bitrate_ex( channel, pcan_brp, tseg1, tseg2, sjw );
  }

  return 0;
}


static void pcan_protocol_process_cmd( uint8_t *ptr, uint16_t size )
{
  struct ucan_command *pcmd = (void*)ptr;

  while( size >= sizeof( struct ucan_command ) )
  {
    switch( UCAN_CMD_OPCODE( pcmd ) )
    {
      case UCAN_CMD_NOP:
        break;
      case UCAN_CMD_RESET_MODE:
      {
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          pcan_can_set_bus_active( UCAN_CMD_CHANNEL(pcmd) , 0 );
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].bus_active = 0;
        }
      }
        break;
      case UCAN_CMD_NORMAL_MODE:
      {
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          if( pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].silient  )
          {
            pcan_can_set_silent( UCAN_CMD_CHANNEL(pcmd) , 0 );
            pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].silient = 0;
          }
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].bus_active = 1;
          pcan_can_set_bus_active( UCAN_CMD_CHANNEL(pcmd) , 1 );
          pcan_protocol_send_status( UCAN_CMD_CHANNEL(pcmd), 0 );
        }
      }
        break;
      case UCAN_CMD_LISTEN_ONLY_MODE:
      {
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          pcan_can_set_silent( UCAN_CMD_CHANNEL(pcmd) , 1 );
          pcan_can_set_bus_active( UCAN_CMD_CHANNEL(pcmd) , 1 );
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].silient = 1;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].bus_active = 1;
          pcan_protocol_send_status( UCAN_CMD_CHANNEL(pcmd), 0 );
        }
      }
        break;
      case UCAN_CMD_TIMING_SLOW:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_timing_slow *ptiming = (void*)pcmd;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].slow_br = *ptiming;
          pcan_protocol_set_baudrate( UCAN_CMD_CHANNEL(pcmd),
                                                             ptiming->brp + 1,
                                                             ptiming->tseg1 + 1,
                                                             ptiming->tseg2 + 1,
                                                             (ptiming->sjw_t & 0x0f)+1,
                                                             (ptiming->sjw_t&0x80) != 0  );
        }
        break;
      /* only for CAN-FD */
      case UCAN_CMD_TIMING_FAST:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_timing_fast *ptiming = (void*)pcmd;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].fast_br = *ptiming;
        }
        break;
      case UCAN_CMD_SET_STD_FILTER:
        if( UCAN_CMD_CHANNEL(pcmd) >= CAN_CHANNEL_MAX )
          break;
        pcan_can_set_filter_mask( UCAN_CMD_CHANNEL(pcmd), 0, 0, 0, 0 );
        break;
      case UCAN_CMD_RESERVED2:
        break;
      case UCAN_CMD_FILTER_STD:
        if( UCAN_CMD_CHANNEL(pcmd) >= CAN_CHANNEL_MAX )
          break;
        pcan_can_set_filter_mask( UCAN_CMD_CHANNEL(pcmd), 0, 0, 0, 0 );
        break;
      case UCAN_CMD_TX_ABORT:
        break;
      case UCAN_CMD_WR_ERR_CNT:
        break;
      case UCAN_CMD_SET_EN_OPTION:
        break;
      case UCAN_CMD_CLR_DIS_OPTION:
        break;
      case UCAN_CMD_SET_ERR_GEN1:
        break;
      case UCAN_CMD_SET_ERR_GEN2:
        break;
      case UCAN_CMD_DIS_ERR_GEN:
        break;
      case UCAN_CMD_RX_BARRIER:
        break;
      case UCAN_CMD_SET_ERR_GEN_S:
        break;
      case UCAN_USB_CMD_CLK_SET:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_usb_clock *pclock = (void*)pcmd;
          switch( pclock->mode )
          {
            default:
            case UCAN_USB_CLK_80MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 80000000u;
            break;
            case UCAN_USB_CLK_60MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 60000000u;
            break;
            case UCAN_USB_CLK_40MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 40000000u;
            break;
            case UCAN_USB_CLK_30MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 30000000u;
            break;
            case UCAN_USB_CLK_24MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 24000000u;
            break;
            case UCAN_USB_CLK_20MHZ:
              pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].can_clock = 20000000u;
            break;
          }
        }
        break;
      case UCAN_USB_CMD_LED_SET:
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          struct ucan_usb_led *pled = (void*)pcmd;
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].led_is_busy = pled->mode;
          pcan_led_set_mode( UCAN_CMD_CHANNEL(pcmd), pled->mode, 0xFFFFFFFF );
        }
        break;
      case UCAN_USB_CMD_DEVID_SET:
      {
        struct ucan_usb_device_id *pdevid = (void*)pcmd;
        if( UCAN_CMD_CHANNEL(pcmd) < CAN_CHANNEL_MAX )
        {
          pcan_device.can[UCAN_CMD_CHANNEL(pcmd)].channel_nr = pdevid->device_id;
        }
      }
        break;
      case 0x87: /* CAN FD ISO MODE */
        break;
      case UCAN_CMD_END_OF_COLLECTION:
        return;
      default:
        assert( 0 );
        break;
    }

    size -= sizeof( struct ucan_command );
    ++pcmd;
  }
}

void pcan_protocol_process_data( uint8_t ep, uint8_t *ptr, uint16_t size )
{
  if( ep == 1 )
  {
    pcan_protocol_process_cmd( ptr, size );
    return;
  }
  /* message data ? */
  struct ucan_msg *pmsg = 0;

  while( size )
  {
    if( size < 4 )
      break;
    pmsg = (void*)ptr;
    if( !pmsg->size || !pmsg->type )
      break;
    if( size < pmsg->size )
      break;
    size -= pmsg->size;
    ptr += pmsg->size;

    switch( pmsg->type )
    {
      //to host only
      //UCAN_MSG_ERROR:
      //UCAN_MSG_BUSLOAD:
      case UCAN_MSG_CAN_TX:
        pcan_protocol_tx_frame( (struct ucan_tx_msg *)pmsg );
        break;
      case UCAN_MSG_CAN_TX_PAUSE:
        /* TODO: */
        break;
      case UCAN_CMD_END_OF_COLLECTION:
      case 0xffff:
        return;
      default:
        assert( 0 );
        break;  
    }
  }
}

void pcan_protocol_init( void )
{
  pcan_can_init_ex( CAN_BUS_1, 500000 );
  pcan_can_set_filter_mask( CAN_BUS_1, 0, 0, 0, 0 );
  pcan_can_init_ex( CAN_BUS_2, 500000 );
  pcan_can_set_filter_mask( CAN_BUS_2, 0, 0, 0, 0 );

  pcan_can_install_rx_callback( CAN_BUS_1, pcan_protocol_rx_frame );
  pcan_can_install_rx_callback( CAN_BUS_2, pcan_protocol_rx_frame );

  pcan_can_install_tx_callback( CAN_BUS_1, pcan_protocol_tx_frame_cb );
  pcan_can_install_tx_callback( CAN_BUS_2, pcan_protocol_tx_frame_cb );
}

void pcan_protocol_poll( void )
{
  uint32_t ts_ms = pcan_timestamp_millis();

  pcan_can_poll();

  /* flush data */
  if( data_pos > 0 )
  {
    /* endmark */
    *(uint32_t*)&data_buffer[data_pos] = 0x00000000;
    uint16_t flush_size = data_pos + 4;
    /* align to 64 */
    flush_size += (64-1);
    flush_size &= ~(64-1);
    int res = pcan_flush_data( &resp_fsm[1], data_buffer, flush_size);
    if( res )
    { 
      data_pos = 0;
    }
  }

  /* timesync part */
  if( !pcan_device.can_drv_loaded )
    return;
  if( ( ts_ms - pcan_device.last_time_sync ) < 1000u )
    return;
  struct ucan_usb_ts_msg *pts = pcan_data_alloc_buffer( UCAN_USB_MSG_CALIBRATION, sizeof( struct ucan_usb_ts_msg ) );
  if( !pts )
    return;

  pts->usb_frame_index = pcan_usb_frame_number();
  pts->unused = 0;
  pcan_device.last_time_sync = ts_ms;
}
