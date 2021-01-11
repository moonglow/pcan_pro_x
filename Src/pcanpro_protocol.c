#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "pcanpro_can.h"
#include "pcan_usbpro_fw.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_protocol.h"
#include "pcanpro_led.h"
#include "pcanpro_usbd.h"
#include "usb_device.h"

#define CAN_CHANNEL_MAX     (2)

#define PCAN_USBPRO_RTR      0x01
#define PCAN_USBPRO_EXT      0x02
#define PCAN_USBPRO_SR      0x80

/* PCAN-USB-PRO status flags */
#define PCAN_USBPRO_BUS_HEAVY      0x01
#define PCAN_USBPRO_BUS_OVERRUN    0x0c

static struct
{
  struct
  {
    /* error handling related */
    uint8_t   err;
    uint8_t   ecc;
    uint8_t   rx_err;
    uint8_t   tx_err;
    /* config */
    uint8_t   silient;
    uint8_t   bus_active;
    uint8_t   loopback;
    uint8_t   err_mask;

    uint32_t   ccbt; /* current baudrate value */
    uint32_t   channel_nr;

    uint8_t   led_is_busy;
  }
  can[CAN_CHANNEL_MAX];

  uint32_t device_nr;
  uint32_t time_calibration_mode;
  uint32_t last_time_sync;
  uint32_t last_flush;
}
pcan_device =
{
  .device_nr = 0xFFFFFFFF,

  .can[0] = 
  {
    .channel_nr = 0xFFFFFFFF
  },
  .can[1] = 
  {
    .channel_nr = 0xFFFFFFFF
  },
};

/* internal structure used to handle messages sent to bulk urb */
struct pcan_usbpro_msg 
{
  uint8_t *  rec_ptr;
  int  rec_buffer_size;
  int  rec_buffer_len;
  union
  {
    uint16_t * rec_counter_read;
    uint32_t * rec_counter;
    uint8_t  * rec_buffer;
  } u;
};

#define PCAN_USB_DATA_BUFFER_SIZE   1024
static uint8_t resp_buffer[2][PCAN_USB_DATA_BUFFER_SIZE];
static uint8_t temp_resp_buffer[2][PCAN_USB_DATA_BUFFER_SIZE];
static uint8_t drv_load_packet[16];
static struct pcan_usbpro_msg resp[2];

static struct t_m2h_fsm resp_fsm[2] = 
{
  [0] = {
    .state = 0,
    .ep_addr = PCAN_USB_EP_CMDIN,
    .pdbuf = temp_resp_buffer[0],
    .dbsize = PCAN_USB_DATA_BUFFER_SIZE,
  },
  [1] = {
    .state = 0,
    .ep_addr = PCAN_USB_EP_MSGIN_CH1,
    .pdbuf = temp_resp_buffer[1],
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
        case USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER:
        {
          static struct pcan_usbpro_bootloader_info bi = 
          {
            .ctrl_type = BOOTLOADER_INFO_STRUCT_TYPE,
            .version[0] = 0,
            .version[1] = 0,
            .version[2] = 0,
            .version[3] = 0,
            .day = 0,
            .month = 0,
            .year = 0,
            .dummy = 0,
            .serial_num_high = 200030,
            .serial_num_low = 1,
            .hw_type = 0,
            .hw_rev = 0,
          };

          return USBD_CtlSendData( pdev, (void*)&bi, sizeof( struct pcan_usbpro_bootloader_info ) );
        }
        case USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE:
        {
          static struct pcan_usbpro_ext_firmware_info fwi =
          {
            .ctrl_type = EXT_FIRMWARE_INFO_STRUCT_TYPE,
            .version[0] = 1,
            .version[1] = 3,
            .version[2] = 3,
            .version[3] = 0,
            .day = 0,
            .month = 0,
            .year = 0,
            .dummy = 0,
            .fw_type = 0,
          };

          return USBD_CtlSendData( pdev,  (void*)&fwi, sizeof( struct pcan_usbpro_ext_firmware_info ) );
        }
        case USB_VENDOR_REQUEST_wVALUE_INFO_uC_CHIPID:
        {
          static struct pcan_usbpro_uc_chipid uc_chid =
          {
            .ctrl_type = uC_CHIPID_STRUCT_TYPE,
            .chip_id = 0,
          };

          return USBD_CtlSendData( pdev,  (void*)&uc_chid, sizeof( struct pcan_usbpro_uc_chipid ) );
        }
        case USB_VENDOR_REQUEST_wVALUE_INFO_USB_CHIPID:
        {
          static struct pcan_usbpro_usb_chipid usb_chid = 
          {
            .ctrl_type = USB_CHIPID_STRUCT_TYPE,
            .chip_id = 0,
          };

          return USBD_CtlSendData( pdev,  (void*)&usb_chid, sizeof( struct pcan_usbpro_usb_chipid ) );
        }
        case USB_VENDOR_REQUEST_wVALUE_INFO_DEVICENR:
        {
          static struct pcan_usbpro_device_nr device_nr = 
          {
            .ctrl_type = DEVICE_NR_STRUCT_TYPE,
            .device_nr = 0xFFFFFFFF,
          };

          return USBD_CtlSendData( pdev,  (void*)&device_nr, sizeof( struct pcan_usbpro_device_nr ) );
        }
        case USB_VENDOR_REQUEST_wVALUE_INFO_CPLD:
        {
          static struct pcan_usbpro_cpld_info cpldi = 
          {
            .ctrl_type = CPLD_INFO_STRUCT_TYPE,
            .cpld_nr = 0,
          };

          return USBD_CtlSendData( pdev,  (void*)&cpldi, sizeof( struct pcan_usbpro_cpld_info ) );
        }

        case USB_VENDOR_REQUEST_wVALUE_INFO_MODE:
        {
          static struct pcan_usbpro_info_mode info = { 0 };

          return USBD_CtlSendData( pdev,  (void*)&info, sizeof( struct pcan_usbpro_info_mode ) );
        }
        case USB_VENDOR_REQUEST_wVALUE_INFO_TIMEMODE:
        {
          static struct pcan_usbpro_time_mode info = { 0 };

          return USBD_CtlSendData( pdev,  (void*)&info, sizeof( struct pcan_usbpro_time_mode ) );
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
        case USB_VENDOR_REQUEST_wVALUE_SETFKT_DEBUG_CAN:
          break;
        case USB_VENDOR_REQUEST_wVALUE_SETFKT_DEBUG_LIN:
          break;
        case USB_VENDOR_REQUEST_wVALUE_SETFKT_DEBUG1:
          break;
        case USB_VENDOR_REQUEST_wVALUE_SETFKT_DEBUG2:
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
  /* CAN */
  if( drv_load_packet[0] == 0 )
  {
    pcan_flush_ep( PCAN_USB_EP_MSGIN_CH1 );
    pcan_flush_ep( PCAN_USB_EP_CMDIN );
  }
  else
  {
    /* LIN */
    ;
  }
}

/*
 * static int pcan_usbpro_sizeof_rec(uint8_t data_type)
 */
static int pcan_usbpro_sizeof_rec(uint8_t data_type)
{
  switch (data_type)
  {
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8:
    return sizeof(struct pcan_usbpro_canmsg_rx);
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_4:
    return sizeof(struct pcan_usbpro_canmsg_rx) - 4;
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_0:
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX:
    return sizeof(struct pcan_usbpro_canmsg_rx) - 8;

  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_STATUS_ERROR_RX:
    return sizeof(struct pcan_usbpro_canmsg_status_error_rx);

  case DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX:
    return sizeof(struct pcan_usbpro_calibration_ts_rx);

  case DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX:
    return sizeof(struct pcan_usbpro_buslast_rx);

  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8:
    return sizeof(struct pcan_usbpro_canmsg_tx);
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4:
    return sizeof(struct pcan_usbpro_canmsg_tx) - 4;
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0:
    return sizeof(struct pcan_usbpro_canmsg_tx) - 8;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE:
    return sizeof(struct pcan_usbpro_baudrate);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
    return sizeof(struct pcan_usbpro_bus_activity);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE:
    return sizeof(struct pcan_usbpro_silent_mode);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
    return sizeof(struct pcan_usbpro_dev_nr);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETWARNINGLIMIT:
    return sizeof(struct pcan_usbpro_warning_limit);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_EXPLICIT:
    return sizeof(struct pcan_usbpro_lookup_explicit);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_GROUP:
    return sizeof(struct pcan_usbpro_lookup_group);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE:
    return sizeof(struct pcan_usbpro_filter_mode);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETRESET_MODE:
    return sizeof(struct pcan_usbpro_reset_mode);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETERRORFRAME:
    return sizeof(struct pcan_usbpro_error_frame);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS:
    return sizeof(struct pcan_usbpro_error_status);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETREGISTER:
    return sizeof(struct pcan_usbpro_set_register);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETREGISTER:
    return sizeof(struct pcan_usbpro_get_register);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG:
    return sizeof(struct pcan_usbpro_calibration);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG:
    return sizeof(struct pcan_usbpro_buslast);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSTRING:
    return sizeof(struct pcan_usbpro_set_string);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETSTRING:
    return sizeof(struct pcan_usbpro_get_string);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_STRING:
    return sizeof(struct pcan_usbpro_string);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SAVEEEPROM:
    return sizeof(struct pcan_usbpro_save_eeprom);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_USB_IN_PACKET_DELAY:
    return sizeof(struct pcan_usbpro_packet_delay);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_TIMESTAMP_PARAM:
    return sizeof(struct pcan_usbpro_timestamp_param);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_ID:
    return sizeof(struct pcan_usbpro_error_gen_id);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_NOW:
    return sizeof(struct pcan_usbpro_error_gen_now);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_SOFTFILER:
    return sizeof(struct pcan_usbpro_softfiler);

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
    return sizeof(struct pcan_usbpro_set_can_led);

  default:
    assert( 0 );
  }

  return -1;
}

/*
 * static uint8_t * pcan_usbpro_msg_init(struct pcan_usbpro_msg *pm,
 *                                       void *buffer_addr, int buffer_size)
 *
 * Initialize PCAN USB-PRO message data structure
 */
static uint8_t * pcan_usbpro_msg_init(struct pcan_usbpro_msg *pm,
         void *buffer_addr, int buffer_size)
{
  if (buffer_size < 4)
    return NULL;

  pm->u.rec_buffer = (uint8_t *)buffer_addr;
  pm->rec_buffer_size = pm->rec_buffer_len = buffer_size;
  pm->rec_ptr = pm->u.rec_buffer + 4;

  return pm->rec_ptr;
}

static uint8_t * pcan_usbpro_msg_init_empty(struct pcan_usbpro_msg *pm,
               void *buffer_addr, int buffer_size)
{
  uint8_t *pr = pcan_usbpro_msg_init(pm, buffer_addr, buffer_size);
  if (pr) {
    pm->rec_buffer_len = 4;
    *pm->u.rec_counter = 0;
  }
  return pr;
}

static void pcan_usbpro_msg_reset( struct pcan_usbpro_msg *pm )
{
  pm->rec_ptr = pm->u.rec_buffer + 4;
  pm->rec_buffer_len = 4;
  *pm->u.rec_counter = 0;
}

/*
 * static int pcan_usbpro_msg_add_rec(struct pcan_usbpro_msg *pm,
 *                                    int id, ...)
 *
 * Add one record to a message being built.
 */
static int pcan_usbpro_msg_add_rec(struct pcan_usbpro_msg *pm,
                                  int id, ...)
{
  int l, i;
  uint8_t *pc;
  va_list ap;

  va_start(ap, id);

  if( pm->rec_buffer_size < ( pcan_usbpro_sizeof_rec( id ) + pm->rec_buffer_len ))
  {
    for(;;);
  }

  pc = pm->rec_ptr + 1;

  i = 0;
  switch (id) {
  case DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX:
    *pc++ = (uint8_t )va_arg(ap, int);            // channel
    *(uint16_t *)pc = (uint16_t )va_arg(ap, int); // buslast_val
    pc += 2;
    *(uint32_t *)pc = va_arg(ap, uint32_t);  // timestamp
    pc += 4;
  break;
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8:
    i += 4;
    /* fall through */
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4:
    i += 4;
    /* fall through */
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0:
    *pc++ = (uint8_t )va_arg(ap, int);  // client
    *pc++ = (uint8_t )va_arg(ap, int);  // flags
    *pc++ = (uint8_t )va_arg(ap, int);  // len
    *(uint32_t *)pc = va_arg(ap, uint32_t);  // id
    pc += 4;
    memcpy(pc, va_arg(ap, int *), i);
    pc += i;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8:
    i += 4;
    /* fall through */
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_4:
    i += 4;
    /* fall through */
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_0:
  case DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX:
    *pc++ = (uint8_t )va_arg(ap, int);  // client
    *pc++ = (uint8_t )va_arg(ap, int);  // flags
    *pc++ = (uint8_t )va_arg(ap, int);  // len
    *(uint32_t *)pc = va_arg(ap, uint32_t);  // timestamp
    pc += 4;
    *(uint32_t *)pc = va_arg(ap, uint32_t);  // id
    pc += 4;
    memcpy(pc, va_arg(ap, int *), i);
    pc += i;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX:
    pc += 3; // dummy
    *(uint32_t *)pc = va_arg(ap, uint32_t); // timestamp64[0] usb frame index
    pc += 4;
    *(uint32_t *)pc = va_arg(ap, uint32_t);  // timestamp64[1] ts us
    pc += 4;
    break;
  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    *(uint16_t *)pc = (uint16_t )va_arg(ap, int);  // onoff,status
    pc += 2;
    break;
  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    pc += 2; // dummy
    /* CCBT, devicenr */
    *(uint32_t *)pc = va_arg(ap, uint32_t);
    pc += 4;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETWARNINGLIMIT:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETRESET_MODE:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETERRORFRAME:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_TIMESTAMP_PARAM:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_NOW:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    /* onoff, silentmode, warninglimit, filter_mode, reset, mode, */
    /* start_or_end, bit_pos */
    *(uint16_t *)pc = (uint16_t )va_arg(ap, int);
    pc += 2;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_EXPLICIT:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    *(uint16_t *)pc = (uint16_t )va_arg(ap, int); // id_type,mode
    pc += 2;
    *(uint32_t *)pc = va_arg(ap, uint32_t); // id, timeout
    pc += 4;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETLOOKUP_GROUP:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    *(uint16_t *)pc = (uint16_t )va_arg(ap, int); // id_type
    pc += 2;
    *(uint32_t *)pc = va_arg(ap, uint32_t); // id_start
    pc += 4;
    *(uint32_t *)pc = va_arg(ap, uint32_t); // id_end
    pc += 4;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETREGISTER:
    *pc++ = (uint8_t )va_arg(ap, int);  // irq_off
    pc += 2; // dummy
    *(uint32_t *)pc = va_arg(ap, uint32_t); // address
    pc += 4;
    *(uint32_t *)pc = va_arg(ap, uint32_t); // value
    pc += 4;
    *(uint32_t *)pc = va_arg(ap, uint32_t); // mask
    pc += 4;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETREGISTER:
    *pc++ = (uint8_t )va_arg(ap, int);  // irq_off
    pc += 2; // dummy
    *(uint32_t *)pc = va_arg(ap, uint32_t); // address
    pc += 4;
    *(uint32_t *)pc = va_arg(ap, uint32_t); // value
    pc += 4;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_USB_IN_PACKET_DELAY:
    pc++; // dummy
    /* mode, delay */
    *(uint16_t *)pc = (uint16_t )va_arg(ap, int); // mode
    pc += 2;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    pc++; // dummy
    *pc++ = (uint8_t )va_arg(ap, int);  // mode
    //*(uint16_t *)pc = (uint16_t )va_arg(ap, int)); // prescaler
    pc += 2; // prescale (readonly)
    //*(uint16_t *)pc = (uint16_t )va_arg(ap, int)); // sampletimequanta
    *(uint16_t *)pc = 4096; // sampletimequanta
    pc += 2;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSTRING:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    *pc++ = (uint8_t )va_arg(ap, int);  // offset
    *pc++ = (uint8_t )va_arg(ap, int);  // len
    memcpy(pc, va_arg(ap, uint8_t *), 60);
    pc += 60;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_GETSTRING:
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SAVEEEPROM:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    pc += 2; // dummy
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_STRING:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    pc += 2; // dummy
    memcpy(pc, va_arg(ap, uint8_t *), 250);
    pc += 250;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_ERROR_GEN_ID:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    *(uint16_t *)pc = (uint16_t )va_arg(ap, int); // bit_pos
    pc += 2;
    *(uint32_t *)pc = va_arg(ap, uint32_t); // id
    pc += 4;
    *(uint16_t *)pc = (uint16_t )va_arg(ap, int); // ok_counter
    pc += 2;
    *(uint16_t *)pc = (uint16_t )va_arg(ap, int); //error_counter
    pc += 2;
    break;

  case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_SOFTFILER:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    pc += 2;
    *(uint32_t *)pc = va_arg(ap, uint32_t); // accmask
    pc += 4;
    *(uint32_t *)pc = va_arg(ap, uint32_t); // acccode
    pc += 4;
    break;

#if 0
  case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
    *pc++ = (uint8_t )va_arg(ap, int);  // channel
    *(uint16_t *)pc =  (uint16_t )va_arg(ap, int); // mode
    pc += 2;
    *(uint32_t *)pc =  va_arg(ap, uint32_t); // timeout
    pc += 4;
    break;
#endif

  default:
    assert( 0 );
    break;
  }

  l = pc - pm->rec_ptr;
  if (l > 0) {
    *pm->u.rec_counter = *pm->u.rec_counter+1;
    *(pm->rec_ptr) = (uint8_t )id;

    pm->rec_ptr = pc;
    pm->rec_buffer_len += l;
  }

  va_end(ap);

  return l;
}

int pcan_protocol_set_baudrate( uint8_t channel, uint32_t  ccbt )
{
#define PCAN_USBPRO_SYSCLK_HZ    (56000000u)
#define PCAN_STM32_SYSCLK_HZ    (24000000u)
  uint32_t   brp, pcan_brp;
  uint32_t  tseg1, tseg2;
  uint32_t  sjw;
  uint32_t  tsam;    /* triple sampling */
  uint32_t   bitrate, pcan_bitrate;
  
  tsam = ( ccbt >> 23 ) & 1;
  (void)tsam;
  tseg2 = (( ccbt >> 20) & 0x07 ) + 1;
  tseg1 = (( ccbt >> 16) & 0x0f ) + 1;
  sjw = (( ccbt >> 14) & 0x03 ) + 1;
  brp = ( ccbt & 0x3fff ) + 1;

  pcan_brp = (24*brp)/56;

  bitrate = (((PCAN_USBPRO_SYSCLK_HZ)/brp)/(1/*tq*/ + tseg1 + tseg2 ));
  pcan_bitrate = (((PCAN_STM32_SYSCLK_HZ)/pcan_brp)/(1/*tq*/ + tseg1 + tseg2 ));

  if( bitrate != pcan_bitrate )
  {
    pcan_can_set_bitrate( channel, bitrate, 0 );
  }
  else
  {
    pcan_can_set_bitrate_ex( channel, pcan_brp, tseg1, tseg2, sjw );
  }
  
  /* save driver value without adjustments */
  pcan_device.can[channel].ccbt = ccbt;
  return 0;
}

int pcan_protocol_rx_frame( uint8_t channel, struct t_can_msg *pmsg )
{
  uint8_t rec_type;
  uint8_t  client;
  uint8_t  flags;

  if( !pcan_device.can[channel].led_is_busy && !(pmsg->flags & MSG_FLAG_ECHO) )
  {
    pcan_led_set_mode( channel ? LED_CH1_RX:LED_CH0_RX, LED_MODE_BLINK_FAST, 237 );
  }

  if( pmsg->flags & MSG_FLAG_RTR )
    rec_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_RTR_RX;
  else if( pmsg->size == 0 )
    rec_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_0;
  else if( pmsg->size <= 4 )
    rec_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_4;
  else
    rec_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8;

  client = 0;
  flags = 0;

  if( pmsg->flags & MSG_FLAG_ECHO )
  {
    flags |= 0x04; /* SELF RECIEVE */
    client = pmsg->dummy;
  }
  if( pmsg->flags & MSG_FLAG_RTR )
    flags |= PCAN_USBPRO_RTR;
  if( pmsg->flags & MSG_FLAG_EXT )
    flags |= PCAN_USBPRO_EXT;

  pcan_usbpro_msg_add_rec( &resp[PCAN_USB_BUFFER_DATA], rec_type,
                                 client, flags,
                                 ( (channel<<4)| pmsg->size),
                                 pmsg->timestamp,
                                 pmsg->id,
                                 pmsg->data );

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

int pcan_protocol_tx_frame( struct pcan_usbpro_canmsg_tx *pmsg )
{
  struct t_can_msg msg = { 0 };
  uint8_t channel;

  channel = (pmsg->len>>4)&0x0f;

  if( channel >= CAN_CHANNEL_MAX )
    return -1;

  msg.id = pmsg->id;
  msg.size = pmsg->len & 0x0f;

  if( msg.size > sizeof( msg.data ) )
    return -1;

  if (pmsg->flags & PCAN_USBPRO_RTR)
    msg.flags |= MSG_FLAG_RTR;
  if (pmsg->flags & PCAN_USBPRO_EXT)
    msg.flags |= MSG_FLAG_EXT;
  if (pmsg->client & PCAN_USBPRO_SR)
  {
    msg.flags |= MSG_FLAG_ECHO;
    msg.dummy = pmsg->client;
  }

  memcpy( msg.data, pmsg->data, msg.size );

  msg.timestamp = pcan_timestamp_us();

  if( pcan_can_write( channel, &msg ) < 0 )
  {
    /* tx queue overflow ? */
    pcan_device.can[channel].err |= 0;
  }
#if 0
  if( msg.flags & CAN_FLAG_ECHO )
  {
    (void)pcan_protocol_rx_frame( channel, &msg );
  }
#endif
  return 0;
}

void pcan_protocol_process_data( uint8_t ep, uint8_t *ptr, uint16_t size )
{
  struct pcan_usbpro_msg m = { 0 };
  uint8_t *rec_ptr = 0;
  static volatile uint32_t wow_big = 0;
  if( size > 64 )
  {
    ++wow_big;
  }

  rec_ptr = pcan_usbpro_msg_init( &m, ptr, size );
  
  if( !rec_ptr )
    return;

  int buffer_ep = ( ep == 1 ) ? PCAN_USB_BUFFER_CMD: PCAN_USB_BUFFER_DATA;
  uint32_t r;

  for( r = 0; r < *m.u.rec_counter_read; r++ )
  {
    int rec_size;
    union pcan_usbpro_rec *prec = (void*)rec_ptr;

    rec_size = pcan_usbpro_sizeof_rec( prec->data_type );
     
    if( rec_size < 0 || size < ( rec_size + 4 ) )
      return; /* bad data */

    switch( prec->data_type )
    {
      default:
        assert( 0 );
      break;
      /* windows only */
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SETWARNINGLIMIT:
      break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE:
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SETERRORFRAME:
        break;
      case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8:
      case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4:
      case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0:
        if( (prec->canmsg_tx.len>>4) < CAN_CHANNEL_MAX )
        {
          (void)pcan_protocol_tx_frame( &prec->canmsg_tx );
        }
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE:
        if( prec->baudrate.channel < CAN_CHANNEL_MAX )
        {
          pcan_usbpro_msg_add_rec( &resp[buffer_ep], DATA_TYPE_USB2CAN_STRUCT_FKT_GETBAUDRATE,
                                 prec->baudrate.channel, pcan_device.can[prec->dev_nr.channel].ccbt );
        }
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE:
        if( prec->baudrate.channel < CAN_CHANNEL_MAX )
        {
          (void)pcan_protocol_set_baudrate( prec->baudrate.channel, prec->baudrate.CCBT );
        }
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE:
        if( prec->bus_activity.channel < CAN_CHANNEL_MAX )
        {
          pcan_usbpro_msg_add_rec( &resp[buffer_ep], DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUSACTIVATE,
                                 prec->bus_activity.channel, pcan_device.can[prec->dev_nr.channel].bus_active );
        }
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
        if( prec->bus_activity.channel < CAN_CHANNEL_MAX )
        {
          uint8_t ch = prec->bus_activity.channel;
          pcan_device.can[ch].bus_active = prec->bus_activity.onoff;
          pcan_can_set_bus_active( (ch == 0)?CAN_BUS_1:CAN_BUS_2, prec->bus_activity.onoff );

          if( !pcan_device.can[ch].led_is_busy )
          {
            pcan_led_set_mode( ch ? LED_CH1_RX:LED_CH0_RX, prec->bus_activity.onoff ? LED_MODE_ON: LED_MODE_OFF, 0xFFFFFFFF );
          }
        }
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE:
        if( prec->silent_mode.channel < CAN_CHANNEL_MAX )
        {
          pcan_device.can[prec->bus_activity.channel].silient = prec->silent_mode.onoff;
          pcan_can_set_silent( (prec->silent_mode.channel == 0)?CAN_BUS_1:CAN_BUS_2, prec->silent_mode.onoff );
        }
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS:
        if( prec->error_status.channel < CAN_CHANNEL_MAX )
        {
          pcan_usbpro_msg_add_rec( &resp[buffer_ep], DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS,
            prec->error_status.channel, 0x0000 );
        }
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG:
        pcan_device.time_calibration_mode = prec->calibration.mode;
        if( !pcan_device.time_calibration_mode )
          break;
        pcan_device.last_time_sync = 0;
        pcan_usbpro_msg_add_rec( &resp[PCAN_USB_BUFFER_DATA], DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX,
                                 pcan_usb_frame_number(), pcan_timestamp_us() );
        
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG:
        /* TODO: */
        (void)prec->buslast.channel;
        (void)prec->buslast.mode;
        (void)prec->buslast.prescaler;
        (void)prec->buslast.sampletimequanta;

        pcan_usbpro_msg_add_rec( &resp[PCAN_USB_BUFFER_DATA], DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX,
                                 prec->buslast.channel, 0, pcan_timestamp_us() );
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
        if( prec->dev_nr.channel < CAN_CHANNEL_MAX )
        {
          pcan_usbpro_msg_add_rec( &resp[PCAN_USB_BUFFER_CMD], DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR,
                                 prec->dev_nr.channel, pcan_device.can[prec->dev_nr.channel].channel_nr );
        }
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SETDEVICENR:
        if( prec->dev_nr.channel < CAN_CHANNEL_MAX )
        {
          pcan_device.can[prec->dev_nr.channel].channel_nr = prec->dev_nr.serial_num;
        }
        break;
      case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
        if( prec->dev_nr.channel < CAN_CHANNEL_MAX )
        {
          pcan_device.can[prec->dev_nr.channel].led_is_busy = prec->set_can_led.mode;
          pcan_led_set_mode( prec->dev_nr.channel, prec->set_can_led.mode, prec->set_can_led.timeout );
        }
        break;
    }


    rec_ptr += rec_size;
    size -= rec_size;
  }
}

void pcan_protocol_init( void )
{
  pcan_can_init_ex( CAN_BUS_1, 500000 );
  pcan_can_set_filter_mask( CAN_BUS_1, 0, 0, 0, 0 );
  pcan_can_init_ex( CAN_BUS_2, 500000 );
  pcan_can_set_filter_mask( CAN_BUS_2, 0, 0, 0, 0 );

  pcan_usbpro_msg_init_empty( &resp[PCAN_USB_BUFFER_CMD], &resp_buffer[PCAN_USB_BUFFER_CMD], sizeof( resp_buffer[PCAN_USB_BUFFER_CMD] ) );
  pcan_usbpro_msg_init_empty( &resp[PCAN_USB_BUFFER_DATA], &resp_buffer[PCAN_USB_BUFFER_DATA], sizeof( resp_buffer[PCAN_USB_BUFFER_DATA] ) );
  
  pcan_can_install_rx_callback( CAN_BUS_1, pcan_protocol_rx_frame );
  pcan_can_install_rx_callback( CAN_BUS_2, pcan_protocol_rx_frame );

  pcan_can_install_tx_callback( CAN_BUS_1, pcan_protocol_tx_frame_cb );
  pcan_can_install_tx_callback( CAN_BUS_2, pcan_protocol_tx_frame_cb );
}

void pcan_protocol_poll( void )
{
  uint32_t ts_ms = pcan_timestamp_millis();

  pcan_can_poll();

  for( int i = 0; i < 2; i++ )
  {
    struct pcan_usbpro_msg *prec = &resp[i];
    if( prec->rec_buffer_len > 4 )
    {
      int res = pcan_flush_data( &resp_fsm[i], prec->u.rec_buffer, prec->rec_buffer_len );
      if( res )
      {
        pcan_usbpro_msg_reset( prec );
      }
    }
  }

  if( pcan_device.time_calibration_mode )
  {
    if( ( ts_ms - pcan_device.last_time_sync ) >= 1000u )
    {
      pcan_device.last_time_sync = ts_ms;
      pcan_usbpro_msg_add_rec( &resp[PCAN_USB_BUFFER_DATA], DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX,
                                 pcan_usb_frame_number(), pcan_timestamp_us() );
    }
  }
}
