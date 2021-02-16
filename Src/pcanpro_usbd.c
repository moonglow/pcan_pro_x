#include <assert.h>
#include "usbd_ctlreq.h"
#include "usbd_ioreq.h"
#include "usbd_conf.h"
#include "usbd_helper.h"
#include "pcanpro_protocol.h"
#include "pcanpro_usbd.h"

static struct t_class_data pcanpro_data = { 0 };

struct t_pcanpro_description
{
  USB_CONFIGURATION_DESCRIPTOR con0;
  USB_INTERFACE_DESCRIPTOR     if0;
  USB_ENDPOINT_DESCRIPTOR      ep1_i0;
  USB_ENDPOINT_DESCRIPTOR      ep2_i0;
  USB_ENDPOINT_DESCRIPTOR      ep3_i0;
  USB_ENDPOINT_DESCRIPTOR      ep4_i0;
#if ( PCAN_PRO ) || ( PCAN_PRO_FD ) || ( PCAN_X6)
  USB_ENDPOINT_DESCRIPTOR      ep5_i0;
  USB_ENDPOINT_DESCRIPTOR      ep6_i0;
#endif
#if INCLUDE_LIN_INTERFACE
  USB_INTERFACE_DESCRIPTOR     if1;
  USB_ENDPOINT_DESCRIPTOR      ep1_i1;
  USB_ENDPOINT_DESCRIPTOR      ep2_i1;
  USB_ENDPOINT_DESCRIPTOR      ep3_i1;
  USB_ENDPOINT_DESCRIPTOR      ep4_i1;
  USB_ENDPOINT_DESCRIPTOR      ep5_i1;
  USB_ENDPOINT_DESCRIPTOR      ep6_i1;
#endif
};

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN static const USB_DEVICE_QUALIFIER_DESCRIPTOR dev_qua __ALIGN_END = 
{
  .bLength            = sizeof( USB_DEVICE_QUALIFIER_DESCRIPTOR ),
  .bDescriptorType    = USB_QUALIFIER_DESCRIPTOR_TYPE,
  .bcdUSB             = 0x0200, /* 2.0 */
  .bDeviceClass       = 0,
  .bDeviceSubClass    = 0,
  .bDeviceProtocol    = 0,
  .bMaxPacketSize0    = 64,
  .bNumConfigurations = 1,
  .bReserved          = 0,
};


#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN  static struct t_pcanpro_description pcanpro_dev __ALIGN_END = 
{
  .con0 =
  {
    .bLength              = sizeof( USB_CONFIGURATION_DESCRIPTOR ),
    .bDescriptorType      = USB_CONFIGURATION_DESCRIPTOR_TYPE,
    .wTotalLength         = sizeof( struct t_pcanpro_description ),
#if INCLUDE_LIN_INTERFACE
    .bNumInterfaces       = 2,
#else
    .bNumInterfaces       = 1,
#endif
    .bConfigurationValue  = 1,
    .iConfiguration       = 4,
    .bmAttributes         = USB_CONFIG_BUS_POWERED,
    .MaxPower             = 120, /* = 240mA */
  },
  /* CAN INTERFACE */
  .if0 =
  {
    .bLength              = sizeof( USB_INTERFACE_DESCRIPTOR ),
    .bDescriptorType      = USB_INTERFACE_DESCRIPTOR_TYPE,
    .bInterfaceNumber     = 0,
    .bAlternateSetting    = 0,
#if ( PCAN_PRO ) || ( PCAN_PRO_FD ) || ( PCAN_X6)
    .bNumEndpoints        = 6,
#else
    .bNumEndpoints        = 4,
#endif
    .bInterfaceClass      = 0,
    .bInterfaceSubClass   = 0,
    .bInterfaceProtocol   = 0,
    .iInterface           = 5,
  },
  .ep1_i0 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_CMDIN,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
  .ep2_i0 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_CMDOUT,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
  .ep3_i0 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGIN_CH1,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
  .ep4_i0 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGOUT_CH1,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
#if ( PCAN_PRO ) || ( PCAN_PRO_FD )  || ( PCAN_X6)
  .ep5_i0 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGIN_CH2, /* not used */
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
  .ep6_i0 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = PCAN_USB_EP_MSGOUT_CH2,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
#endif
#if INCLUDE_LIN_INTERFACE
  /* LIN INTERFACE */
  .if1 =
  {
    .bLength              = sizeof( USB_INTERFACE_DESCRIPTOR ),
    .bDescriptorType      = USB_INTERFACE_DESCRIPTOR_TYPE,
    .bInterfaceNumber     = 1,
    .bAlternateSetting    = 0,
    .bNumEndpoints        = 6,
    .bInterfaceClass      = 0,
    .bInterfaceSubClass   = 0,
    .bInterfaceProtocol   = 0,
    .iInterface           = 6,
  },
  .ep1_i1 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x84,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
  .ep2_i1 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x04,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
  .ep3_i1 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x85,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
  .ep4_i1 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x05,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
  .ep5_i1 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x86,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  },
  .ep6_i1 = 
  {
    .bLength              = sizeof( USB_ENDPOINT_DESCRIPTOR ),
    .bDescriptorType      = USB_ENDPOINT_DESCRIPTOR_TYPE,
    .bEndpointAddress     = 0x06,
    .bmAttributes         = USB_ENDPOINT_TYPE_BULK,
    .wMaxPacketSize       = 64,/* FS: 64, HS: 512 */
    .bInterval            = 0,
  }
#endif
};

static uint8_t device_init( USBD_HandleTypeDef *pdev, uint8_t cfgidx )
{
  USB_ENDPOINT_DESCRIPTOR *p_ep = &pcanpro_dev.ep1_i0;
  
  for( int i = 0; i < pcanpro_dev.if0.bNumEndpoints; i++ )
  {
    uint8_t ep_addr = p_ep[i].bEndpointAddress;
    
    if( p_ep[i].bmAttributes == USB_ENDPOINT_TYPE_BULK )
    {
      if( pdev->dev_speed == USBD_SPEED_FULL )
        p_ep[i].wMaxPacketSize = PCAN_FS_MAX_BULK_PACKET_SIZE;
      else if( pdev->dev_speed == USBD_SPEED_HIGH )
        p_ep[i].wMaxPacketSize = PCAN_HS_MAX_BULK_PACKET_SIZE;
      else
        assert( 0 );
    }
    
    USBD_LL_OpenEP( pdev, ep_addr,
                          p_ep[i].bmAttributes,
                          p_ep[i].wMaxPacketSize );
    
    if( ( ep_addr & 0x80 ) != 0 )
      pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 1;
    else
      pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 1;
  }
    
  pdev->pClassData = (void*)&pcanpro_data;


  USBD_LL_PrepareReceive( pdev, PCAN_USB_EP_CMDOUT, pcanpro_data.cmd_ep_buffer, sizeof( pcanpro_data.cmd_ep_buffer ) );
  USBD_LL_PrepareReceive( pdev, PCAN_USB_EP_MSGOUT_CH1, pcanpro_data.data1_ep_buffer, sizeof( pcanpro_data.data1_ep_buffer ) );
  USBD_LL_PrepareReceive( pdev, PCAN_USB_EP_MSGOUT_CH2, pcanpro_data.data2_ep_buffer, sizeof( pcanpro_data.data2_ep_buffer ) );

  return USBD_OK;
}

static uint8_t device_deinit( USBD_HandleTypeDef *pdev, uint8_t cfgidx )
{
  USB_ENDPOINT_DESCRIPTOR const *p_ep = &pcanpro_dev.ep1_i0;
  
  for( int i = 0; i < pcanpro_dev.if0.bNumEndpoints; i++ )
  {
    uint8_t ep_addr = p_ep[i].bEndpointAddress;
    USBD_LL_FlushEP( pdev, ep_addr );
    USBD_LL_CloseEP( pdev, ep_addr );
    if( ( ep_addr & 0x80 ) != 0 )
      pdev->ep_in[ep_addr & EP_ADDR_MSK].is_used = 0;
    else
      pdev->ep_out[ep_addr & EP_ADDR_MSK].is_used = 0;
  }
  
  pdev->pClassData = (void*)0;
#if 0
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*) pdev->pData;
  USB_DevDisconnect( hpcd->Instance );
  HAL_Delay( 250 );
  USB_DevConnect( hpcd->Instance );
#endif
  return USBD_OK;
}

static uint8_t  device_setup( USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req )
{ 
  switch( req->bmRequest & USB_REQ_TYPE_MASK )
  {
    case USB_REQ_TYPE_VENDOR:
      return pcan_protocol_device_setup( pdev, req );
    case USB_REQ_TYPE_CLASS:
      return USBD_OK;
    case USB_REQ_TYPE_STANDARD:
      return USBD_OK;
    default:
      USBD_CtlError( pdev, req );
      return USBD_FAIL;
  }
}

static uint8_t device_ep0_rx_ready( USBD_HandleTypeDef *pdev )
{
  pcan_ep0_receive();
  return USBD_OK;
}

static uint8_t device_data_in( USBD_HandleTypeDef *pdev, uint8_t epnum )
{
  struct t_class_data *p_data = (void*)pdev->pClassData;
  
  if( pdev->pClassData == 0 )
    return USBD_FAIL;
  
/* use ZLP */
#if 1
  PCD_HandleTypeDef *hpcd = pdev->pData;
  uint32_t len = pdev->ep_in[epnum].total_length;
  /* packet is multiple of maxpacket, so tell host what all transfer is done */
  if( len && ( len % hpcd->IN_ep[epnum].maxpacket ) == 0U )
  {
    /* update the packet total length */
    pdev->ep_in[epnum].total_length = 0U;
    /* send ZLP */
    USBD_LL_Transmit( pdev, epnum, NULL, 0U );
  }
  else
  {
    /* tx done, no active transfer */
    p_data->ep_tx_in_use[epnum] = 0;
  }
#else
  pdev->ep_in[epnum].total_length = 0U;
  p_data->ep_tx_in_use[epnum] = 0;
#endif
  return USBD_OK;
  
}

static uint8_t device_data_out( USBD_HandleTypeDef *pdev, uint8_t epnum )
{
  int size;
   
  if( pdev->pClassData == 0 )
    return USBD_FAIL;
  
  size = USBD_LL_GetRxDataSize( pdev, epnum );

  if( epnum == PCAN_USB_EP_CMDOUT )
  {
    pcan_protocol_process_data( epnum, pcanpro_data.cmd_ep_buffer, size );
    USBD_LL_PrepareReceive( pdev, epnum, pcanpro_data.cmd_ep_buffer, sizeof( pcanpro_data.cmd_ep_buffer ) );
  }
  else if( epnum == PCAN_USB_EP_MSGOUT_CH1 )
  {
    pcan_protocol_process_data( epnum, pcanpro_data.data1_ep_buffer, size );
    USBD_LL_PrepareReceive( pdev, epnum, pcanpro_data.data1_ep_buffer, sizeof( pcanpro_data.data1_ep_buffer ) );
  }
  else if( epnum == PCAN_USB_EP_MSGOUT_CH2 )
  {
    pcan_protocol_process_data( epnum, pcanpro_data.data2_ep_buffer, size );
    USBD_LL_PrepareReceive( pdev, epnum, pcanpro_data.data2_ep_buffer, sizeof( pcanpro_data.data2_ep_buffer ) );
  }
  else
  {
    return USBD_FAIL;
  }

  return USBD_OK;
}

static uint8_t *device_get_hs_cfg( uint16_t *length )
{
  *length = sizeof( struct t_pcanpro_description );
  return (void*)&pcanpro_dev;
}

static uint8_t *device_get_fs_cfg( uint16_t *length )
{
  *length = sizeof( struct t_pcanpro_description );
  return (void*)&pcanpro_dev;
}

static uint8_t *device_get_other_speed_cfg( uint16_t *length )
{
  *length = sizeof( struct t_pcanpro_description );
  return (void*)&pcanpro_dev;
}

uint8_t *device_get_device_qualifier( uint16_t *length )
{
  *length = sizeof( USB_DEVICE_QUALIFIER_DESCRIPTOR );
  
  return (void*)&dev_qua;
}

static uint8_t sof_handler( struct _USBD_HandleTypeDef *pdev )
{
  uint32_t USBx_BASE = (uint32_t)(((PCD_HandleTypeDef *)pdev->pData)->Instance);
  (void)USBx_BASE;
  
  //vn_data_handler_sof( (USBx_DEVICE->DSTS>>8u)&0x3FFFu );
  
  return USBD_OK;
}

static uint8_t *device_get_user_string( USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length )
{
  __ALIGN_BEGIN static uint8_t USBD_StrDesc[64] __ALIGN_END;

  UNUSED( pdev );

  switch( index )
  {
    case 6:
#if PCAN_PRO_FD
      USBD_GetString((uint8_t *)"PCAN-USB Pro FD LIN", USBD_StrDesc, length );
      break;
#elif PCAN_PRO
      USBD_GetString((uint8_t *)"PCAN-USB-PRO LIN Device", USBD_StrDesc, length );
      break;
#else
      /* fallthrough */
#endif
    default:
      USBD_GetString((uint8_t *)"UNHANDLED", USBD_StrDesc, length );
      break;
  }
  return USBD_StrDesc;
}

USBD_ClassTypeDef usbd_pcanpro =
{
  .Init = device_init,
  .DeInit = device_deinit,
  .Setup = device_setup,
  .EP0_TxSent = 0,
  .EP0_RxReady = device_ep0_rx_ready,
  .DataIn = device_data_in,
  .DataOut = device_data_out,
  .SOF = sof_handler,
  .IsoINIncomplete = 0,
  .IsoOUTIncomplete = 0,
  .GetHSConfigDescriptor = device_get_hs_cfg,
  .GetFSConfigDescriptor = device_get_fs_cfg,
  .GetOtherSpeedConfigDescriptor = device_get_other_speed_cfg,
  .GetDeviceQualifierDescriptor = device_get_device_qualifier,
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  .GetUsrStrDescriptor = device_get_user_string,
#endif
};
