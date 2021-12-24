#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"


#define USBD_VID                        0x0c72
#define USBD_LANGID_STRING              1033
#define USBD_MANUFACTURER_STRING        "PEAK-System Technik GmbH"
#if  PCAN_PRO_FD
#define USBD_PID_HS                     0x0011
#define USBD_PRODUCT_STRING_HS          "PCAN-USB Pro FD"
#define USBD_INTERFACE_CAN_STRING_HS    "PCAN-USB Pro FD CAN"
#define USBD_INTERFACE_LIN_STRING_HS    "PCAN-USB Pro FD LIN"
#elif PCAN_FD
#define USBD_PID_HS                     0x0012
#define USBD_PRODUCT_STRING_HS          "PCAN-USB FD"
#define USBD_INTERFACE_CAN_STRING_HS    "PCAN-USB FD CAN"
#define USBD_INTERFACE_LIN_STRING_HS    "PCAN-USB FD LIN"
#elif PCAN_PRO
#define USBD_PID_HS                     0x000d
#define USBD_PRODUCT_STRING_HS          "PCAN-USB-PRO"
#define USBD_INTERFACE_CAN_STRING_HS    "PCAN-USB-PRO CAN Device"
#define USBD_INTERFACE_LIN_STRING_HS    "PCAN-USB-PRO LIN Device"
#elif PCAN_X6
#define USBD_PID_HS                     0x0014
#define USBD_PRODUCT_STRING_HS          "PCAN-USB X6"
#define USBD_INTERFACE_CAN_STRING_HS    "PCAN-USB X6 CAN Device"
#define USBD_INTERFACE_LIN_STRING_HS    "PCAN-USB X6 LIN Device"
#else
#error Oops
#endif
#define USBD_CONFIGURATION_STRING_HS    "Config00"


#define USB_SIZ_BOS_DESC                0x0C

#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
/** USB standard device descriptor. */
__ALIGN_BEGIN uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
#if (USBD_LPM_ENABLED == 1)
  0x01,                       /*bcdUSB */ /* changed to USB version 2.01
                                             in order to support LPM L1 suspend
                                             resume test of USBCV3.0*/
#else
  0x00,                       /*bcdUSB */
#endif
  0x02,
  0,                          /*bDeviceClass*/
  0,                          /*bDeviceSubClass*/
  0,                          /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_HS),        /*idProduct*/
  HIBYTE(USBD_PID_HS),        /*idProduct*/
  0x00,                       /*bcdDevice rel. 0.00*/
  0x00,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  0 /*USBD_IDX_SERIAL_STR*/,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

/** BOS descriptor. */
#if (USBD_LPM_ENABLED == 1)
#if defined ( __ICCARM__ )
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t USBD_BOSDesc[USB_SIZ_BOS_DESC] __ALIGN_END =
{
  0x5,
  USB_DESC_TYPE_BOS,
  0xC,
  0x0,
  0x1,  /* 1 device capability */
        /* device capability */
  0x7,
  USB_DEVICE_CAPABITY_TYPE,
  0x2,
  0x2,  /*LPM capability bit set */
  0x0,
  0x0,
  0x0
};
#endif

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */

/** USB lang indentifier descriptor. */
__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END =
{
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif /* defined ( __ICCARM__ ) */
/* Internal string descriptor. */
__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;


uint8_t * USBD_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

uint8_t * USBD_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

uint8_t * USBD_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == 0)
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_HS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_HS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

uint8_t * USBD_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t * USBD_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if(speed == USBD_SPEED_HIGH)
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_HS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_HS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

uint8_t * USBD_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  if( speed == 0 )
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_CAN_STRING_HS, USBD_StrDesc, length);
  }
  else
  {
    USBD_GetString((uint8_t *)USBD_INTERFACE_CAN_STRING_HS, USBD_StrDesc, length);
  }
  return USBD_StrDesc;
}

#if (USBD_CLASS_USER_STRING_DESC == 1)
static uint8_t *USBD_UserStrDescriptor( USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length )
{
  __ALIGN_BEGIN static uint8_t USBD_StrDesc[64] __ALIGN_END;

  UNUSED( pdev );

  switch( index )
  {
    case 6:
      USBD_GetString((uint8_t *)USBD_INTERFACE_LIN_STRING_HS, USBD_StrDesc, length );
      break;
    default:
      USBD_GetString((uint8_t *)"UNHANDLED", USBD_StrDesc, length );
      break;
  }
  return USBD_StrDesc;
}
#endif

#if (USBD_LPM_ENABLED == 1)
static uint8_t * USBD_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_BOSDesc);
  return (uint8_t*)USBD_BOSDesc;
}
#endif

USBD_DescriptorsTypeDef usbd_desc =
{
  .GetDeviceDescriptor = USBD_DeviceDescriptor,
  .GetLangIDStrDescriptor = USBD_LangIDStrDescriptor,
  .GetManufacturerStrDescriptor = USBD_ManufacturerStrDescriptor,
  .GetProductStrDescriptor = USBD_ProductStrDescriptor,
  .GetSerialStrDescriptor = 0,
  .GetConfigurationStrDescriptor = USBD_ConfigStrDescriptor,
  .GetInterfaceStrDescriptor = USBD_InterfaceStrDescriptor,
#if (USBD_CLASS_USER_STRING_DESC == 1)
  .GetUserStrDescriptor = USBD_UserStrDescriptor,
#endif
#if (USBD_LPM_ENABLED == 1)
  USBD_USR_BOSDescriptor,
#endif
};
