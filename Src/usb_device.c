#include <assert.h>
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "pcanpro_usbd.h"

USBD_HandleTypeDef h_usb_device;
extern PCD_HandleTypeDef hpcd_usb;
extern void Error_Handler(void);

#ifndef USB_MODULE_ID
#define USB_MODULE_ID DEVICE_HS
#endif

void pcan_usb_device_init(void)
{
  /* Init Device Library, add supported class and start the library. */
  if( USBD_Init( &h_usb_device, &usbd_desc, USB_MODULE_ID ) != USBD_OK)
  {
    Error_Handler();
  }
  if( USBD_RegisterClass( &h_usb_device, &usbd_pcanpro ) != USBD_OK)
  {
    Error_Handler();
  }
  
  if( USBD_Stop( &h_usb_device ) != USBD_OK )
  {
    Error_Handler();
  }
  
  HAL_Delay( 1000 );
  
  if( USBD_Start( &h_usb_device ) != USBD_OK )
  {
    Error_Handler();
  }
}

void pcan_usb_device_poll( void )
{
  HAL_PCD_IRQHandler( &hpcd_usb );
}

uint16_t pcan_usb_frame_number( void )
{
  uint32_t USBx_BASE = (uint32_t)(((PCD_HandleTypeDef *)h_usb_device.pData)->Instance);
  
  return (USBx_DEVICE->DSTS>>8u)&0x3FFFu;
}

int pcan_flush_ep( uint8_t ep )
{
  USBD_HandleTypeDef *pdev = &h_usb_device;
  struct t_class_data *p_data = (void*)pdev->pClassData;

  p_data->ep_tx_in_use[ep&0x0F] = 0;
  return USBD_LL_FlushEP( pdev, ep ) == USBD_OK;
}

int pcan_flush_data( struct t_m2h_fsm *pfsm, void *src, int size )
{
  USBD_HandleTypeDef *pdev = &h_usb_device;
  struct t_class_data *p_data = (void*)pdev->pClassData;

  if( !p_data )
    return 0;

  switch( pfsm->state )
  {
    case 1:
      if( p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] )
        return 0;
      pfsm->state = 0;
      /* fall through */
    case 0:
      assert( p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] == 0 );
      //size = (size+(64-1))&(~(64-1));
      if( size > pfsm->dbsize )
        break;
      memcpy( pfsm->pdbuf, src, size );
      p_data->ep_tx_in_use[pfsm->ep_addr&0x0F] = 1;
      /* prepare data transmit */
      pdev->ep_in[pfsm->ep_addr & EP_ADDR_MSK].total_length = size;
      USBD_LL_Transmit( pdev, pfsm->ep_addr, pfsm->pdbuf, size );
      
      pfsm->total_tx += size;
      pfsm->state = 1;
      return 1;
  }
  
  return 0;
}
