#pragma once
#include <stdint.h>
#include <usbd_ioreq.h>

#define PCAN_USB_BUFFER_CMD  (0)
#define PCAN_USB_BUFFER_DATA (1)

void pcan_protocol_init( void );
void pcan_protocol_poll( void );
void pcan_protocol_process_data( uint8_t ep, uint8_t *ptr, uint16_t size );
void pcan_ep0_receive( void );
uint8_t pcan_protocol_device_setup( USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req );
