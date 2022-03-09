#pragma once
#include <stdint.h>

enum 
{
  CAN_BUS_1,
  CAN_BUS_2,
  CAN_BUS_TOTAL
};

#define CAN_PAYLOAD_MAX_SIZE (8u)
struct t_can_msg
{
  uint32_t timestamp;
  uint32_t id;
  uint8_t  flags;
  uint8_t  size;
  uint8_t  dummy;
  uint8_t  data[CAN_PAYLOAD_MAX_SIZE];
};

struct t_can_stats
{
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;
};

struct t_can_bitrate
{
  uint16_t brp;
  uint16_t tseg1;
  uint8_t tseg2;
  uint8_t sjw;
};

#define MSG_FLAG_STD  (0<<0)
#define MSG_FLAG_EXT  (1<<0)
#define MSG_FLAG_RTR  (1<<1)
#define MSG_FLAG_ECHO (1<<3)
/* FDCAN */
#define MSG_FLAG_BRS   (1<<4)
#define MSG_FLAG_FD    (1<<5)
#define MSG_FLAG_ESI   (1<<6)

#define CAN_INT_FILTER_MAX (14)
#define CAN_EXT_FILTER_MAX (2)

void pcan_can_poll( void );
uint32_t pcan_can_msg_time( const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt );
int pcan_can_stats( int bus, struct t_can_stats *p_stats );
int pcan_can_init_ex( int bus, uint32_t bitrate );
void pcan_can_set_silent( int bus, uint8_t silent_mode );
void pcan_can_set_iso_mode( int bus, uint8_t iso_mode );
void pcan_can_set_loopback( int bus, uint8_t loopback );
void pcan_can_set_bus_active( int bus, uint16_t mode );
void pcan_can_set_bitrate( int bus, uint32_t bitrate, int is_data_bitrate );
void pcan_can_set_bitrate_ex( int bus, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw );
int pcan_can_write( int bus, struct t_can_msg *p_msg );
void pcan_can_install_rx_callback( int bus, int (*rx_isr)( uint8_t, struct  t_can_msg* ));
void pcan_can_install_tx_callback( int bus, int (*rx_isr)( uint8_t, struct  t_can_msg* ));
void pcan_can_install_err_callback( int bus, void (*cb)( int, uint32_t ) );

int pcan_can_set_filter_mask( int bus, int num, int format, uint32_t id, uint32_t mask );
int pcan_can_filter_init_stdid_list( int bus, const uint16_t *id_list, int id_len  );
