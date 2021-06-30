#include <assert.h>
#include "io_macro.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"

#define CAN1_RX  B, 8, MODE_AF_PP, NOPULL, SPEED_FREQ_VERY_HIGH, AF9_CAN1
#define CAN1_TX  B, 9, MODE_AF_PP, NOPULL, SPEED_FREQ_VERY_HIGH, AF9_CAN1

#define CAN2_RX  B, 5, MODE_AF_PP, NOPULL, SPEED_FREQ_VERY_HIGH, AF9_CAN2
#define CAN2_TX  B, 6, MODE_AF_PP, NOPULL, SPEED_FREQ_VERY_HIGH, AF9_CAN2

static CAN_HandleTypeDef hcan[CAN_BUS_TOTAL] = 
{ 
  [CAN_BUS_1].Instance = CAN1,
  [CAN_BUS_2].Instance = CAN2
};

#define CAN_TX_FIFO_SIZE (256)
static struct t_can_dev
{
  void *dev;
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  struct t_can_msg tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;
  uint32_t esr_reg;
  int (*rx_isr)( uint8_t, struct  t_can_msg* );
  int (*tx_isr)( uint8_t, struct  t_can_msg* );
  void (*err_handler)( int bus, uint32_t esr );
}
can_dev_array[CAN_BUS_TOTAL] = 
{       
  [CAN_BUS_1] = { .dev = &hcan[CAN_BUS_1] },
  [CAN_BUS_2] = { .dev = &hcan[CAN_BUS_2] },
};

#define INTERNAL_CAN_IT_FLAGS          (  CAN_IT_TX_MAILBOX_EMPTY |\
                                          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |\
                                          CAN_IT_ERROR_WARNING |\
                                          CAN_IT_ERROR_PASSIVE |\
                                          CAN_IT_LAST_ERROR_CODE |\
                                          CAN_IT_ERROR )

#define CAN2_FILTER_START (14u)

#define CAN_WITHOUT_ISR 1

uint32_t pcan_can_msg_time( const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt )
{
  const uint32_t data_bits = pmsg->size<<3;
  const uint32_t control_bits = ( pmsg->flags & MSG_FLAG_EXT ) ? 67:47;
 
  if( pmsg->flags & MSG_FLAG_BRS )
    return (control_bits*nt) + (data_bits*dt);
  else
    return (control_bits+data_bits)*nt;
}

int pcan_can_set_filter_mask( int bus, int num, int format, uint32_t id, uint32_t mask )
{
  CAN_FilterTypeDef filter = { 0 };
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;
  
  if( num >= CAN_INT_FILTER_MAX )
    return -1;
  
  /* CAN1 & CAN2 filter shared 28 filters, we use 14 for each one */
  if( p_can->Instance == CAN2 )
  {
    num += CAN2_FILTER_START;
  }
  
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  if( format == MSG_FLAG_EXT )
  {
    id &= 0x1FFFFFFF;
    
    /* EXTID[28:13] */
    filter.FilterIdHigh = id >> 13;
    /* EXTID[12:0] + IDE */
    filter.FilterIdLow = ((id << 3)&0xFFFF) | CAN_ID_EXT;
    filter.FilterMaskIdHigh = mask >> 13;
    filter.FilterMaskIdLow = ((mask << 3)&0xFFFF) | CAN_ID_EXT;
  }
  else
  {
    id &= 0x7FF;
    filter.FilterIdHigh = id << 5;
    filter.FilterIdLow =  0x0;
    filter.FilterMaskIdHigh = mask << 5;
    filter.FilterMaskIdLow = 0x0;
  }
  
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.FilterBank = num;
  filter.SlaveStartFilterBank = CAN2_FILTER_START;
  
  if( HAL_CAN_ConfigFilter( p_can, &filter ) != HAL_OK )
    return -1;
  
  return 0;
}

int pcan_can_filter_init_stdid_list( int bus, const uint16_t *id_list, int id_len  )
{
  CAN_FilterTypeDef filter = { 0 };
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;
 
  int i, offset;

  offset = ( p_can->Instance == CAN2 ) ? CAN2_FILTER_START: 0;  
  
  /* STDID[10:3] | STDID[2:0] RTR IDE EXID[17:15] */
  /* IDE = 0, RTR = 0, EXID[17:15] = { 0 } */
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  
  i = 0;
  while( i < id_len )
  {
    filter.FilterBank = ((i)>>2)+offset;
    filter.SlaveStartFilterBank = CAN2_FILTER_START;
    
    filter.FilterMaskIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterIdHigh = 0;
    
    switch( (id_len-i) )
    {
    /* fallthrough */
    default:
      filter.FilterMaskIdLow = id_list[i++]<<5;
    case 3:
      filter.FilterMaskIdHigh = id_list[i++]<<5;
    case 2:
      filter.FilterIdLow = id_list[i++]<<5;
    case 1:
      filter.FilterIdHigh = id_list[i++]<<5;
    }
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    
    if( HAL_CAN_ConfigFilter( p_can, &filter ) != HAL_OK )
      return -1;
  }
  
  return id_len;
}

static int _can_send( CAN_HandleTypeDef *p_can, struct t_can_msg *p_msg )
{
  CAN_TxHeaderTypeDef msg = { .TransmitGlobalTime = DISABLE };
  uint32_t txMailbox = 0;

  if( HAL_CAN_IsTxMessagePending( p_can, CAN_TX_MAILBOX0 ) )
    return -1;

  if( p_msg->flags & MSG_FLAG_EXT )
  {
    msg.ExtId = p_msg->id & 0x1FFFFFFF;
    msg.IDE = CAN_ID_EXT;
  }
  else
  {
    msg.StdId = p_msg->id & 0x7FF;
    msg.IDE = CAN_ID_STD;
  }
  
  msg.DLC = p_msg->size;
  msg.RTR = (p_msg->flags & MSG_FLAG_RTR)?CAN_RTR_REMOTE:CAN_RTR_DATA;
  
  if( HAL_CAN_AddTxMessage( p_can, &msg, (void*)p_msg->data, &txMailbox ) != HAL_OK )
    return -1;

  return txMailbox;
}

static void pcan_can_flush_tx( int bus )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  struct t_can_msg *p_msg;

  /* empty fifo */
  if( p_dev->tx_head == p_dev->tx_tail )
    return;
  
  p_msg = &p_dev->tx_fifo[p_dev->tx_tail];
  if( _can_send( p_dev->dev, p_msg ) < 0 )
    return;

  if( p_dev->tx_isr )
  {
    (void)p_dev->tx_isr( bus, p_msg );
  }

  /* update fifo index */
  p_dev->tx_tail = (p_dev->tx_tail+1)&(CAN_TX_FIFO_SIZE-1);
}

int pcan_can_write( int bus, struct t_can_msg *p_msg )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];

  if( !p_msg )
    return 0;

  uint32_t  tx_head_next = (p_dev->tx_head+1)&(CAN_TX_FIFO_SIZE-1);
  /* overflow ? just skip it */
  if( tx_head_next == p_dev->tx_tail )
  {
    ++p_dev->tx_ovfs;
    return -1;
  }

  p_dev->tx_fifo[p_dev->tx_head] = *p_msg;
  p_dev->tx_head = tx_head_next;

  return 0;
}

void pcan_can_install_rx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->rx_isr = cb;
}

void pcan_can_install_tx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->tx_isr = cb;
}

void pcan_can_install_err_callback( int bus, void (*cb)( int , uint32_t ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->err_handler = cb;
}

/* all internal CANs on APB1 */
static int _get_precalculated_bitrate( uint32_t bitrate, uint32_t *brp, uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw )
{
  *sjw = CAN_SJW_1TQ;

  switch( bitrate )
  {
    case 1000000u:
      *brp = 3;
      *tseg1 = CAN_BS1_6TQ;
      *tseg2 = CAN_BS2_1TQ;
    break;
    case 800000u:
      *brp = 2;
      *tseg1 = CAN_BS1_12TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    default:
    case 500000u:
      *brp = 3;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 250000u:
      *brp = 6;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 125000u:
      *brp = 12;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 100000u:
      *brp = 15;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 50000u:
      *brp = 30;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 20000u:
      *brp = 75;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 10000u:
      *brp = 150;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
  }

  return 0;
}

int pcan_can_init_ex( int bus, uint32_t bitrate )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;
  uint32_t brp;
  uint32_t tseg1, tseg2, sjw;

  p_can->Init.Mode = CAN_MODE_NORMAL;//CAN_MODE_NORMAL;// CAN_MODE_LOOPBACK;
    
  p_can->Init.TimeTriggeredMode = DISABLE;
  p_can->Init.AutoBusOff = ENABLE;
  p_can->Init.AutoWakeUp = ENABLE;

  p_can->Init.AutoRetransmission = DISABLE;
  p_can->Init.ReceiveFifoLocked = DISABLE;
  p_can->Init.TransmitFifoPriority = ENABLE;
  
  /* APB1 bus ref clock = 24MHz, best sp is 87.5% */
  _get_precalculated_bitrate( bitrate, &brp, &tseg1, &tseg2, &sjw );

  p_can->Init.SyncJumpWidth = sjw;
  p_can->Init.Prescaler = brp;
  p_can->Init.TimeSeg1 = tseg1;
  p_can->Init.TimeSeg2 = tseg2;
  
  //(void)HAL_CAN_AbortTxRequest( p_can, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
  //(void)HAL_CAN_Stop( p_can );
  //(void)HAL_CAN_DeInit( p_can );
  
  if( HAL_CAN_Init( p_can ) != HAL_OK )
  {
    assert(0);
  }
  
  if( HAL_CAN_ActivateNotification( p_can, INTERNAL_CAN_IT_FLAGS ) != HAL_OK )
  {
    assert(0);
  }
  
  if( HAL_CAN_Start( p_can ) != HAL_OK )
  {
    assert(0);
  }
  
  return 0;
}

void pcan_can_set_silent( int bus, uint8_t silent_mode )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  p_can->Init.Mode = silent_mode ? CAN_MODE_SILENT: CAN_MODE_NORMAL;
  if( HAL_CAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

/* bxCAN does not support FDCAN ISO mode switch */
void pcan_can_set_iso_mode( int bus, uint8_t iso_mode )
{
  (void)bus;
  (void)iso_mode;
}

void pcan_can_set_loopback( int bus, uint8_t loopback )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;
  
  p_can->Init.Mode = loopback ? CAN_MODE_LOOPBACK: CAN_MODE_NORMAL;
  if( HAL_CAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_set_bus_active( int bus, uint16_t mode )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( mode )
  {
    HAL_CAN_Start( p_can );
    HAL_CAN_AbortTxRequest( p_can, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
  }
  else
  {
    HAL_CAN_AbortTxRequest( p_can, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
    HAL_CAN_Stop( p_can );
  }
}

/* set predefined best values */
void pcan_can_set_bitrate( int bus, uint32_t bitrate, int is_data_bitrate )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;
  uint32_t brp;
  uint32_t tseg1, tseg2, sjw;

  _get_precalculated_bitrate( bitrate, &brp, &tseg1, &tseg2, &sjw );

  if( is_data_bitrate )
  {
    return;
  }
  else
  {
    p_can->Init.Prescaler = brp;
    p_can->Init.SyncJumpWidth = sjw;
    p_can->Init.TimeSeg1 = tseg1;
    p_can->Init.TimeSeg2 = tseg2;
  }
  
  if( HAL_CAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_set_bitrate_ex( int bus, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw )
{
  static const uint32_t sjw_table[] = 
  { 
    CAN_SJW_1TQ, CAN_SJW_2TQ, CAN_SJW_3TQ, CAN_SJW_4TQ 
  };
  static const uint32_t tseg1_table[] = 
  { 
    CAN_BS1_1TQ, CAN_BS1_2TQ, CAN_BS1_3TQ, CAN_BS1_4TQ, 
    CAN_BS1_5TQ, CAN_BS1_6TQ, CAN_BS1_7TQ, CAN_BS1_8TQ, 
    CAN_BS1_9TQ, CAN_BS1_10TQ, CAN_BS1_11TQ, CAN_BS1_12TQ,
    CAN_BS1_13TQ, CAN_BS1_14TQ, CAN_BS1_15TQ, CAN_BS1_16TQ
  };
  static const uint32_t tseg2_table[] =
  { 
    CAN_BS2_1TQ, CAN_BS2_2TQ, CAN_BS2_3TQ, CAN_BS2_4TQ,
    CAN_BS2_5TQ, CAN_BS2_6TQ, CAN_BS2_7TQ, CAN_BS2_8TQ
  };

  if( sjw > 4 )
    sjw = 4;
  if( tseg1 > 16 )
    tseg1 = 16;
  if( tseg2 > 8 )
    tseg2 = 8;

  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  p_can->Init.Prescaler = brp;

  p_can->Init.SyncJumpWidth = sjw_table[sjw - 1];
  p_can->Init.TimeSeg1 = tseg1_table[tseg1 - 1];
  p_can->Init.TimeSeg2 = tseg2_table[tseg2 - 1];

  if( HAL_CAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

static void pcan_can_tx_complete( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_msgs;
}

static void pcan_can_tx_err( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_errs;
}

int pcan_can_stats( int bus, struct t_can_stats *p_stats )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  
  p_stats->tx_msgs = p_dev->tx_msgs;
  p_stats->tx_errs = p_dev->tx_errs;
  p_stats->rx_msgs = p_dev->rx_msgs;
  p_stats->rx_errs = p_dev->rx_errs;
  p_stats->rx_ovfs = p_dev->rx_ovfs;

  return sizeof( struct t_can_stats );
}

void pcan_can_poll( void )
{
  static uint32_t err_last_check = 0;
  uint32_t ts_ms;
  
  ts_ms = pcan_timestamp_millis();
#if ( CAN_WITHOUT_ISR == 1 )
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
#endif
  
  pcan_can_flush_tx( CAN_BUS_1 );
  pcan_can_flush_tx( CAN_BUS_2 );

  if( (uint32_t)( err_last_check - ts_ms ) > 250 )
  {
    err_last_check = ts_ms;
    for( int i = 0; i < CAN_BUS_TOTAL; i++ )
    {
      if( !can_dev_array[i].err_handler )
        continue;
      CAN_HandleTypeDef *pcan = can_dev_array[i].dev;
      if( can_dev_array[i].esr_reg != pcan->Instance->ESR )
      {
        can_dev_array[i].esr_reg = pcan->Instance->ESR;
        can_dev_array[i].err_handler( i, can_dev_array[i].esr_reg );
      }
    }
  }
}

/* --------------- HAL PART ------------- */
static int _bus_from_int_dev( CAN_TypeDef *can )
{
  if( can == CAN1 )
    return CAN_BUS_1;
  else if( can == CAN2 )
    return CAN_BUS_2;
  /* abnormal! */
  return CAN_BUS_1;
}

static void pcan_can_isr_frame( CAN_HandleTypeDef *hcan, uint32_t fifo )
{
  CAN_RxHeaderTypeDef hdr;
  const int bus = _bus_from_int_dev( hcan->Instance );
  struct t_can_dev * const p_dev = &can_dev_array[bus];
  struct t_can_msg  msg = { 0 };
  
  if( HAL_CAN_GetRxMessage( hcan, fifo, &hdr, msg.data ) != HAL_OK )
    return;

  /* oversize frame ? */
  if( hdr.DLC > CAN_PAYLOAD_MAX_SIZE )
    return;

  if( hdr.IDE == CAN_ID_STD )
  {
    msg.id = hdr.StdId;
  }
  else
  {
    msg.id = hdr.ExtId;
    msg.flags |= MSG_FLAG_EXT;
  }

  if( hdr.RTR != CAN_RTR_DATA )
  {
    msg.flags |= MSG_FLAG_RTR;
  }

  msg.size = hdr.DLC;
  msg.timestamp = pcan_timestamp_us();
  
  if( p_dev->rx_isr )
  {
    if( p_dev->rx_isr( bus, &msg ) < 0 )
    {
      ++p_dev->rx_ovfs;
      return;
    }
  }
  ++p_dev->rx_msgs;
}

/* WARN: CAN1 & CAN2 use filter registers of CAN1 */
void HAL_CAN_MspInit( CAN_HandleTypeDef *hcan )
{
  if( hcan->Instance == CAN1 )
  {
    if( !__HAL_RCC_CAN1_IS_CLK_ENABLED() )
    {
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    PORT_ENABLE_CLOCK( PIN_PORT( CAN1_RX ), PIN_PORT( CAN1_TX ) );

    PIN_INIT( CAN1_RX );
    PIN_INIT( CAN1_TX );
    
#if ( CAN_WITHOUT_ISR == 0 ) 
    HAL_NVIC_SetPriority( CAN1_TX_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_TX_IRQn );
    
    HAL_NVIC_SetPriority( CAN1_RX0_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_RX0_IRQn );
    
    HAL_NVIC_SetPriority( CAN1_RX1_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_RX1_IRQn );

    HAL_NVIC_SetPriority( CAN1_SCE_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_SCE_IRQn );
#endif
  }
  else if( hcan->Instance == CAN2 )
  {
    if( !__HAL_RCC_CAN1_IS_CLK_ENABLED() )
    {
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
    if( !__HAL_RCC_CAN2_IS_CLK_ENABLED() )
    {
      __HAL_RCC_CAN2_CLK_ENABLE();
    }

    PORT_ENABLE_CLOCK( PIN_PORT( CAN2_RX ), PIN_PORT( CAN2_TX ) );

    PIN_INIT( CAN2_RX );
    PIN_INIT( CAN2_TX );
    
#if ( CAN_WITHOUT_ISR == 0 ) 
    HAL_NVIC_SetPriority( CAN2_TX_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN2_TX_IRQn );
    
    HAL_NVIC_SetPriority( CAN2_RX0_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN2_RX0_IRQn );
    
    HAL_NVIC_SetPriority( CAN2_RX1_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN2_RX1_IRQn );
    
    HAL_NVIC_SetPriority( CAN2_SCE_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN2_SCE_IRQn );
#endif
  }
}

void HAL_CAN_MspDeInit( CAN_HandleTypeDef *hcan )
{
  if( hcan->Instance == CAN1 )
  {
    /* if CAN2 not used anymore */
    if( !__HAL_RCC_CAN2_IS_CLK_ENABLED() )
    {
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    PIN_DEINIT( CAN1_RX );
    PIN_DEINIT( CAN1_TX );
    
    HAL_NVIC_DisableIRQ( CAN1_TX_IRQn );
    HAL_NVIC_DisableIRQ( CAN1_RX0_IRQn );
    HAL_NVIC_DisableIRQ( CAN1_RX1_IRQn );
    HAL_NVIC_DisableIRQ( CAN1_SCE_IRQn );
  }
  else if( hcan->Instance == CAN2 )
  {
    __HAL_RCC_CAN2_CLK_DISABLE();
  
    PIN_DEINIT( CAN2_RX );
    PIN_DEINIT( CAN2_TX );
    
    HAL_NVIC_DisableIRQ( CAN2_TX_IRQn );
    HAL_NVIC_DisableIRQ( CAN2_RX0_IRQn );
    HAL_NVIC_DisableIRQ( CAN2_RX1_IRQn );
    HAL_NVIC_DisableIRQ( CAN2_SCE_IRQn );
  }
}


/* CAN HAL subsystem callbacks */
void HAL_CAN_TxMailbox0CompleteCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_tx_complete( _bus_from_int_dev( hcan->Instance ), 0 );
}

void HAL_CAN_TxMailbox1CompleteCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_tx_complete( _bus_from_int_dev( hcan->Instance ), 1 ); 
}

void HAL_CAN_TxMailbox2CompleteCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_tx_complete( _bus_from_int_dev( hcan->Instance ), 2 );
}

void HAL_CAN_TxMailbox0AbortCallback( CAN_HandleTypeDef *hcan ){}
void HAL_CAN_TxMailbox1AbortCallback( CAN_HandleTypeDef *hcan ){}
void HAL_CAN_TxMailbox2AbortCallback( CAN_HandleTypeDef *hcan ){}

void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_isr_frame( hcan, CAN_RX_FIFO0 );
}

void HAL_CAN_RxFifo1MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_isr_frame( hcan, CAN_RX_FIFO1 );
}

void HAL_CAN_RxFifo0FullCallback( CAN_HandleTypeDef *hcan )
{
}

void HAL_CAN_RxFifo1FullCallback( CAN_HandleTypeDef *hcan )
{
}

void HAL_CAN_SleepCallback( CAN_HandleTypeDef *hcan ){}
void HAL_CAN_WakeUpFromRxMsgCallback( CAN_HandleTypeDef *hcan ){}

void HAL_CAN_ErrorCallback( CAN_HandleTypeDef *hcan )
{
  /* handle errors */
  uint32_t err = HAL_CAN_GetError( hcan );
  int bus = _bus_from_int_dev( hcan->Instance );
  
  if ( err & HAL_CAN_ERROR_TX_TERR0 ) 
  {
    pcan_can_tx_err( bus, 0 );
  }
  
  if ( err & HAL_CAN_ERROR_TX_TERR1 ) 
  {
    pcan_can_tx_err( bus, 1 );
  }
  
  if ( err & HAL_CAN_ERROR_TX_TERR2 ) 
  {
    pcan_can_tx_err( bus, 2 );
  }
  
  HAL_CAN_ResetError( hcan );
}


/* ISR handlers */
#if ( CAN_WITHOUT_ISR == 0 ) 
/* CAN1 */
void CAN1_TX_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
}

void CAN1_RX0_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
}

void CAN1_RX1_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
}

void CAN1_SCE_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
}
/* CAN2 */
void CAN2_TX_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
}

void CAN2_RX0_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
}

void CAN2_RX1_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
}

void CAN2_SCE_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
}
#endif