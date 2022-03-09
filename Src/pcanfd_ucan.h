/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CAN driver for PEAK System micro-CAN based adapters
 *
 * Copyright (C) 2001-2020 PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Contact:      <linux@peak-system.com>
 * Author:       Stephane Grosjean <s.grosjean@peak-system.com>
 */
#ifndef UCAN_H
#define UCAN_H
#include <stdint.h>

/* uCAN commands opcodes list (low-order 10 bits) */
#define UCAN_CMD_NOP      0x000
#define UCAN_CMD_RESET_MODE    0x001
#define UCAN_CMD_NORMAL_MODE    0x002
#define UCAN_CMD_LISTEN_ONLY_MODE  0x003
#define UCAN_CMD_TIMING_SLOW    0x004
#define UCAN_CMD_TIMING_FAST    0x005
#define UCAN_CMD_SET_STD_FILTER    0x006
#define UCAN_CMD_RESERVED2    0x007
#define UCAN_CMD_FILTER_STD    0x008
#define UCAN_CMD_TX_ABORT    0x009
#define UCAN_CMD_WR_ERR_CNT    0x00a
#define UCAN_CMD_SET_EN_OPTION    0x00b
#define UCAN_CMD_CLR_DIS_OPTION    0x00c

#define UCAN_CMD_SET_ERR_GEN1    0x00d
#define UCAN_CMD_SET_ERR_GEN    UCAN_CMD_SET_ERR_GEN1
#define UCAN_CMD_SET_ERR_GEN2    0x00e
#define UCAN_CMD_DIS_ERR_GEN    0x00f
#define UCAN_CMD_RX_BARRIER    0x010
#define UCAN_CMD_SET_ERR_GEN_S    0x011

#define UCAN_CMD_END_OF_COLLECTION  0x3ff

/* uCAN received messages list */
#define UCAN_MSG_CAN_RX      0x0001
#define UCAN_MSG_ERROR      0x0002
#define UCAN_MSG_STATUS      0x0003
#define UCAN_MSG_BUSLOAD    0x0004

/* uCAN transmitted messages */
#define UCAN_MSG_CAN_TX      0x1000

/* uCAN Tx Pause record */
#define UCAN_MSG_CAN_TX_PAUSE    0x1002

/* uCAN command common header */
#define UCAN_CMD_OPCODE(c)    ((c)->opcode_channel & 0x3ff)
#define UCAN_CMD_CHANNEL(c)    ((c)->opcode_channel >> 12)
#define UCAN_CMD_OPCODE_CHANNEL(c, o)  (((c) << 12) | ((o) & 0x3ff))

struct ucan_command {
  uint16_t  opcode_channel;
  uint16_t  args[3];
}__attribute__((packed));

#define UCAN_TSLOW_BRP_BITS    10
#define UCAN_TFAST_BRP_BITS    10

#if 1
/* current version of uCAN IP core */
#define UCAN_TSLOW_TSGEG1_BITS    8
#define UCAN_TSLOW_TSGEG2_BITS    7
#define UCAN_TSLOW_SJW_BITS    7

#define UCAN_TFAST_TSGEG1_BITS    5
#define UCAN_TFAST_TSGEG2_BITS    4
#define UCAN_TFAST_SJW_BITS    4
#else
/* first version of uCAN IP core */
#define UCAN_TSLOW_TSGEG1_BITS    6
#define UCAN_TSLOW_TSGEG2_BITS    4
#define UCAN_TSLOW_SJW_BITS    4

#define UCAN_TFAST_TSGEG1_BITS    4
#define UCAN_TFAST_TSGEG2_BITS    3
#define UCAN_TFAST_SJW_BITS    2
#endif

#define UCAN_TSLOW_BRP_MASK    ((1 << UCAN_TSLOW_BRP_BITS) - 1)
#define UCAN_TSLOW_TSEG1_MASK    ((1 << UCAN_TSLOW_TSGEG1_BITS) - 1)
#define UCAN_TSLOW_TSEG2_MASK    ((1 << UCAN_TSLOW_TSGEG2_BITS) - 1)
#define UCAN_TSLOW_SJW_MASK    ((1 << UCAN_TSLOW_SJW_BITS) - 1)

#define UCAN_TFAST_BRP_MASK    ((1 << UCAN_TFAST_BRP_BITS) - 1)
#define UCAN_TFAST_TSEG1_MASK    ((1 << UCAN_TFAST_TSGEG1_BITS) - 1)
#define UCAN_TFAST_TSEG2_MASK    ((1 << UCAN_TFAST_TSGEG2_BITS) - 1)
#define UCAN_TFAST_SJW_MASK    ((1 << UCAN_TFAST_SJW_BITS) - 1)

/* uCAN TIMING_SLOW command fields */
#define UCAN_TSLOW_SJW_T(s, t)    (((s) & UCAN_TSLOW_SJW_MASK) | \
                ((!!(t)) << 7))
#define UCAN_TSLOW_TSEG2(t)    ((t) & UCAN_TSLOW_TSEG2_MASK)
#define UCAN_TSLOW_TSEG1(t)    ((t) & UCAN_TSLOW_TSEG1_MASK)
#define UCAN_TSLOW_BRP(b)      ((b) & UCAN_TSLOW_BRP_MASK) 

struct ucan_timing_slow {
  uint16_t  opcode_channel;

  uint8_t  ewl;    /* Error Warning limit */
  uint8_t  sjw_t;    /* Sync Jump Width + Triple sampling */
  uint8_t  tseg2;    /* Timing SEGment 2 */
  uint8_t  tseg1;    /* Timing SEGment 1 */

  uint16_t  brp;    /* BaudRate Prescaler */
}__attribute__((packed));

/* uCAN TIMING_FAST command fields */
#define UCAN_TFAST_SJW(s)    ((s) & UCAN_TFAST_SJW_MASK)
#define UCAN_TFAST_TSEG2(t)    ((t) & UCAN_TFAST_TSEG2_MASK)
#define UCAN_TFAST_TSEG1(t)    ((t) & UCAN_TFAST_TSEG1_MASK)
#define UCAN_TFAST_BRP(b)    cpu_to_le16((b) & UCAN_TFAST_BRP_MASK)

struct ucan_timing_fast {
  uint16_t  opcode_channel;

  uint8_t  unused;
  uint8_t  sjw;    /* Sync Jump Width */
  uint8_t  tseg2;    /* Timing SEGment 2 */
  uint8_t  tseg1;    /* Timing SEGment 1 */

  uint16_t  brp;    /* BaudRate Prescaler */
}__attribute__((packed));

/* (old) uCAN FILTER_STD command fields */
#define UCAN_FLTSTD_ROW_IDX_BITS  6

struct ucan_filter_std {
  uint16_t  opcode_channel;

  uint16_t  idx;
  uint32_t  mask;    /* CAN-ID bitmask in idx range */
}__attribute__((packed));

/* uCAN SET_STD_FILTER command fields */
struct ucan_std_filter {
  uint16_t  opcode_channel;

  uint8_t  unused;
  uint8_t  idx;
  uint32_t  mask;    /* CAN-ID bitmask in idx range */
}__attribute__((packed));

/* uCAN TX_ABORT commands fields */
#define UCAN_TX_ABORT_FLUSH    0x0001

struct ucan_tx_abort {
  uint16_t  opcode_channel;

  uint16_t  flags;
  uint32_t  unused;
}__attribute__((packed));

/* uCAN WR_ERR_CNT command fields */
#define UCAN_WRERRCNT_TE    0x4000  /* Tx error cntr write Enable */
#define UCAN_WRERRCNT_RE    0x8000  /* Rx error cntr write Enable */

struct ucan_wr_err_cnt {
  uint16_t  opcode_channel;

  uint16_t  sel_mask;
  uint8_t  tx_counter;  /* Tx error counter new value */
  uint8_t  rx_counter;  /* Rx error counter new value */

  uint16_t  unused;
}__attribute__((packed));

/* uCAN SET_EN_OPTION/CLR_DIS_OPTION commands fields */
#define UCAN_OPTION_ERROR    0x0001
#define UCAN_OPTION_BUSLOAD    0x0002
#define UCAN_OPTION_ISO_MODE    0x0004
#define UCAN_OPTION_LO_MODE    0x0008  /* Diag FD only */
#define UCAN_OPTION_20AB_MODE    0x0010  /* force CAN 2.0 A/B format */

struct ucan_option {
  uint16_t  opcode_channel;

  uint16_t  mask;
}__attribute__((packed));

/* uCAN received messages global format */
struct ucan_msg {
  uint16_t  size;
  uint16_t  type;
  uint32_t  ts_low;
  uint32_t  ts_high;
}__attribute__((packed));

/* uCAN flags for CAN/CANFD messages */
#define UCAN_MSG_API_SRR    0x80  /* tx frame echo */
#define UCAN_MSG_ERROR_STATE_IND  0x40  /* error state indicator */
#define UCAN_MSG_BITRATE_SWITCH    0x20  /* bitrate switch */
#define UCAN_MSG_EXT_DATA_LEN    0x10  /* extended data length */
#define UCAN_MSG_SINGLE_SHOT    0x08
#define UCAN_MSG_HW_SRR      0x04  /* loopback */
#define UCAN_MSG_EXT_ID      0x02
#define UCAN_MSG_RTR      0x01

#define UCAN_MSG_CHANNEL(m)    ((m)->channel_dlc & 0xf)
#define UCAN_MSG_DLC(m)      ((m)->channel_dlc >> 4)

struct ucan_rx_msg {
  uint16_t  size;
  uint16_t  type;
  uint32_t  ts_low;
  uint32_t  ts_high;
  uint32_t  tag_low;
  uint32_t  tag_high;
  uint8_t  channel_dlc;
  uint8_t  client;
  uint16_t  flags;
  uint32_t  can_id;
  uint8_t  d[];
}__attribute__((packed));

/* uCAN error types */
#define UCAN_ERMSG_BIT_ERROR    0
#define UCAN_ERMSG_FORM_ERROR    1
#define UCAN_ERMSG_STUFF_ERROR    2
#define UCAN_ERMSG_OTHER_ERROR    3
#define UCAN_ERMSG_ERR_CNT_DEC    4

#define UCAN_ERMSG_CHANNEL(e)    ((e)->channel_type_d & 0x0f)
#define UCAN_ERMSG_ERRTYPE(e)    (((e)->channel_type_d >> 4) & 0x07)
#define UCAN_ERMSG_D(e)      ((e)->channel_type_d & 0x80)

#define UCAN_ERMSG_ERRCODE(e)    ((e)->code_g & 0x7f)
#define UCAN_ERMSG_G(e)      ((e)->code_g & 0x80)

struct ucan_error_msg {
  uint16_t  size;
  uint16_t  type;
  uint32_t  ts_low;
  uint32_t  ts_high;
  uint8_t  channel_type_d;
  uint8_t  code_g;
  uint8_t  tx_err_cnt;
  uint8_t  rx_err_cnt;
}__attribute__((packed));

#define UCAN_STMSG_CHANNEL(e)    ((e)->channel_p_w_b & 0x0f)
#define UCAN_STMSG_RB(e)    ((e)->channel_p_w_b & 0x10)
#define UCAN_STMSG_PASSIVE(e)    ((e)->channel_p_w_b & 0x20)
#define UCAN_STMSG_WARNING(e)    ((e)->channel_p_w_b & 0x40)
#define UCAN_STMSG_BUSOFF(e)    ((e)->channel_p_w_b & 0x80)

struct ucan_status_msg {
  uint16_t  size;
  uint16_t  type;
  uint32_t  ts_low;
  uint32_t  ts_high;
  uint8_t  channel_p_w_b;
  uint8_t  unused[3];
}__attribute__((packed));

#define UCAN_BLMSG_CHANNEL(e)    ((e)->channel & 0x0f)

struct ucan_bus_load_msg {
  uint16_t  size;
  uint16_t  type;
  uint32_t  ts_low;
  uint32_t  ts_high;
  uint8_t  channel;
  uint8_t  unused;
  uint16_t  bus_load;
}__attribute__((packed));

/* uCAN transmitted message format */
#define UCAN_MSG_CHANNEL_DLC(c, d)  (((c) & 0xf) | ((d) << 4))

struct ucan_tx_msg {
  uint16_t  size;
  uint16_t  type;
  uint32_t  tag_low;
  uint32_t  tag_high;
  uint8_t  channel_dlc;
  uint8_t  client;
  uint16_t  flags;
  uint32_t  can_id;
  uint8_t  d[];
}__attribute__((packed));

/* uCAN Tx Pause record */
#define UCAN_TXPAUSE_DELAY_MAX    0x3ff
#define UCAN_TXPAUSE_DELAY(d)    ((d) & UCAN_TXPAUSE_DELAY_MAX)

struct ucan_tx_pause {
  uint16_t  size;
  uint16_t  type;
  uint16_t  delay;      /* pause in µs (10-low order bits) */
  uint16_t  reserved;
}__attribute__((packed));

#endif
