/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCAN-USB Pro / PCAN-USB Pro FD firmware objects
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
#ifndef UCAN_USB_H
#define UCAN_USB_H

#include "pcanfd_ucan.h"

/*
 * Extended commands (non uCAN commands):
 *
 * Clock Modes command
 */
#define UCAN_USB_CMD_CLK_SET    0x80

#define UCAN_USB_CLK_80MHZ    0x0
#define UCAN_USB_CLK_60MHZ    0x1
#define UCAN_USB_CLK_40MHZ    0x2
#define UCAN_USB_CLK_30MHZ    0x3
#define UCAN_USB_CLK_24MHZ    0x4
#define UCAN_USB_CLK_20MHZ    0x5
#define UCAN_USB_CLK_DEF    UCAN_USB_CLK_80MHZ

struct ucan_usb_clock {
    uint16_t opcode_channel;

    uint8_t mode;
    uint8_t unused[5];
}__attribute__((packed));

/* LED control command */
#define UCAN_USB_CMD_LED_SET    0x86

#define UCAN_USB_LED_DEV    0x00
#define UCAN_USB_LED_FAST    0x01
#define UCAN_USB_LED_SLOW    0x02
#define UCAN_USB_LED_FIXED    0x03
#define UCAN_USB_LED_OFF    0x04
#define UCAN_USB_LED_DEF    UCAN_USB_LED_DEV

struct ucan_usb_led {
    uint16_t opcode_channel;

    uint8_t mode;
    uint8_t unused[5];
}__attribute__((packed));

/* Extended usage of uCAN commands UCAN_CMD_XXX_YY_OPTION for PCAN-USB FD */
#define UCAN_USB_OPTION_CALIBRATION  0x8000
#define UCAN_USB_OPTION_DEBUG    0x4000
#define UCAN_USB_OPTION_FAST_FWD  0x2000

struct ucan_usb_option {
    uint16_t opcode_channel;

    uint16_t ext_mask;
    uint16_t unused;
    uint16_t usb_mask;
}__attribute__((packed));

/* Extended usage of uCAN messages for PCAN-USB Pro FD */
#define UCAN_USB_MSG_CALIBRATION  0x100

struct ucan_usb_ts_msg {
    uint16_t size;
    uint16_t type;
    uint32_t ts_low;
    uint32_t ts_high;
    uint16_t usb_frame_index;
    uint16_t unused;
}__attribute__((packed));

#define UCAN_USB_MSG_OVERRUN    0x101

#define UCAN_USB_OVMSG_CHANNEL(o)  ((o)->channel & 0xf)

struct ucan_usb_ovr_msg {
    uint16_t size;
    uint16_t type;
    uint32_t ts_low;
    uint32_t ts_high;
    uint8_t channel;
    uint8_t unused[3];
}__attribute__((packed));

#define UCAN_USB_CMD_DEVID_SET    0x81

struct ucan_usb_device_id {
    uint16_t opcode_channel;

    uint16_t unused;
    uint32_t device_id;
}__attribute__((packed));

#define UCAN_USB_CMD_LOCINFO_SET  0x83
#define UCAN_USB_CMD_LOCINFO_GET  0x84
#define UCAN_USB_CMD_LOCINFO_RSP  0x85

struct ucan_usb_location_info {
    uint16_t opcode_channel;

    uint8_t index;
    uint8_t w;
    uint16_t unused;
    uint8_t d0;
    uint8_t d1;
}__attribute__((packed));

#define UCAN_USB_LOCINFO_LEN    250

#define UCAN_USB_MSG_DEBUG    0x200

struct ucan_usb_dbg_msg {
    uint16_t size;
    uint16_t type;
    uint8_t d[64];
}__attribute__((packed));

/* these are PCAN-Chip specific cmds to control I/O pins */
#define UCAN_USB_CMD_DPIN_CFG_SET  0x8d
#define UCAN_USB_CMD_DPIN_CFG_REQ  0xca
#define UCAN_USB_CMD_DPIN_CFG_RSP  0xcb

#define UCAN_USB_CMD_DPIN_VAL_SET  0x8e
#define UCAN_USB_CMD_DPIN_VAL_REQ  0xcc
#define UCAN_USB_CMD_DPIN_VAL_RSP  0xcd

#define UCAN_USB_CMD_DPIN_SET_HIGH  0x8f
#define UCAN_USB_CMD_DPIN_SET_LOW  0x90

#define UCAN_USB_CMD_ANAL_VAL_REQ  0xce
#define UCAN_USB_CMD_ANAL_VAL_RSP  0xcf

struct ucan_usb_io_ctrl {
    uint16_t opcode;
    uint16_t reserved;

    uint32_t io_val;
}__attribute__((packed));

#endif
