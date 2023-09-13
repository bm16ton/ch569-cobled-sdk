/*
 *         _______                    _    _  _____ ____
 *        |__   __|                  | |  | |/ ____|  _ \
 *           | | ___  ___ _ __  _   _| |  | | (___ | |_) |
 *           | |/ _ \/ _ \ '_ \| | | | |  | |\___ \|  _ <
 *           | |  __/  __/ | | | |_| | |__| |____) | |_) |
 *           |_|\___|\___|_| |_|\__, |\____/|_____/|____/
 *                               __/ |
 *                              |___/
 *
 * TeenyUSB - light weight usb stack for micro controllers
 *
 * Copyright (c) 2020 XToolBox  - admin@xtoolbox.org
 *                         www.usb.org
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __USB_CDC_H__
#define __USB_CDC_H__
#include "stdint.h"
#include "cdc.h"
#include "packed.h"
#define  USB_CDC_DTR   0x01
#define  USB_CDC_RTS   0x02

typedef struct _usb_setup_packet{
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_packet;

/** Enums for \ref usb_cdc_line_coding_t  stopbits field
 *  indicating the CDC stopbits
 */
typedef enum{
    CDC_1STOP = 0,
    CDC_2_STOP = 1,
}cdc_stopbits_t;

/** Enums for \ref usb_cdc_line_coding_t  parity field
 *  indicating the CDC parity
 */
typedef enum{
    CDC_ODD = 0,
    CDC_EVEN = 1,
    CDC_MARK = 2,
    CDC_SPACE = 3,
}cdc_parity_t;

/* Type Defines: */
/** Type define for CDC line coding
 */
 
 /* Type Defines: */
/** Type define for CDC line coding
 */
 
typedef __PACK_BEGIN struct _usb_cdc_line_coding
{
    volatile uint32_t bitrate;        /**< bit rate */
    volatile uint8_t  stopbits;       /**< stop bits, \ref cdc_stopbits_t*/
    volatile uint8_t  parity;         /**< parity, \ref cdc_parity_t */
    volatile uint8_t  databits;       /**< data bits: 5,6,7,8 */
}__PACK_END usb_cdc_line_coding_t;

struct usb_cdc_line_coding {
	volatile uint32_t bitrate;
	volatile uint8_t stopbits;
	volatile uint8_t parity;
	volatile uint8_t databits;
} __attribute__((packed));

enum usb_cdc_line_coding_bCharFormat {
	USB_CDC_1_STOP_BITS			= 0,
	USB_CDC_2_STOP_BITS			= 1,
};

enum usb_cdc_line_coding_bParityType {
	USB_CDC_ODD_PARITY			= 0,
	USB_CDC_EVEN_PARITY			= 1,
	USB_CDC_MARK_PARITY			= 2,
	USB_CDC_SPACE_PARITY			= 3,
};

typedef struct usb_cdc_line_coding usb_cdc_line_coding_s;

/* Type Defines: */
/** Type define for CDC line status
 */
typedef struct _usb_cdc_line_state
{
    uint16_t CDC:1;
    uint16_t DSR:1;
    uint16_t Break:1;
    uint16_t Ring:1;
    uint16_t FramingError:1;
    uint16_t ParityError:1;
    uint16_t Overrun:1;
    uint16_t revserved: 9;
}usb_cdc_line_state_t;

typedef struct _usb_cdc_state
{
    usb_setup_packet req;
    usb_cdc_line_state_t line_state;
}usb_cdc_state_t;



// Abstract Control Management Functional Descriptor bmCapabilities fields
#define CDC_CAP_COMM     1
#define CDC_CAP_LINE     2
#define CDC_CAP_BREAK    4
#define CDC_CAP_NETWORK  8

// CDC request code define
#define CDC_SEND_ENCAPSULATED_COMMAND    0x00
#define CDC_GET_ENCAPSULATED_RESPONSE    0x01
#define CDC_SET_COMM_FEATURE             0x02
#define CDC_GET_COMM_FEATURE             0x03
#define CDC_CLEAR_COMM_FEATURE           0x04
#define CDC_SET_LINE_CODING              0x20
#define CDC_GET_LINE_CODING              0x21
#define CDC_SET_CONTROL_LINE_STATE       0x22
#define CDC_SEND_BREAK                   0x23

// CDC notify code define
#define CDC_RING_DETECT                  0x09
#define CDC_SERIAL_STATE                 0x20

#endif
