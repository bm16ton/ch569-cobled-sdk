/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_uart.h
* Author             : WCH, bvernoux
* Version            : V1.1
* Date               : 2022/08/07
* Description
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef __CH56x_UART_H__
#define __CH56x_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "CH56xSFR.h"

#if defined (__ASSEMBLER__)
#define MMIO32(addr)		(addr)
#else
#define MMIO32(addr)		(*(volatile uint32_t *)(addr))
#endif
/**
  * @brief	base addresses
  */
#define UART0		0x40003000
#define UART1		0x40003400
#define UART2		0x40003800
#define UART3		0x40003C00

/**
  * @brief	register offsets
  */
#define R8_UART_MCR(uart)			MMIO32((uart) + 0x00)	/* MODEM CONTROL */
#define R8_UART_IER(uart)			MMIO32((uart) + 0x01)	/* INTERRUPT ENABLE */
#define R8_UART_FCR(uart)			MMIO32((uart) + 0x02)	/* FIFO CONTROL */
#define R8_UART_LCR(uart)			MMIO32((uart) + 0x03)	/* LINE CONTROL */
#define R8_UART_IIR(uart)			MMIO32((uart) + 0x04)	/* INTERRUPT IDENT */
#define R8_UART_LSR(uart)			MMIO32((uart) + 0x05)	/* LINE STATUS REGISTER */
#define R8_UART_RBR(uart)			MMIO32((uart) + 0x08)	/* RECEIVE BUFFER */
#define R8_UART_THR(uart)			MMIO32((uart) + 0x08)	/* TX HOLD BUFFER */
#define R8_UART_RFC(uart)			MMIO32((uart) + 0x0A)	/* RX FIFO COUNT */
#define R8_UART_TFC(uart)			MMIO32((uart) + 0x0B)	/* TX FIFO COUNT */
#define R8_UART_DL(uart)			MMIO32((uart) + 0x0C)	/* BAUDRATE DIVISOR LATCH */
#define R8_UART_DIV(uart)			MMIO32((uart) + 0x0E)	/* PRESCALER DIVISOR */


/**
  * @brief	LINE CONTROL MASKS AND OFFSETS
  */
#define UART_WORDSIZE_5		(0x00 << 0)
#define UART_WORDSIZE_6		(0x01 << 0)
#define UART_WORDSIZE_7		(0x02 << 0)
#define UART_WORDSIZE_8		(0x03 << 0)

#define UART_STOPBITS_OFFS		2U
#define UART_STOPBITS_1		(0x00 << 2)   /* 1 stop bit */
#define UART_STOPBITS_2		(0x01 << 2)   /* 2 stop bits */

#define UART_PARITY_EN			(0x01 << 3)  /* MUST BE SET BEFORE PARITY LEVEL */

#define UART_PARITY_ODD		(0x00 << 4)
#define UART_PARITY_EVEN		(0x01 << 4)
#define UART_PARITY_MARK		(0x02 << 4)
#define UART_PARITY_CLR		(0x03 << 4)

#define UART_BREAK_EN  		(0x01 << 6)


/**
  * @brief	Line Error Status Definition
  */
#define  STA_ERR_BREAK   RB_LSR_BREAK_ERR   // Data interval error
#define  STA_ERR_FRAME   RB_LSR_FRAME_ERR   // Data frame error
#define  STA_ERR_PAR     RB_LSR_PAR_ERR     // Parity bit error
#define  STA_ERR_FIFOOV  RB_LSR_OVER_ERR    // Receive data overflow

#define  STA_TXFIFO_EMP  RB_LSR_TX_FIFO_EMP // The current transmit FIFO is empty and can continue to fill the transmit data
#define  STA_TXALL_EMP   RB_LSR_TX_ALL_EMP  // All data have been sent
#define  STA_RECV_DATA   RB_LSR_DATA_RDY    // Currently receiving data
typedef unsigned char *puint8_t;
/**
  * @brief  Serial port byte trigger configuration
  */
typedef enum
{
	UART_1BYTE_TRIG = 0, // 1 Byte trigger
	UART_2BYTE_TRIG = 1, // 2 Byte trigger
	UART_4BYTE_TRIG = 2, // 4 Byte trigger
	UART_7BYTE_TRIG = 3, // 7 Byte trigger

} UARTByteTRIGTypeDef;


void uart_set_stopbits(uint32_t uart, uint32_t stopbits);
uint32_t uart_get_stopbits(uint32_t uart);
void cdc_uart_set_stopbits(uint32_t uart, uint32_t stopbits);
void uart_set_parity(uint32_t uart, uint32_t parity);
uint32_t uart_get_parity(uint32_t uart);
void uart_set_wordsize(uint32_t uart, uint32_t wsize);
uint32_t uart_get_databits(uint32_t uart);
void uart_set_break(uint32_t uart, bool enable);
/****************** UART0 */
void UART0_init(uint32_t baudrate, uint32_t systemclck);
void UART0_DefInit(void); /* Serial Default initialization configuration */
void UART0_BaudRateCfg(uint32_t baudrate); /* Serial Baud rate configuration */
void UART0_ByteTrigCfg(UARTByteTRIGTypeDef b); /*Serial Byte-triggered interrupt configuration */
void UART0_INTCfg(uint8_t s,  uint8_t i); /* Serial Interrupt configuration */
void UART0_Reset(void); /* Serial Software reset */

#define UART0_CLR_RXFIFO() (R8_UART0_FCR |= RB_FCR_RX_FIFO_CLR) /* Clear current RX FIFO */
#define UART0_CLR_TXFIFO() (R8_UART0_FCR |= RB_FCR_TX_FIFO_CLR) /* Clear the current TX FIFO */

#define UART0_GetITFlag() (R8_UART0_IIR & RB_IIR_INT_MASK) /* Get current interrupt flag */
// please refer to LINE error and status define
#define UART0_GetLinSTA() (R8_UART0_LSR) /* Get current communication status */
#define UART0_GetMSRSTA() (R8_UART0_MSR) /* Get the current flow control state, only applicable UART0 */

#define	UART0_SendByte(b) (R8_UART0_THR = b) /* Serial Single byte send */
void UART0_SendString(puint8_t buf, uint16_t l); /* Serial Multibyte send */
#define UART0_tx UART0_SendString

#define	UART0_RecvByte() (R8_UART0_RBR) /* Serial Read single byte */
uint16_t UART0_RecvString(puint8_t buf); /*Serial Read multiple bytes */
uint16_t UART0_rx(uint8_t* buf, int buf_len_max);

/****************** UART1 */
void UART1_init(uint32_t baudrate, uint32_t systemclck);
void UART1_DefInit(void); /*Serial Default initialization configuration */
void UART1_BaudRateCfg(uint32_t baudrate); /*Serial Baud rate configuration */
void UART1_ByteTrigCfg(UARTByteTRIGTypeDef b); /*Serial Byte-triggered interrupt configuration */
void UART1_INTCfg(uint8_t s,  uint8_t i); /*Serial Interrupt configuration */
void UART1_Reset(void); /*Serial Software reset */

#define UART1_CLR_RXFIFO() (R8_UART1_FCR |= RB_FCR_RX_FIFO_CLR) /* Clear current receive FIFO */
#define UART1_CLR_TXFIFO() (R8_UART1_FCR |= RB_FCR_TX_FIFO_CLR) /* Clear the current TX FIFO */

#define UART1_GetITFlag() (R8_UART1_IIR&RB_IIR_INT_MASK) /* Get current interrupt flag */
// please refer to LINE error and status define
#define UART1_GetLinSTA() (R8_UART1_LSR) /* Get current communication status */

#define	UART1_SendByte(b) (R8_UART1_THR = b) /*Serial Single byte send */
void UART1_SendString(puint8_t buf, uint16_t l); /*Serial Multibyte send */
#define UART1_tx UART1_SendString

#define	UART1_RecvByte() (R8_UART1_RBR) /* Serial Read single byte */
uint16_t UART1_RecvString(puint8_t buf); /* Serial Read multiple bytes */
uint16_t UART1_rx(uint8_t* buf, int buf_len_max);

/****************** UART2 */
void UART2_init(uint32_t baudrate, uint32_t systemclck);
void UART2_DefInit(void); /*Serial Default initialization configuration */
void UART2_BaudRateCfg(uint32_t baudrate); /*Serial Baud rate configuration */
void UART2_ByteTrigCfg(UARTByteTRIGTypeDef b); /*Serial Byte-triggered interrupt configuration */
void UART2_INTCfg(uint8_t s,  uint8_t i); /*Serial Interrupt configuration */
void UART2_Reset(void); /*Serial Software reset */

#define UART2_CLR_RXFIFO() (R8_UART2_FCR |= RB_FCR_RX_FIFO_CLR) /* Clear current receive FIFO */
#define UART2_CLR_TXFIFO() (R8_UART2_FCR |= RB_FCR_TX_FIFO_CLR) /* Clear the current TX FIFO */

#define UART2_GetITFlag() (R8_UART2_IIR&RB_IIR_INT_MASK) /* Get current interrupt flag */
// please refer to LINE error and status define
#define UART2_GetLinSTA() (R8_UART2_LSR) /* Get current communication status */

#define	UART2_SendByte(b) (R8_UART2_THR = b) /* Serial Single byte send */
void UART2_SendString(puint8_t buf, uint16_t l); /* Serial Multibyte send */
#define UART2_tx UART2_SendString

#define	UART2_RecvByte() (R8_UART2_RBR) /* Serial Read single byte */
uint16_t UART2_RecvString(puint8_t buf); /* Serial Read multiple bytes */
uint16_t UART2_rx(uint8_t* buf, int buf_len_max);

/****************** UART3 */
void UART3_init(uint32_t baudrate, uint32_t systemclck);
void UART3_DefInit(void); /* Serial Default initialization configuration */
void UART3_BaudRateCfg(uint32_t baudrate); /* Serial Baud rate configuration */
void UART3_ByteTrigCfg(UARTByteTRIGTypeDef b); /* Serial Byte-triggered interrupt configuration */
void UART3_INTCfg(uint8_t s,  uint8_t i); /* Serial Interrupt configuration */
void UART3_Reset(void); /* Serial Software reset */

#define UART3_CLR_RXFIFO() (R8_UART3_FCR |= RB_FCR_RX_FIFO_CLR) /* Clear current RX FIFO */
#define UART3_CLR_TXFIFO() (R8_UART3_FCR |= RB_FCR_TX_FIFO_CLR) /* Clear the current TX FIFO */

#define UART3_GetITFlag() (R8_UART3_IIR&RB_IIR_INT_MASK) /* Get current interrupt flag */
// please refer to LINE error and status define
#define UART3_GetLinSTA() (R8_UART3_LSR) /* Get current communication status */

#define	UART3_SendByte(b) (R8_UART3_THR = b) /*Serial Single byte send */
void UART3_SendString(puint8_t buf, uint16_t l); /*Serial Multibyte send */
#define UART3_tx UART3_SendString

#define	UART3_RecvByte() (R8_UART3_RBR) /*Serial Read single byte */
uint16_t UART3_RecvString(puint8_t buf); /*Serial Read multiple bytes */
uint16_t UART3_rx(uint8_t* buf, int buf_len_max);

#ifdef __cplusplus
}
#endif

#endif  // __CH56x_UART_H__
