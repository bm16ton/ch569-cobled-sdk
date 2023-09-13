/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_uart.c
* Author             : WCH, bvernoux
* Version            : V1.1
* Date               : 2022/07/30
* Description
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_debug_log.h"


//static uint32_t aux_serial_active_baud_rate;
extern vuint32_t vitrul_buad;
/*******************************************************************************
 * @fn     UART0_init
 *
 * @brief  UART0 initialization configuration of serial port:
 *         FIFO enabled, number of trigger point bytes, serial port data length setting,
 *         baud rate and frequency division coefficient
 *
 * @param  baudrate - UART baud rate
 *         systemclck - System clock / MCU frequency in Hz(usually defined in FREQ_SYS)
 *
 * @return   None
 **/
 
 
void UART0_init(uint32_t baudrate, uint32_t systemclck)
{
	uint32_t x;
	uint32_t t = systemclck;
	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;

	R8_UART0_DIV = 1;
	R16_UART0_DL = x;
	R8_UART0_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART0_LCR = RB_LCR_WORD_SZ;
	R8_UART0_IER = RB_IER_TXD_EN;
	R32_PA_SMT |= (1<<6) |(1<<5); // PA6 TXD0 & PA5 RXD0
	R32_PA_DIR |= (1<<6); // PA6 Output
}

/******************************************************************************
 * @fn     UART0_DefInit
 *
 * @brief  Default initialization configuration of serial port:
 *         FIFO enabled, number of trigger point bytes, serial port data length setting,
 *         baud rate and frequency division coefficient
 *
 * @return   None
 */
void UART0_DefInit( void )
{
	UART0_BaudRateCfg( 115200 );
	R8_UART0_FCR = (2<<6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;// FIFO is open, trigger point is 4 bytes
	R8_UART0_LCR = RB_LCR_WORD_SZ;
	R8_UART0_IER = RB_IER_TXD_EN;
	R8_UART0_DIV = 1;
}

/*******************************************************************************
 * @fn     UART1_init
 *
 * @brief  UART1 initialization configuration of serial port:
 *         FIFO enabled, number of trigger point bytes, serial port data length setting,
 *         baud rate and frequency division coefficient
 *
 * @param  baudrate - UART baud rate
 *         systemclck - System clock / MCU frequency in Hz(usually defined in FREQ_SYS)
 *
 * @return   None
 **/
void UART1_init(uint32_t baudrate, uint32_t systemclck)
{
	uint32_t x;
	uint32_t t = systemclck;
	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;

	R8_UART1_DIV = 1;
	R16_UART1_DL = x;
	R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART1_LCR = RB_LCR_WORD_SZ;
	R8_UART1_IER = RB_IER_TXD_EN;
	R32_PA_SMT |= (1<<8) |(1<<7); // PA8 TXD1 & PA7 RXD1
	R32_PA_DIR |= (1<<8); // PA8 Output
}

/*******************************************************************************
 * @fn     UART1_DefInit
 *
 * @brief  Default initialization configuration of serial port:
 *         FIFO enabled, number of trigger point bytes, serial port data length setting,
 *         baud rate and frequency division coefficient
 *
 * @return   None
 **/
void UART1_DefInit( void )
{
	UART1_BaudRateCfg( 115200 );
	R8_UART1_FCR = (2<<6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;// FIFO is open, trigger point is 4 bytes
	R8_UART1_LCR = RB_LCR_WORD_SZ;
	R8_UART1_IER = RB_IER_TXD_EN;
	R8_UART1_DIV = 1;
}

/*******************************************************************************
 * @fn     UART2_init
 *
 * @brief  UART2 initialization configuration of serial port:
 *         FIFO enabled, number of trigger point bytes, serial port data length setting,
 *         baud rate and frequency division coefficient
 *
 * @param  baudrate - UART baud rate
 *         systemclck - System clock / MCU frequency in Hz(usually defined in FREQ_SYS)
 *
 * @return   None
 **/
void UART2_init(uint32_t baudrate, uint32_t systemclck)
{
	uint32_t x;
	uint32_t t = systemclck;
	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;

	R8_UART2_DIV = 1;
	R16_UART2_DL = x;
	R8_UART2_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART2_LCR = RB_LCR_WORD_SZ;
	R8_UART2_IER = RB_IER_TXD_EN;
	R32_PA_SMT |= (1<<3) |(1<<2); // PA3 TXD2 & PA2 RXD2
	R32_PA_DIR |= (1<<3); // PA3 Output
}

/*******************************************************************************
 * @fn     UART2_DefInit
 *
 * @brief  Default initialization configuration of serial port:
 *         FIFO enabled, number of trigger point bytes, serial port data length setting,
 *         baud rate and frequency division coefficient
 *
 * @return   None
 */
void UART2_DefInit( void )
{
	UART2_BaudRateCfg( 115200 );
	R8_UART2_FCR = (2<<6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;// FIFO is open, trigger point is 4 bytes
	R8_UART2_LCR = RB_LCR_WORD_SZ;
	R8_UART2_IER = RB_IER_TXD_EN;
	R8_UART2_DIV = 1;
}

/*******************************************************************************
 * @fn     UART3_init
 *
 * @brief  UART3 initialization configuration of serial port:
 *         FIFO enabled, number of trigger point bytes, serial port data length setting,
 *         baud rate and frequency division coefficient
 *
 * @param  baudrate - UART baud rate
 *         systemclck - System clock / MCU frequency in Hz(usually defined in FREQ_SYS)
 *
 * @return   None
 **/
void UART3_init(uint32_t baudrate, uint32_t systemclck)
{
	uint32_t x;
	uint32_t t = systemclck;
	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;

	R8_UART3_DIV = 1;
	R16_UART3_DL = x;
	R8_UART3_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART3_LCR = RB_LCR_WORD_SZ;
	R8_UART3_IER = RB_IER_TXD_EN;
	R32_PB_SMT |= (1<<4) |(1<<3); // PB4 TXD3 & PB3 RXD3
	R32_PB_DIR |= (1<<4); // PB4 Output
}

/*******************************************************************************
 * @fn     UART3_DefInit
 *
 * @brief  Default initialization configuration of serial port:
 *         FIFO enabled, number of trigger point bytes, serial port data length setting,
 *         baud rate and frequency division coefficient
 *
 * @return   None
 */
void UART3_DefInit( void )
{
	UART3_BaudRateCfg( 115200 );
	R8_UART3_FCR = (2<<6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;// FIFO is open, trigger point is 4 bytes
	R8_UART3_LCR = RB_LCR_WORD_SZ;
	R8_UART3_IER = RB_IER_TXD_EN;
	R8_UART3_DIV = 1;
}

/*******************************************************************************
 * @fn     UART0_BaudRateCfg
 *
 * @brief  Serial port baud rate configuration
 *
 * @return   None
 */
void UART0_BaudRateCfg( uint32_t baudrate )
{
	uint32_t	x;

	x = 10 * FREQ_SYS / 8 / baudrate;
	x = ( x + 5 ) / 10;
	R16_UART0_DL = (uint16_t)x;
}

/*******************************************************************************
 * @fn     UART1_BaudRateCfg
 *
 * @brief  Serial port baud rate configuration
 *
 * @return   None
 */
void UART1_BaudRateCfg( uint32_t baudrate )
{
	uint32_t	x;

	x = 10 * FREQ_SYS / 8 / baudrate;
	x = ( x + 5 ) / 10;
	R16_UART1_DL = (uint16_t)x;
}

/*******************************************************************************
 * @fn     UART2_BaudRateCfg
 *
 * @brief  Serial port baud rate configuration
 *
 * @return   None
 */
void UART2_BaudRateCfg( uint32_t baudrate )
{
	uint32_t	x;

	x = 10 * FREQ_SYS / 8 / baudrate;
	x = ( x + 5 ) / 10;
	R16_UART2_DL = (uint16_t)x;
}

/*******************************************************************************
 * @fn     UART3_BaudRateCfg
 *
 * @brief  Serial port baud rate configuration
 *
 * @return   None
 */
void UART3_BaudRateCfg( uint32_t baudrate )
{
	uint32_t	x;

	x = 10 * FREQ_SYS / 8 / baudrate;
	x = ( x + 5 ) / 10;
	R16_UART3_DL = (uint16_t)x;
}

/*******************************************************************************
 * @fn     UART0_ByteTrigCfg
 *
 * @brief  Serial port byte trigger interrupt configuration
 *
 * @param  b - Trigger bytes
 *           refer to UARTByteTRIGTypeDef
 * @return   None
 */
void UART0_ByteTrigCfg( UARTByteTRIGTypeDef b )
{
	R8_UART0_FCR = (R8_UART0_FCR&~RB_FCR_FIFO_TRIG)|(b<<6);
}

/*******************************************************************************
 * @fn     UART1_ByteTrigCfg
 *
 * @brief  Serial port byte trigger interrupt configuration
 *
 * @param  b - Trigger bytes
 *           refer to UARTByteTRIGTypeDef
 * @return   None
 **/
void UART1_ByteTrigCfg( UARTByteTRIGTypeDef b )
{
	R8_UART1_FCR = (R8_UART1_FCR&~RB_FCR_FIFO_TRIG)|(b<<6);
}

/*******************************************************************************
 * @fn     UART2_ByteTrigCfg
 *
 * @brief  Serial port byte trigger interrupt configuration
 *
 * @param  b - Trigger bytes
 *           refer to UARTByteTRIGTypeDef
 * @return   None
 */
void UART2_ByteTrigCfg( UARTByteTRIGTypeDef b )
{
	R8_UART2_FCR = (R8_UART2_FCR&~RB_FCR_FIFO_TRIG)|(b<<6);
}

/*******************************************************************************
 * @fn     UART3_ByteTrigCfg
 *
 * @brief  Serial port byte trigger interrupt configuration
 *
 * @param  b - Trigger bytes
 *           refer to UARTByteTRIGTypeDef
 * @return   None
 ***/
void UART3_ByteTrigCfg( UARTByteTRIGTypeDef b )
{
	R8_UART3_FCR = (R8_UART3_FCR&~RB_FCR_FIFO_TRIG)|(b<<6);
}

/*******************************************************************************
 * @fn     UART0_INTCfg
 *
 * @brief  Serial port interrupt configuration
 *
 * @param  s - Interrupt control state
 *				ENABLE  - Enable the corresponding interrupt
 *				DISABLE - Disable the corresponding interrupt
 *		   i - Interrupt type
 *				RB_IER_MODEM_CHG  - Modem Input Change Interrupt Enable bit (only UART0 supported)
 *				RB_IER_LINE_STAT  - Receive Line Status Interrupt
 *				RB_IER_THR_EMPTY  - Transmit Holding Register Empty Interrupt
 *				RB_IER_RECV_RDY   - Receive data interrupt
 * @return   None
 **/
void UART0_INTCfg( uint8_t s,  uint8_t i )
{
	if( s )
	{
		R8_UART0_IER |= i;
		R8_UART0_MCR |= RB_MCR_INT_OE;
	}
	else
	{
		R8_UART0_IER &= ~i;
	}
}

/*******************************************************************************
 * @fn     UART1_INTCfg
 *
 * @brief  Serial port interrupt configuration
 *
 * @param  s - Interrupt control state
 *				ENABLE  - Enable the corresponding interrupt
 *				DISABLE - Disable the corresponding interrupt
 *		   i -  Interrupt type
 *				RB_IER_MODEM_CHG  - Modem Input Change Interrupt Enable bit (only UART0 supported)
 *				RB_IER_LINE_STAT  - Receive Line Status Interrupt
 *				RB_IER_THR_EMPTY  - Transmit Holding Register Empty Interrupt
 *				RB_IER_RECV_RDY   - Receive data interrupt
 *
 * @return   None
 **/
void UART1_INTCfg( uint8_t s,  uint8_t i )
{
	if( s )
	{
		R8_UART1_IER |= i;
		R8_UART1_MCR |= RB_MCR_INT_OE;
	}
	else
	{
		R8_UART1_IER &= ~i;
	}
}

/*******************************************************************************
 * @fn     UART2_INTCfg
 *
 * @brief  Serial port interrupt configuration
 *
 * @param  s - Interrupt control state
 *				ENABLE  - Enable the corresponding interrupt
 *				DISABLE - Disable the corresponding interrupt
 *		   i -  Interrupt type
 *				RB_IER_MODEM_CHG  - Modem Input Change Interrupt Enable bit (only UART0 supported)
 *				RB_IER_LINE_STAT  - Receive Line Status Interrupt
 *				RB_IER_THR_EMPTY  - Transmit Holding Register Empty Interrupt
 *				RB_IER_RECV_RDY   - Receive data interrupt
 *
 * @return   None
 **/
void UART2_INTCfg( uint8_t s,  uint8_t i )
{
	if( s )
	{
		R8_UART2_IER |= i;
		R8_UART2_MCR |= RB_MCR_INT_OE;
	}
	else
	{
		R8_UART2_IER &= ~i;
	}
}

/*******************************************************************************
 * @fn     UART3_INTCfg
 *
 * @brief  Serial port interrupt configuration
 *
 * @param  s - Interrupt control state
 *				ENABLE  - Enable the corresponding interrupt
 *				DISABLE - Disable the corresponding interrupt
 *		   i -  Interrupt type
 *				RB_IER_MODEM_CHG  - Modem Input Change Interrupt Enable bit (only UART0 supported)
 *				RB_IER_LINE_STAT  - Receive Line Status Interrupt
 *				RB_IER_THR_EMPTY  - Transmit Holding Register Empty Interrupt
 *				RB_IER_RECV_RDY   - Receive data interrupt
 *
 * @return   None
 **/
void UART3_INTCfg( uint8_t s,  uint8_t i )
{
	if( s )
	{
		R8_UART3_IER |= i;
		R8_UART3_MCR |= RB_MCR_INT_OE;
	}
	else
	{
		R8_UART3_IER &= ~i;
	}
}

/*******************************************************************************
 * @fn     UART0_Reset
 *
 * @brief  Serial software reset
 *
 * @return  None
 **/
void UART0_Reset( void )
{
	R8_UART0_IER = RB_IER_RESET;
}

/*******************************************************************************
 * @fn     UART1_Reset
 *
 * @brief  Serial software reset
 *
 * @return  None
 **/
void UART1_Reset( void )
{
	R8_UART1_IER = RB_IER_RESET;
}

/*******************************************************************************
 * @fn     UART2_Reset
 *
 * @brief  Serial software reset
 *
 * @return  None
 **/
void UART2_Reset( void )
{
	R8_UART2_IER = RB_IER_RESET;
}

/*******************************************************************************
 * @fn     UART3_Reset
 *
 * @brief  Serial software reset
 *
 * @return  None
 **/
void UART3_Reset( void )
{
	R8_UART3_IER = RB_IER_RESET;
}

/*******************************************************************************
 * @fn     UART0_SendString
 *
 * @brief  Serial multi-byte sending
 *
 * @param  buf - The first address of the data content to be sent
 *         l - Length of data to be sent
 * @return   None
 */
void UART0_SendString( puint8_t buf, uint16_t l )
{
	uint16_t len = l;

	while(len)
	{
		if(R8_UART0_TFC != UART_FIFO_SIZE)
		{
			R8_UART0_THR = *buf++;
			len--;
		}
	}
}

/*******************************************************************************
 * @fn     UART1_SendString
 *
 * @brief  Serial multi-byte sending
 *
 * @param  buf - The first address of the data content to be sent
 *         l - Length of data to be sent
 * @return   None
 */
void UART1_SendString( puint8_t buf, uint16_t l )
{
	uint16_t len = l;

	while(len)
	{
		if(R8_UART1_TFC != UART_FIFO_SIZE)
		{
			R8_UART1_THR = *buf++;
			len--;
		}
	}
}

/*******************************************************************************
 * @fn     UART2_SendString
 *
 * @brief  Serial multi-byte sending
 *
 * @param  buf - The first address of the data content to be sent
 *         l - Length of data to be sent
 * @return   None
 */
void UART2_SendString( puint8_t buf, uint16_t l )
{
	uint16_t len = l;

	while(len)
	{
		if(R8_UART2_TFC != UART_FIFO_SIZE)
		{
			R8_UART2_THR = *buf++;
			len--;
		}
	}
}

/*******************************************************************************
 * @fn     UART3_SendString
 *
 * @brief  Serial multi-byte sending
 *
 * @param  buf - The first address of the data content to be sent
 *         l - Length of data to be sent
 * @return   None
 */
void UART3_SendString( puint8_t buf, uint16_t l )
{
	uint16_t len = l;

	while(len)
	{
		if(R8_UART3_TFC != UART_FIFO_SIZE)
		{
			R8_UART3_THR = *buf++;
			len--;
		}
	}
}

/*******************************************************************************
 * @fn     UART0_RecvString
 *
 * @brief  Serial read multibyte
 *
 * @param  buf - Read data storage buffer first address
 *
 * @return Read data length
 */
uint16_t UART0_RecvString( puint8_t buf )
{
	uint16_t len = 0;

	while( R8_UART0_RFC )
	{
		*buf++ = R8_UART0_RBR;
		len ++;
	}

	return (len);
}

/*******************************************************************************
 * @fn     UART1_RecvString
 *
 * @brief  Serial read multibyte
 *
 * @param  buf - Read data storage buffer first address
 *
 * @return Read data length
 */
uint16_t UART1_RecvString( puint8_t buf )
{
	uint16_t len = 0;

	while( R8_UART1_RFC )
	{
		*buf++ = R8_UART1_RBR;
		len ++;
	}

	return (len);
}

/*******************************************************************************
 * @fn     UART1_rx
 *
 * @brief  UART1 receive data
 *
 * @param  buf - Buffer to receive data
 * @param  buf_len_max - Maximum data to receive
 *
 * @return nb data received
 */
uint16_t UART1_rx(uint8_t* buf, int buf_len_max)
{
	uint16_t len = 0;

	while( R8_UART1_RFC )
	{
		if(len < buf_len_max)
		{
			*buf++ = R8_UART1_RBR;
			len++;
		}
		else
		{
			/* Not enough space in buffer exit loop */
			break;
		}
	}
	return (len);
}

/*******************************************************************************
 * @fn     UART2_RecvString
 *
 * @brief  Serial read multibyte
 *
 * @param  buf - Read data storage buffer first address
 *
 * @return Read data length
 */

uint16_t UART2_RecvString( puint8_t buf )
{
	uint16_t len = 0;

	while( R8_UART2_RFC )
	{
		*buf++ = R8_UART2_RBR;
		len ++;
	}

	return (len);
}

/*******************************************************************************
 * @fn     UART2_rx
 *
 * @brief  UART2 receive data
 *
 * @param  buf - Buffer to receive data
 * @param  buf_len_max - Maximum data to receive
 *
 * @return nb data received
 */
uint16_t UART2_rx(uint8_t* buf, int buf_len_max)
{
	uint16_t len = 0;

	while( R8_UART2_RFC )
	{
		if(len < buf_len_max)
		{
			*buf++ = R8_UART2_RBR;
			len++;
		}
		else
		{
			/* Not enough space in buffer exit loop */
			break;
		}
	}
	return (len);
}

/*******************************************************************************
 * @fn     UART3_RecvString
 *
 * @brief  Serial read multibyte
 *
 * @param  buf - Read data storage buffer first address
 *
 * @return Read data length
 */

uint16_t UART3_RecvString( puint8_t buf )
{
	uint16_t len = 0;

	while( R8_UART3_RFC )
	{
		*buf++ = R8_UART3_RBR;
		len ++;
	}

	return (len);
}

/*******************************************************************************
 * @fn     UART3_rx
 *
 * @brief  UART3 receive data
 *
 * @param  buf - Buffer to receive data
 * @param  buf_len_max - Maximum data to receive
 *
 * @return nb data received
 */
uint16_t UART3_rx(uint8_t* buf, int buf_len_max)
{
	uint16_t len = 0;

	while( R8_UART3_RFC )
	{
		if(len < buf_len_max)
		{
			*buf++ = R8_UART3_RBR;
			len++;
		}
		else
		{
			/* Not enough space in buffer exit loop */
			break;
		}
	}
	return (len);
}

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding) {
	if (coding->parity)
//		uart_set_databits(UART2, (coding->databits + 1 <= 8 ? 8 : 9));
//	else
//		uart_set_databits(UART2, (coding->databits <= 8 ? 8 : 9));
/*
	switch(coding->stopbits) {
	case 0:
		uart_set_stopbits(UART2, UART_STOPBITS_1);
		break;
	case 1:
	default:
		uart_set_stopbits(UART2, UART_STOPBITS_2);
		break;
	}
*/
	switch(coding->parity) {
	case 0:
		uart_set_parity(UART2, UART_PARITY_CLR);
		break;
	case 1:
		uart_set_parity(UART2, UART_PARITY_ODD);
		break;
	case 2:
	default:
		uart_set_parity(UART2, UART_PARITY_EVEN);
		break;
	}
}

void aux_serial_get_encoding(usb_cdc_line_coding_s *const coding)
{
	coding->bitrate = vitrul_buad;

	switch (uart_get_stopbits(UART2)) {
	case UART_STOPBITS_1:
		coding->stopbits = UART_STOPBITS_1;
		break;

	case UART_STOPBITS_2:
	default:
		coding->stopbits = UART_STOPBITS_2;
		break;
	}

	switch (uart_get_parity(UART2)) {
	case UART_PARITY_CLR:
	default:
		coding->parity = UART_PARITY_CLR;
		break;
	case UART_PARITY_ODD:
		coding->parity = UART_PARITY_ODD;
		break;
	case UART_PARITY_EVEN:
		coding->parity = UART_PARITY_EVEN;
		break;
	}

	const uint32_t data_bits = uart_get_databits(UART2);
	if (coding->parity == UART_PARITY_CLR)
		coding->databits = data_bits;
	else
		coding->databits = data_bits - 1;
}
/*
uint32_t usart_get_parity(uint32_t usart)
{
	const uint32_t reg32 = USART_CR1(usart);
	return reg32 & USART_PARITY_MASK;
}
*/
/*
void uart_set_stopbits(volatile uint32_t uart, uint8_t stopbits)
{
//	volatile uint32_t reg32 = R8_UART_LCR(uart);
//	volatile uint32_t reg32 = (stopbits << 2);
//	R8_UART2_LCR = reg32; 
	
	uint32_t reg32;

	reg32 = POOPUART_LCR((volatile uint32_t)uart);
	reg32 = (reg32 &= ~UART_STOPBITMSK);
	reg32 = (reg32 & (stopbits << 2));
	POOPUART_LCR((volatile uint32_t)uart) = reg32;
	
}
*/
uint32_t uart_get_parity(uint32_t uart)
{
	const uint32_t reg32 = R8_UART_LCR((volatile uint32_t)uart);
	return reg32 & (0x03 << 4);
}

void uart_set_parity(uint32_t uart, uint32_t parity)
{
	uint32_t reg32;
    R8_UART2_LCR |= UART_PARITY_EN;
	reg32 = R8_UART2_LCR;
	reg32 = (reg32 & ~UART_PARITY_CLR) | parity;
	R8_UART2_LCR = reg32;
}


/*
void UART2_set_stopbits(uint8_t stopbits) {
    R8_UART2_LCR |= ((R8_UART2_LCR) & (stopbits << 2));
}

static const char* stop_name[] = {"1", "1.5", "2"};
uint32_t uart_get_stopbits(volatile uint32_t uart)
{
//	const uint32_t reg32 = R8_UART_LCR(uart);
//	cprintf("in get R8_UART_LCR = %02x\n", (reg32 & UART_STOPBITS_1));
//	return reg32 & UART_STOPBITS_1;
	
//	const uint32_t reg32 = R8_UART_LCR(uart);
//	cprintf("in get R8_UART_LCR = %02x\n", (reg32 & UART_STOPBITMSK));
//	return reg32 & UART_STOPBITMSK;
    return (R8_UART2_LCR & (0x02 << 2));
}
*/
/*
uint32_t uart_get_parity(volatile uint32_t uart)
{
	const uint32_t reg32 = R8_UART_LCR((volatile uint32_t)uart);
	return R8_UART2_LCR & UART_PARITY_CLR;
}
*/
void uart_set_databits(uint32_t uart, uint32_t bits)
{
	if (bits == 8) {
		R8_UART_LCR(uart) = R8_UART_LCR(uart) |= (0x03 << 0); 
	} else if (bits == 7) {
		R8_UART_LCR(uart) = R8_UART_LCR(uart) |= (0x02 << 0);  
	} else if (bits == 6) {
	    R8_UART_LCR(uart) = R8_UART_LCR(uart) |= (0x01 << 0);
	} else if (bits == 5) {   
	    R8_UART_LCR(uart) = R8_UART_LCR(uart) |= (0x00 << 0);
	}
}

uint32_t uart_get_databits(uint32_t uart)
{
	const uint32_t reg32 = R8_UART_LCR(uart) & 0x0;
	if (reg32 == 0x03) {
		return 8;
	} else if (reg32 == 0x02) {
		return 7;
	} else if (reg32 == 0x01) {
	    return 6;
	} else if (reg32 == 0x00) {
	    return 5;
	}
}

void uart_set_stopbits(uint32_t uart, uint32_t stopbits)
{
	uint32_t reg32;

	reg32 = R8_UART_LCR(uart);
	reg32 = (reg32 & ~UART_STOPBITMSK) | (stopbits << 2);
	R8_UART_LCR(uart) = reg32;
}

uint32_t uart_get_stopbits(uint32_t uart)
{
	const uint32_t reg32 = R8_UART_LCR(uart);
	return reg32 & UART_STOPBITMSK;
}

/*
void uart_set_databits(volatile uint32_t uart, uint8_t wsize)
{
	volatile uint32_t reg32 = POOPUART_LCR(uart);	
	reg32 |= (wsize << 0);  
	POOPUART_LCR(uart) = (uint8_t)reg32;
}

uint32_t uart_get_databits(volatile uint32_t uart)
{
	const uint32_t reg32 = POOPUART_LCR(uart) & UART_DATABITS_OFFSET;
	if (reg32 == 0x00)
		return 5;
	if (reg32 == 0x01)
		return 6;
	if (reg32 == 0x02)
		return 7;
	if (reg32 == 0x03)
		return 8;
	else
		return 8;
}
*/
void uart_set_break(volatile uint32_t uart, bool enable)
{
    volatile uint32_t reg32 = R8_UART_LCR(uart);	
	if (enable == 1)
		reg32 |= UART_BREAK_EN;  /*enable break */
    else
		reg32 &= ~UART_BREAK_EN;
}
