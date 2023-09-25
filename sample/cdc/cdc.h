/********************************** (C) COPYRIGHT *******************************
* File Name          : cdc.h
* Author             : WCH
* Version            : V1.0
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef CDC_CDC_H_
#define CDC_CDC_H_


extern volatile uint16_t USBByteCount;
extern volatile uint16_t USBBufOutPoint;
extern volatile uint8_t  UploadPoint2_Busy;
extern volatile uint8_t  DownloadPoint2_Busy;
extern volatile uint16_t Uart_Sendlenth;

void CDC_reinit( uint32_t baudrate );
void CDC_Uart_Init( uint32_t baudrate );
void TMR2_TimerInit1( void );
void CDC_Uart_Deal( void );
void CDC_Variable_Clear(void);

#endif /* CDC_CDC_H_ */
