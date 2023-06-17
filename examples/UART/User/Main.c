/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/

//#define  FREQ_SYS   80000000
#undef FREQ_SYS
/* System clock / MCU frequency in Hz */
#define FREQ_SYS (120000000)
#include "CH56x_common.h"
#include "CH56x_usb_devbulk_desc_cmd.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_usb_devbulk_desc_cmd.h"

#include "hydrausb3_usb_devbulk_vid_pid.h"
/* FLASH_ROMA Read Unique ID (8bytes/64bits) */
#define FLASH_ROMA_UID_ADDR (0x77fe4)
usb_descriptor_serial_number_t unique_id;

/* USB VID PID */
usb_descriptor_usb_vid_pid_t vid_pid =
{
	.vid = USB_VID,
	.pid = USB_PID
};


uint8_t TxBuff[]="This is a Tx exam\r\n";
uint8_t RxBuff[1024];
uint8_t trigB;

void UART2_IRQHandler (void) __attribute__((interrupt()));
/*******************************************************************************
 * @fn       DebugInit
 *
 * @brief    Initializes the UART1 peripheral.
 *
 * @param    baudrate: UART1 communication baud rate.
 *
 * @return   None
 */
void DebugInit(uint32_t baudrate)
{
	uint32_t x;
	uint32_t t = FREQ_SYS;

	x = 10 * t * 2 / 16 / baudrate;
	x = ( x + 5 ) / 10;
	R8_UART1_DIV = 1;
	R16_UART1_DL = x;
	R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	R8_UART1_LCR = RB_LCR_WORD_SZ;
	R8_UART1_IER = RB_IER_TXD_EN;
	R32_PA_SMT |= (1<<8) |(1<<7);
	R32_PA_DIR |= (1<<8);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{
//	SystemInit(FREQ_SYS);
	Delay_Init(FREQ_SYS);

	bsp_gpio_init();

	/* Init BSP (MCU Frequency & SysTick) */
	bsp_init(FREQ_SYS);

	memset(&unique_id, 0, 8);
	uint8_t len;


	GPIOA_SetBits(GPIO_Pin_3);
	GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeIN_PU_NSMT);			// RXD-
	GPIOA_ModeCfg(GPIO_Pin_3, GPIO_Slowascent_PP_8mA);		// TXD-
	UART2_DefInit();

#if 0
	UART2_SendString( TxBuff, sizeof(TxBuff) );

#endif

#if 0
	while(1)
	{
		len = UART2_RecvString(RxBuff);
		if( len )
		{
			UART2_SendString( RxBuff, len );
		}
	}

#endif

#if 1
	UART2_ByteTrigCfg( UART_7BYTE_TRIG );
	trigB = 7;
	UART2_INTCfg( ENABLE, RB_IER_RECV_RDY|RB_IER_LINE_STAT );
	PFIC_EnableIRQ( UART2_IRQn );
//	DebugInit(115200);
	PRINT("Start @ChipID=%02X\r\n", R8_CHIP_ID );
	FLASH_ROMA_READ(FLASH_ROMA_UID_ADDR, (uint32_t*)&unique_id, 8);

	// USB2 & USB3 Init
	// USB2 & USB3 are managed in LINK_IRQHandler()/TMR0_IRQHandler()/USBHS_IRQHandler()/USBSS_IRQHandler()
	R32_USB_CONTROL = 0;
	PFIC_EnableIRQ(USBSS_IRQn);
	PFIC_EnableIRQ(LINK_IRQn);

	PFIC_EnableIRQ(TMR0_IRQn);
	R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
	TMR0_TimerInit(67000000); // USB3.0 connection failure timeout about 0.56 seconds

	/* USB Descriptor set String Serial Number with CH569 Unique ID */
	usb_descriptor_set_string_serial_number(&unique_id);

	/* USB Descriptor set USB VID/PID */
	usb_descriptor_set_usb_vid_pid(&vid_pid);

	/* USB3.0 initialization, make sure that the two USB3.0 interrupts are enabled before initialization */
	USB30D_init(ENABLE);
//USB2_force();

#endif

	while(1)
	{
//		PRINT("11111111\r\n");
//		mDelaymS(1000);
		;
	}
}



/*******************************************************************************
 * @fn       UART1_IRQHandler
 *
 * @brief    Interruption function
 *
 * @return   None
 */
void UART2_IRQHandler(void)
{
	uint8_t i;
	switch( UART2_GetITFlag() )
	{
		case UART_II_LINE_STAT:
			PRINT("UART2_GetLinSTA()\r\n",UART2_GetLinSTA());
			break;

		case UART_II_RECV_RDY:
			for(i=0; i!=trigB; i++)
			{
				RxBuff[i] = UART2_RecvByte();
				memcpy(endp1Tbuff, &RxBuff[i], 1/*DEF_ENDP1_MAX_SIZE*/);
				R16_UEP1_T_LEN = 1;
            	R8_UEP1_TX_CTRL ^= RB_UEP_T_TOG_1;
            	R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
		//		UART2_SendByte(RxBuff[i]);
			}

			break;

		case UART_II_RECV_TOUT:
			i = UART2_RecvString(RxBuff);
			memcpy(endp1Tbuff, RxBuff, 1/*DEF_ENDP1_MAX_SIZE*/);
			R16_UEP1_T_LEN = 1;
            R8_UEP1_TX_CTRL ^= RB_UEP_T_TOG_1;
            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
		//	UART2_SendString( RxBuff, i );
			break;

		case UART_II_THR_EMPTY:
			break;

		case UART_II_MODEM_CHG:
			break;

		default:
			break;
	}
}
