/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : bvernoux
* Version            : V1.0.1
* Date               : 2022/08/22
* Description        : Example USB Device Bulk
*                      For USB2 High Speed and USB3 Super Speed(with Burst)
*                      Support Windows Compatible ID(USB WCID)
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_debug_log.h"

#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_usb_devbulk_desc_cmd.h"

#include "hydrausb3_usb_devbulk_vid_pid.h"

#undef FREQ_SYS
/* System clock / MCU frequency in Hz */
#define FREQ_SYS (120000000)

// Note: DEBUG over UART is not required as all logs can be retrieved from USB2/USB3 with USB_CMD_LOGR
#if(defined DEBUG) // DEBUG=1 to be defined in in Makefile DEFINE_OPTS (Example DEFINE_OPTS = -DDEBUG=1)
//#define UART1_BAUD (115200)
//#define UART1_BAUD (921600)
//#define UART1_BAUD (3000000) // Real baud rate 3Mbauds(For Fsys 96MHz or 120MHz) => Requires USB2HS Serial like FTDI C232HM-DDHSL-0
#define UART1_BAUD (5000000) // Real baud rate is round to 5Mbauds (For Fsys 120MHz) => Requires USB2HS Serial like FTDI C232HM-DDHSL-0
#endif

/* Blink time in ms */
#define BLINK_FAST (50) // Blink LED each 100ms (50*2)

#define BLINK_USB3 (250) // Blink LED each 500ms (250*2)
#define BLINK_USB2 (500) // Blink LED each 1000ms (500*2)

void UART2_IRQHandler (void) __attribute__((interrupt()));

int blink_ms = BLINK_USB2;

debug_log_buf_t log_buf;

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
uint8_t RxBuff[100];
uint8_t trigB;
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{

	int old_DeviceUsbType = -1;
	/* HydraUSB3 configure GPIO In/Out */
	bsp_gpio_init();

	/* Init BSP (MCU Frequency & SysTick) */
	bsp_init(FREQ_SYS);
	log_init(&log_buf);

#if(defined DEBUG)
	/* Configure serial debugging for printf()/log_printf()... */
//	UART1_init(UART1_BAUD, FREQ_SYS);
#endif
	log_printf("Start\n");
	log_printf("ChipID(Hex)=%02X\n", R8_CHIP_ID);

	memset(&unique_id, 0, 8);
	FLASH_ROMA_READ(FLASH_ROMA_UID_ADDR, (uint32_t*)&unique_id, 8);
	log_printf("FLASH_ROMA_UID(Hex)=%02X %02X %02X %02X %02X %02X %02X %02X\n",
			   unique_id.sn_8b[0], unique_id.sn_8b[1], unique_id.sn_8b[2], unique_id.sn_8b[3],
			   unique_id.sn_8b[4], unique_id.sn_8b[5], unique_id.sn_8b[6], unique_id.sn_8b[7]);

	log_printf("HydraUSB3_USB FW v1.0.1 22-Aug-2022(CPU Freq=%d MHz)\n", (FREQ_SYS/1000000));
	log_printf("DEF_ENDP1_MAX_SIZE=%d DEF_ENDP2_MAX_SIZE=%d\n", DEF_ENDP1_MAX_SIZE, DEF_ENDP2_MAX_SIZE);

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
//	USB3_force();
	//USB2_force();
	//USB20_Device_Init(ENABLE);
	// Infinite loop USB2/USB3 managed with Interrupt
		uint8_t len;


	GPIOA_SetBits(GPIO_Pin_3);
	GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeIN_PU_NSMT);			// RXD-
	GPIOA_ModeCfg(GPIO_Pin_3, GPIO_Slowascent_PP_8mA);		// TXD-
	UART2_init(115200, FREQ_SYS);


#if 1
	UART2_ByteTrigCfg( UART_1BYTE_TRIG );
	trigB = 1;
	UART2_INTCfg( ENABLE, RB_IER_RECV_RDY|RB_IER_LINE_STAT );
	PFIC_EnableIRQ( UART2_IRQn );
#endif

	while(1)
	{
		if( bsp_ubtn() == 0 )
		{
			blink_ms = BLINK_FAST;
			bsp_uled_on();
			bsp_wait_ms_delay(blink_ms);
			bsp_uled_off();
			bsp_wait_ms_delay(blink_ms);
		}
		else
		{
			if(g_DeviceConnectstatus == USB_INT_CONNECT_ENUM)
			{
				switch(g_DeviceUsbType)
				{
					case USB_U20_SPEED: // USB2
					{
						if(g_DeviceUsbType != old_DeviceUsbType)
						{
							old_DeviceUsbType = g_DeviceUsbType;
							log_printf("USB2\n");
						}
						blink_ms = BLINK_USB2;
						bsp_uled_on();
						bsp_wait_ms_delay(blink_ms);
						bsp_uled_off();
						bsp_wait_ms_delay(blink_ms);
					}
					break;

					case USB_U30_SPEED: // USB3
					{
						if(g_DeviceUsbType != old_DeviceUsbType)
						{
							old_DeviceUsbType = g_DeviceUsbType;
							log_printf("USB3\n");
						}
						blink_ms = BLINK_USB3;
						bsp_uled_on();
						bsp_wait_ms_delay(blink_ms);
						bsp_uled_off();
						bsp_wait_ms_delay(blink_ms);
					}
					break;

					default:
						bsp_uled_on(); // LED is steady until USB3 SS or USB2 HS is ready
				}
			}
			else
			{
				bsp_uled_on(); // LED is steady until USB3 SS or USB2 HS is ready
			}
		}
	}
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   Example of basic HardFault Handler called if an exception occurs
 *
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void HardFault_Handler(void)
{
	printf("HardFault\n");
	printf(" SP=0x%08X\n", __get_SP());
	printf(" MIE=0x%08X\n", __get_MIE());
	printf(" MSTATUS=0x%08X\n", __get_MSTATUS());
	printf(" MCAUSE=0x%08X\n", __get_MCAUSE());
	bsp_wait_ms_delay(1);
}

void UART2_IRQHandler(void)
{
	uint8_t i;
	switch( UART2_GetITFlag() )
	{
		case UART_II_LINE_STAT:
			PRINT("UART2_GetLinSTA()\r\n",UART2_GetLinSTA());
			break;

		case UART_II_RECV_RDY:
//			for(i=0; i!=trigB; i++)
//			{
//				RxBuff[i] = UART2_RecvByte();
//				memcpy(endp1Tbuff, &RxBuff[i], 1/*DEF_ENDP1_MAX_SIZE*/);
//				R16_UEP1_T_LEN = 1;
//            	R8_UEP1_TX_CTRL ^= RB_UEP_T_TOG_1;
//            	R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
		//		UART2_SendByte(RxBuff[i]);
//			}

//			break;

//		case UART_II_RECV_TOUT:
//			i = UART2_RecvString(RxBuff);
//			memcpy(endp1Tbuff, RxBuff, 100);
//			R16_UEP1_T_LEN = 1;
//            R8_UEP1_TX_CTRL ^= RB_UEP_T_TOG_1;
//            R8_UEP1_TX_CTRL = (R8_UEP1_TX_CTRL & ~RB_UEP_TRES_MASK) | UEP_T_RES_ACK;
		//	UART2_SendString( RxBuff, i );
//			break;

		case UART_II_THR_EMPTY:
			break;

		case UART_II_MODEM_CHG:
			break;

		default:
			break;
	}
}
