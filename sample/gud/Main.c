/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.c
* Author             : WCH
* Version            : V1.0
* Date               : 2020/07/31
* Description 		 :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include <math.h>
#include "CH56x_common.h"
#include "tft.h"
#include "fonts/bitmap_typedefs.h"
#include "fonts/16ton.h"
//#define FREQ_SYS    80000000
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_usb_devbulk_desc_cmd.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_debug_log.h"


#define USB_VID_BYTE_MSB (0x16)
#define USB_VID_BYTE_LSB (0xC0)
#define USB_VID ((USB_VID_BYTE_MSB << 8) | USB_VID_BYTE_LSB)

#define USB_PID_BYTE_MSB (0x05)
#define USB_PID_BYTE_LSB (0xDC)
#define USB_PID ((USB_PID_BYTE_MSB << 8) | USB_PID_BYTE_LSB)
extern int lcddma;

uint8_t spiBuff[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
uint8_t spiBuffrev[16];

#undef FREQ_SYS
/* System clock / MCU frequency in Hz */
#define FREQ_SYS (120000000)
//#define DEBUG 1
#define UART1_BAUD (5000000)
//#define DEBUG  Debug_UART1

uint8_t RxBuff[1024];
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

 #define d2r(d) ((d) * 6.2831853 / 360.0)

void planets(void)
{
    int p1, p2, p3, pp1, pp2, pp3;
	p1 = 0;
	p2 = 45;
	p3 = 90;
	pp1 = 0;
	pp2 = 45;
	pp3 = 90;
	st_fill_screen(ST_COLOR_RED);
	st_fill_circle(120, 160, 40, ST_COLOR_YELLOW);
	while (1) {

//	    st_fill_rect(1, 40, 270, 235, ST_COLOR_RED);

//	st_fill_screen(ST_COLOR_RED);
//		   st_draw_string(15, 36, "PLANETS!", ST_COLOR_BLACK, &font_fixedsys_mono_24);
//		st_fill_circle(120, 160, 40, ST_COLOR_YELLOW);
//		st_draw_circle(120, 160, 55, ST_COLOR_LIGHTGREY);
//		st_draw_circle(120, 160, 75, ST_COLOR_LIGHTGREY);
//		st_draw_circle(120, 160, 100, ST_COLOR_LIGHTGREY);
//		st_draw_bitmap(1, 1, &extfirm);

		pp1 = (p1 - 3) % 360;
		pp2 = (p2 - 2) % 360;
		pp3 = (p3 - 1) % 360;

        st_fill_circle(120 + (sin(d2r(pp1)) * 55),
			       160 + (cos(d2r(pp1)) * 55), 5, ST_COLOR_RED);
		st_fill_circle(120 + (sin(d2r(pp2)) * 75),
			       160 + (cos(d2r(pp2)) * 75), 10, ST_COLOR_RED);
		st_fill_circle(120 + (sin(d2r(pp3)) * 100),
			       160 + (cos(d2r(pp3)) * 100), 8, ST_COLOR_RED);

		st_fill_circle(120 + (sin(d2r(p1)) * 55),
			       160 + (cos(d2r(p1)) * 55), 5, ST_COLOR_PURPLE);
		st_fill_circle(120 + (sin(d2r(p2)) * 75),
			       160 + (cos(d2r(p2)) * 75), 10, ST_COLOR_BLACK);
		st_fill_circle(120 + (sin(d2r(p3)) * 100),
			       160 + (cos(d2r(p3)) * 100), 8, ST_COLOR_DARKGREEN);
		p1 = (p1 + 3) % 360;
		p2 = (p2 + 2) % 360;
		p3 = (p3 + 1) % 360;
	}
}

void DebugInit(uint32_t  baudrate)
{
    uint32_t  x;
    uint32_t  t = FREQ_SYS;

    x = 10 * t * 2 / 16 / baudrate;
    x = (x + 5) / 10;
    R8_UART1_DIV = 1;
    R16_UART1_DL = x;
    R8_UART1_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART1_LCR = RB_LCR_WORD_SZ;
    R8_UART1_IER = RB_IER_TXD_EN;
    R32_PA_SMT |= (1 << 8) | (1 << 7);
    R32_PA_DIR |= (1 << 8);
}
/*
void DC_CMD (void) {
R32_PB_CLR |= 1 << 10;  //dc pb10 low
}

void DC_DATA (void) {
R32_PB_OUT |= 1 << 10;  //dc pb10 high
}

void CS_IDLE (void) {
R32_PB_OUT |= 1 << 11;  //cs pb11 high
}

void CS_ACTIVE (void) {
R32_PB_CLR |= 1 << 11;  //cs pb11 low
}
*/
/*
__attribute__((always_inline)) static inline void _write_command_8bit(uint8_t cmd)
{

	CS_ACTIVE();
	DC_CMD();
    SPI1_MasterSendByte(cmd);
	CS_IDLE();
	DC_DATA();
}

__attribute__((always_inline)) static inline void _write_data_8bit(uint8_t dat)
{

	CS_ACTIVE();
	DC_DATA();
	SPI1_MasterSendByte(dat);
	CS_IDLE();

}
*/
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */

void tftinit(void) {
CS_ACTIVE; //always use even if use_cs not set

	// Hardwae reset is not mandatory if software rest is done
#ifdef ST_HAS_RST
		ST_RST_ACTIVE;
		DelayMs(100);
		ST_RST_IDLE;
#endif
#ifdef USE_CS
		CS_ACTIVE;
#endif
    DelayMs(100);

    _write_command_8bit(0x01); // SW reset

    DelayMs(100);
    _write_command_8bit(0x11); // Sleep out, also SW reset
    DelayMs(100);

    _write_command_8bit(0x3A);

      _write_data_8bit(0x55);           // 16 bit colour interface


    _write_command_8bit(0xC2);
    _write_data_8bit(0x44);

    _write_command_8bit(0xC5);
    _write_data_8bit(0x00);
    _write_data_8bit(0x00);
    _write_data_8bit(0x00);
    _write_data_8bit(0x00);

    _write_command_8bit(0xE0);
    _write_data_8bit(0x0F);
    _write_data_8bit(0x1F);
    _write_data_8bit(0x1C);
    _write_data_8bit(0x0C);
    _write_data_8bit(0x0F);
    _write_data_8bit(0x08);
    _write_data_8bit(0x48);
    _write_data_8bit(0x98);
    _write_data_8bit(0x37);
    _write_data_8bit(0x0A);
    _write_data_8bit(0x13);
    _write_data_8bit(0x04);
    _write_data_8bit(0x11);
    _write_data_8bit(0x0D);
    _write_data_8bit(0x00);

    _write_command_8bit(0xE1);
    _write_data_8bit(0x0F);
    _write_data_8bit(0x32);
    _write_data_8bit(0x2E);
    _write_data_8bit(0x0B);
    _write_data_8bit(0x0D);
    _write_data_8bit(0x05);
    _write_data_8bit(0x47);
    _write_data_8bit(0x75);
    _write_data_8bit(0x37);
    _write_data_8bit(0x06);
    _write_data_8bit(0x10);
    _write_data_8bit(0x03);
    _write_data_8bit(0x24);
    _write_data_8bit(0x20);
    _write_data_8bit(0x00);

    _write_command_8bit(ST7789_INVOFF);

    _write_command_8bit(0x36);
    _write_data_8bit(0x28);
    _write_data_8bit(0x00);
    _write_command_8bit(0x29);                     // display on
    CS_IDLE; //always use even if use_cs not set
    DC_DATA;
    DelayMs(100);
    CS_ACTIVE;  //always use even if use_cs not set
    DelayMs(100);

}


#define FLASH_ROMA_UID_ADDR (0x77fe4)
usb_descriptor_serial_number_t unique_id;

/* USB VID PID */
usb_descriptor_usb_vid_pid_t vid_pid =
{
	.vid = USB_VID,
	.pid = USB_PID
};


volatile uint8_t trigB = 7;

int main()
{
    uint8_t i;
	lcddma = 1;
    SystemInit(FREQ_SYS);
    Delay_Init(FREQ_SYS);

    DebugInit(115200);
    #if(defined DEBUG)
	/* Configure serial debugging for printf()/log_printf()... */
	UART1_init(UART1_BAUD, FREQ_SYS);
#endif
    printf("Start @ChipID=%02X\r\n", R8_CHIP_ID);


    printf("1.spi1 mul master mode send data ...\n");
    DelayMs(100);

//    R32_PB_OUT |= 1 << 11;
	CS_IDLE;

    R32_PB_PD &= ~(1 << 10 | 1 << 11 | 1 << 12 | 1 << 13);
    R32_PB_DRV &= ~(1 << 10 | 1 << 11 | 1 << 12 | 1 << 13);
    R32_PB_DIR |= (1 << 10 | 1 << 11 | 1 << 12 | 1 << 13);

    SPI1_MasterDefInit();
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

	  GPIOA_SetBits(GPIO_Pin_3);
  GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeIN_PU_NSMT);			// RXD
  GPIOA_ModeCfg(GPIO_Pin_3, GPIO_Slowascent_PP_8mA);		// TXD

  UART2_init(115200, FREQ_SYS);
  UART2_ByteTrigCfg( UART_7BYTE_TRIG );
  trigB = 7;
  UART2_INTCfg( ENABLE, RB_IER_RECV_RDY|RB_IER_LINE_STAT );
  PFIC_EnableIRQ( UART2_IRQn );
//    R32_PB_CLR |= 1 << 11;  //cs low
#ifdef USE_CS
	CS_ACTIVE;
#endif

//    SPI1_MasterSendByte(0x55);
//	_write_data_8bit(0x55);
//    R32_PB_OUT |= 1 << 11; // cs high
	tftinit();
#ifdef USE_CS
	CS_IDLE;
#endif
    DelayMs(1);
st_fill_screen(ST_COLOR_YELLOW);
    // FIFO
//    R32_PB_CLR |= 1 << 11;
//	CS_ACTIVE;

//    SPI1_MasterTrans(spiBuff, 9);

//    R32_PB_OUT |= 1 << 11;
//	CS_IDLE;
st_draw_bitmap(1, 10, &b16ton);
    DelayMs(5000);

planets();
    while(1);
}

void UART2_IRQHandler(void)
{
	uint8_t i;


	switch( UART2_GetITFlag() )
	{
		case UART_II_LINE_STAT:
			cprintf("UART2_GetLinSTA()\r\n",UART2_GetLinSTA());
			break;

		case UART_II_RECV_RDY:
//			bsp_disable_interrupt();

			for(i=0; i!=trigB; i++)
			{

				RxBuff[i] = UART2_RecvByte();
				UART2_SendByte(RxBuff[i]);
			}
//			scdc.size = uartsend;
//			scdc.dataready = 1;
			cprintf("uart size = %d\n", i);
			USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
			memcpy(endp1Tbuff, RxBuff, i);

			USB30_IN_set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, 1024);
        	USB30_send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
//			print_size1 = sprintf(test_buf, "0x%08X ", i);
//			memcpy(&endp1Tbuff, test_buf, print_size1);
//			USB30_send_ERDY(ENDP_1 | IN, 1);
//			USBSS_IRQHandler();
			break;

		case UART_II_RECV_TOUT:
//			bsp_disable_interrupt();
			i = UART2_RecvString(RxBuff);
			UART2_SendString( RxBuff, i );
			cprintf("uart string size = %d\n", i);
//			scdc.dataready = 1;
			USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
			memcpy(endp1Tbuff, RxBuff, i);

			USB30_IN_set(ENDP_1, ENABLE, ACK, DEF_ENDP1_IN_BURST_LEVEL, 1024);
        	USB30_send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
//			USBSS_IRQHandler();
//			print_size1 = sprintf(test_buf, "0x%08X ", i);
//			memcpy(&endp1Tbuff, test_buf, print_size1);
//			USB30_send_ERDY(ENDP_1 | IN, 1);
			break;

		case UART_II_THR_EMPTY:
			break;

		case UART_II_MODEM_CHG:
			break;

		default:
			break;
	}
}

