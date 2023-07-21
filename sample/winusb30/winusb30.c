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
 * Copyright (c) 2021 XToolBox  - admin@xtoolbox.org
 *                         www.tusb.org
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

#include "teeny_usb.h"
#include "teeny_usb_desc.h"
#include "teeny_usb_util.h"
#include "string.h"
#include "CH56x_common.h"
#include "CH56x_debug_log.h"
#include "tusbd_cdc.h"
#include "tusb_cdc.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "winusb30.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_usb_devbulk_desc_cmd.h"
#include "generic-devboard.h"

#define  TX_EP   1
#define  RX_EP   1
#define CDC_RX_EP_SIZE    1024
#undef FREQ_SYS
/* System clock / MCU frequency in Hz */
#define FREQ_SYS (120000000)
//#define DEBUG 1
#define UART1_BAUD (5000000)
//#define DEBUG  Debug_UART1

uint8_t RxBuff[1024];

void UART2_IRQHandler (void) __attribute__((interrupt()));

debug_log_buf_t log_buf;

extern __attribute__((aligned(16))) uint8_t endp1Tbuff[DEF_ENDP1_MAX_SIZE] __attribute__((section(".DMADATA"))); // Endpoint 1 data Transmit buffer
extern __attribute__((aligned(16))) uint8_t endp1Rbuff[DEF_ENDP1_MAX_SIZE] __attribute__((section(".DMADATA"))); // Endpoint 1 data Transmit buffer
int cdc_recv_data(tusb_cdc_device_t* cdc, const void* data, uint16_t len);
int cdc_send_done(tusb_cdc_device_t* cdc, const void* data, uint16_t len);
void cdc_line_coding_change(tusb_cdc_device_t* cdc);
volatile int linecode = 0;


tusb_cdc_device_t cdc_dev = {
  .backend = &cdc_device_backend,
  .ep_in = 1,
  .ep_out = 1,
  .ep_int = 2,
  .on_recv_data = cdc_recv_data,
  .on_send_done = cdc_send_done,
  .on_line_coding_change = cdc_line_coding_change,
  .rx_buf = endp1Rbuff,
  .rx_size = sizeof(endp1Rbuff),
};

#define FLASH_ROMA_UID_ADDR (0x77fe4)
usb_descriptor_serial_number_t unique_id;

/* USB VID PID */
usb_descriptor_usb_vid_pid_t vid_pid =
{
	.vid = USB_VID,
	.pid = USB_PID
};

void tusb_delay_ms(uint32_t ms)
{
  uint32_t i,j;
  for(i=0;i<ms;++i)
    for(j=0;j<200;++j);
}

void tusb_delay_poo(void)
{
  uint32_t i;
  for(i=0;i<1515580;++i) {
   __NOP();
   	}
}


static int cdc_len = 0;
int cdc_recv_data(tusb_cdc_device_t* cdc, const void* data, uint16_t len)
{
  cdc_len = (int)len;
  return 1; // return 1 means the recv buffer is busy
}

int cdc_send_done(tusb_cdc_device_t* cdc, const void* data, uint16_t len)
{
  tusb_set_rx_valid(cdc->dev, cdc->ep_out);

  return 0;
}


void cdc_line_coding_change(tusb_cdc_device_t* cdc)
{
//TUSB_LOGD("from change (struct) parity %d stopbits\n", cdc->line_coding.parity, cdc->line_coding.stopbits);
//    cdc->line_coding.bitrate
//UART1_BaudRateCfg(cdc->line_coding.bitrate);
//    cdc->line_coding.parity
//uart_set_parity(UART1, cdc->line_coding.parity);
//    cdc->line_coding.databits
//uart_set_wordsize(UART1, cdc->line_coding.databits);
 //   cdc->line_coding.stopbits
//cdc_uart_set_stopbits(UART1, cdc->line_coding.stopbits);
cprintf("from change struct bitrate %d parity %d stopbits\n",cdc->line_coding.bitrate, cdc->line_coding.parity, cdc->line_coding.stopbits);
}

/* Blink time in ms */
#define BLINK_FAST (50) // Blink LED each 100ms (50*2)

#define BLINK_USB3 (250) // Blink LED each 500ms (250*2)
#define BLINK_USB2 (500) // Blink LED each 1000ms (500*2)

int blink_ms = BLINK_USB2;

static tusb_device_interface_t* device_interfaces[] = {
  (tusb_device_interface_t*)&cdc_dev, 0,   // CDC need two interfaces
};

tusb_device_config_t device_config = {
  .if_count = sizeof(device_interfaces)/sizeof(device_interfaces[0]),
  .interfaces = &device_interfaces[0],
};

volatile uint8_t trigB = 7;
static tusb_device_t g_dev;
void board_init(void);
int main(void)
{
	int old_DeviceUsbType = -1;
	/* HydraUSB3 configure GPIO In/Out */
	generic_bsp_gpio_init();

	/* Init BSP (MCU Frequency & SysTick) */
	bsp_init(FREQ_SYS);
	log_init(&log_buf);

#if(defined DEBUG)
	/* Configure serial debugging for printf()/log_printf()... */
	UART1_init(UART1_BAUD, FREQ_SYS);
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
  GPIOA_SetBits(GPIO_Pin_3);
  GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeIN_PU_NSMT);			// RXD
  GPIOA_ModeCfg(GPIO_Pin_3, GPIO_Slowascent_PP_8mA);		// TXD
  UART2_init(115200, FREQ_SYS);
  UART2_ByteTrigCfg( UART_7BYTE_TRIG );
  trigB = 7;
  UART2_INTCfg( ENABLE, RB_IER_RECV_RDY|RB_IER_LINE_STAT );
  PFIC_EnableIRQ( UART2_IRQn );
	// Infinite loop USB2/USB3 managed with Interrupt
	while(1)
	{
		if( bsp_ubtn() )
		{
			blink_ms = BLINK_FAST;
			generic_uled_on();
			bsp_wait_ms_delay(blink_ms);
			generic_uled_off();
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
						generic_uled_on();
						bsp_wait_ms_delay(blink_ms);
						generic_uled_off();
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
						generic_uled_on();
						bsp_wait_ms_delay(blink_ms);
						generic_uled_off();
						bsp_wait_ms_delay(blink_ms);
					}
					break;

					default:
						generic_uled_on(); // LED is steady until USB3 SS or USB2 HS is ready
				}
			}
			else
			{
				generic_uled_on(); // LED is steady until USB3 SS or USB2 HS is ready
			}
		}
	}
}

uint16_t placem = 0;

void UART2_IRQHandler(void)
{
	uint8_t i;


	switch( UART2_GetITFlag() )
	{
		case UART_II_LINE_STAT:
			cprintf("UART2_GetLinSTA()\r\n",UART2_GetLinSTA());
			break;

		case UART_II_RECV_RDY:
			for(i=0; i!=trigB; i++)
			{

				RxBuff[placem + i] = UART2_RecvByte();
//				UART2_SendByte(RxBuff[placem + i]);
			}
			cprintf("uart size = %d\n", placem + i);
			placem = placem + i;
//		memcpy(endp1Tbuff, RxBuff, placem);
//		USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
//		USB30_IN_clearIT(ENDP_1); // Clear endpoint state Keep only packet sequence number
//		USB30_IN_set(ENDP_1, DISABLE, NRDY, 0, 0);
//		USB30_IN_set(ENDP_1, ENABLE, ACK, 1, 0); // Set the endpoint to be able to send 4 packets
//		USB30_send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
			break;

		case UART_II_RECV_TOUT:
			i = UART2_RecvString(&RxBuff[placem]);
//			UART2_SendString( RxBuff, i );
			cprintf("uart string size = %d\n", i);
			placem = (placem + i);
//		memcpy(endp1Tbuff, RxBuff, placem);
//		USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
//		USB30_IN_clearIT(ENDP_1); // Clear endpoint state Keep only packet sequence number
//		USB30_IN_set(ENDP_1, DISABLE, NRDY, 0, 0);
//		USB30_IN_set(ENDP_1, ENABLE, ACK, 1, 0); // Set the endpoint to be able to send 4 packets
//		USB30_send_ERDY(ENDP_1 | IN, DEF_ENDP1_IN_BURST_LEVEL);
			break;
		case UART_II_THR_EMPTY:
			break;

		case UART_II_MODEM_CHG:
			break;

		default:
			break;
	}
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void HardFault_Handler(void)
{
	printf("HardFault\n");
	printf(" SP=0x%08X\n", __get_SP());
	printf(" MIE=0x%08X\n", __get_MIE());
	printf(" MSTATUS=0x%08X\n", __get_MSTATUS());
	printf(" MCAUSE=0x%08X\n", __get_MCAUSE());
	bsp_wait_ms_delay(1);
}
