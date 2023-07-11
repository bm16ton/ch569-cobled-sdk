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
  generic_bsp_gpio_init();
  board_init();
  bsp_init(FREQ_SYS);
  log_init(&log_buf);
  Delay_Init(FREQ_SYS);
  UART1_init(UART1_BAUD, FREQ_SYS);
  memset(&unique_id, 0, 8);
  FLASH_ROMA_READ(FLASH_ROMA_UID_ADDR, (uint32_t*)&unique_id, 8);
  TUSB_LOGD("cdcacm device begin\n");
  usb_descriptor_set_string_serial_number(&unique_id);

	/* USB Descriptor set USB VID/PID */
  usb_descriptor_set_usb_vid_pid(&vid_pid);
  SetDescriptor(&g_dev, &BULK30_descriptors);
  tusb_set_device_config(&g_dev, &device_config);
  tusb_open_device(&g_dev, (const tusb_hardware_param_t*)&BULK20_descriptors);
  TUSB_LOGD("Work in %c Speed mode\n", "HFLS"[tusb_get_device_speed(&g_dev)]);
  GPIOA_SetBits(GPIO_Pin_3);
  GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeIN_PU_NSMT);			// RXD
  GPIOA_ModeCfg(GPIO_Pin_3, GPIO_Slowascent_PP_8mA);		// TXD
  UART2_init(115200, FREQ_SYS);
  UART2_ByteTrigCfg( UART_7BYTE_TRIG );
  trigB = 7;
  UART2_INTCfg( ENABLE, RB_IER_RECV_RDY|RB_IER_LINE_STAT );
  PFIC_EnableIRQ( UART2_IRQn );

  while(1){

						generic_uled_on();
						generic_uled2_off();
						tusb_delay_poo();
						tusb_delay_poo();
						tusb_delay_poo();
						tusb_delay_poo();
						generic_uled_off();
						generic_uled2_on();
						tusb_delay_poo();
						tusb_delay_poo();
						tusb_delay_poo();
						tusb_delay_poo();

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
