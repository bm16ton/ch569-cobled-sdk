/********************************** (C) COPYRIGHT *******************************
* File Name      : Main.c
* Author       : WCH
* Version      : V1.0
* Date         : 2020/07/31
* Description    :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "CH56x_common.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_usb_devbulk_desc_cmd.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_debug_log.h"
#include "cdc.h"

#define USB_VID_BYTE_MSB (0x16)
#define USB_VID_BYTE_LSB (0xC0)
#define USB_VID ((USB_VID_BYTE_MSB << 8) | USB_VID_BYTE_LSB)

#define USB_PID_BYTE_MSB (0x05)
#define USB_PID_BYTE_LSB (0xDC)
#define USB_PID ((USB_PID_BYTE_MSB << 8) | USB_PID_BYTE_LSB)

extern volatile int linechange;
extern struct usb_cdc_line_coding coding;

#undef FREQ_SYS
/* System clock / MCU frequency in Hz */
#define FREQ_SYS (120000000)
//#define DEBUG 1
#define UART1_BAUD (115200)
//#define DEBUG  Debug_UART1

uint8_t RxBuff[1024];
//void UART2_IRQHandler (void) __attribute__((interrupt()));


/*******************************************************************************
 * @fn     DebugInit
 *
 * @brief  Initializes the UART1 peripheral.
 *
 * @param  baudrate: UART1 communication baud rate.
 *
 * @return   None
 */



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


/*********************************************************************
 * @fn    main
 *
 * @brief   Main program.
 *
 * @return  none
 */


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

  SystemInit(FREQ_SYS);
  Delay_Init(FREQ_SYS);

  DebugInit(115200);
  #if(defined DEBUG)
  /* Configure serial debugging for printf()/log_printf()... */
  UART1_init(UART1_BAUD, FREQ_SYS);
#endif
  printf("Start @ChipID=%02X\r\n", R8_CHIP_ID);

  TMR2_TimerInit1();
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
  GPIOA_ModeCfg(GPIO_Pin_2, GPIO_ModeIN_PU_NSMT);    // RXD
  GPIOA_ModeCfg(GPIO_Pin_3, GPIO_Slowascent_PP_8mA);  // TXD

  UART2_init(115200, FREQ_SYS);
  UART2_ByteTrigCfg( UART_7BYTE_TRIG );
  trigB = 7;

  DelayMs(1);

  while (1) {
    CDC_Uart_Deal();

 }
}





