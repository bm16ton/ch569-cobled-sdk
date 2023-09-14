/********************************** (C) COPYRIGHT *******************************
* File Name      : Main.c
* Author       : WCH
* Version      : V1.0
* Date         : 2020/07/31
* Description    :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include <math.h>
#include "CH56x_common.h"
#include "tft.h"
#include "fonts/bitmap_typedefs.h"
#include "fonts/16ton.h"
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
extern int lcddma;
extern volatile int linechange;
extern struct usb_cdc_line_coding coding;
uint8_t spiBuff[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
uint8_t spiBuffrev[16];

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

 #define d2r(d) ((d) * 6.2831853 / 360.0)

  int p1 = 0;
  int p2 = 45; 
  int p3 = 90;
  int pp1 = 0;
  int pp2 = 45;
  int pp3 = 90;

void planets(void)
{
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

  _write_data_8bit(0x55);       // 16 bit colour interface


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
  _write_command_8bit(0x29);           // display on
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

  lcddma = 1;
  SystemInit(FREQ_SYS);
  Delay_Init(FREQ_SYS);
struct usb_cdc_line_coding coding;
  DebugInit(115200);
  #if(defined DEBUG)
  /* Configure serial debugging for printf()/log_printf()... */
  UART1_init(UART1_BAUD, FREQ_SYS);
#endif
  printf("Start @ChipID=%02X\r\n", R8_CHIP_ID);


  printf("1.spi1 mul master mode send data ...\n");
  DelayMs(100);


  CS_IDLE;

  R32_PB_PD &= ~(1 << 10 | 1 << 11 | 1 << 12 | 1 << 13);
  R32_PB_DRV &= ~(1 << 10 | 1 << 11 | 1 << 12 | 1 << 13);
  R32_PB_DIR |= (1 << 10 | 1 << 11 | 1 << 12 | 1 << 13);

  SPI1_MasterDefInit();
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

#ifdef USE_CS
  CS_ACTIVE;
#endif

  tftinit();
#ifdef USE_CS
  CS_IDLE;
#endif
  DelayMs(1);
  st_fill_screen(ST_COLOR_YELLOW);

  st_draw_bitmap(1, 10, &b16ton);
  DelayMs(5000);

  st_fill_screen(ST_COLOR_RED);
  st_fill_circle(120, 160, 40, ST_COLOR_YELLOW);
  while (1) {
    CDC_Uart_Deal();
    planets();
    if (linechange == 1) {
    R8_UART_LCR(UART2) = R8_UART_LCR(UART2) |= coding.stopbits << 2;
    R8_UART_LCR(UART2) = R8_UART_LCR(UART2) |= coding.databits;
    cprintf("get stopbits = %02x\n", uart_get_stopbits(UART2));
    cprintf("get data bits = %02x\n", uart_get_databits(UART2));
    linechange = 0;
  }
 }
}





