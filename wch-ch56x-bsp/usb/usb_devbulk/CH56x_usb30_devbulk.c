/********************************** (C) COPYRIGHT *******************************
* File Name    : CH56x_usb30_devbulk.c
* Author     : WCH, bvernoux
* Version    : V1.1.1
* Date     : 2022/12/11
* Description  :
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_debug_log.h"

#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_usb_devbulk_desc_cmd.h"
#include "cdc.h"

vuint32_t vitrul_buad  = 115200;
//struct usb_cdc_line_coding coding;
//#include "usb_cdc.h"
//#define DEBUG_USB3_REQ 1 // Debug USB Req (have impact on real-time to correctly enumerate USB 3.0..)
//#define DEBUG_USB3_EP0 1 // Debug EP0 (have impact on real-time to correctly enumerate USB 3.0..)
//#define DEBUG_USB3_EPX 1 // Debug EP1 to EP7

/* Global define */
/* Global Variable */
vuint8_t tx_lmp_port = 0;
vuint8_t link_sta = 0;
static uint32_t SetupLen = 0;
static uint8_t  SetupReqCode = 0;
static uint8_t *pDescr;

vuint8_t g_DeviceConnectstatus = 0;
vuint8_t g_DeviceUsbType = 0;

__attribute__((aligned(16))) uint8_t endp0RTbuff[512] __attribute__((section(".DMADATA"))); // Endpoint 0 data transceiver buffer
__attribute__((aligned(16))) uint8_t endp1Tbuff[DEF_ENDP1_MAX_SIZE] __attribute__((section(".DMADATA"))); // Endpoint 1 data Transmit buffer
__attribute__((aligned(16))) uint8_t endp2RTbuff[DEF_ENDP2_MAX_SIZE] __attribute__((section(".DMADATA"))); // Endpoint 2 data transceiver buffer
__attribute__((aligned(16))) uint8_t endp2Txbuff[DEF_ENDP2_MAX_SIZE] __attribute__((section(".DMADATA"))); // Endpoint 2 data transceiver buffer
/*******************************************************************************
 * @fn   USB3_force
 *
 * @brief  Switch to USB3 or do a fallback to USB2 if not available
 *
 * @return None
 */
void USB3_force(void)
{
  USB20_Device_Init(DISABLE);

  // USB2 & USB3 Init
  // USB2 & USB3 are managed in LINK_IRQHandler/TMR0_IRQHandler/USBHS_IRQHandler
  R32_USB_CONTROL = 0;
  PFIC_EnableIRQ(USBSS_IRQn);
  PFIC_EnableIRQ(LINK_IRQn);

  PFIC_EnableIRQ(TMR0_IRQn);
  R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
  TMR0_TimerInit(67000000); // USB3.0 connection failure timeout about 0.56 seconds

  USB30D_init(ENABLE); // USB3.0 initialization, make sure that the two USB3.0 interrupts are enabled before initialization
}

/*******************************************************************************
 * @fn   USB30_bus_reset
 *
 * @brief  USB3.0 bus reset, select the reset method in the code,
 *   it is recommended to use the method of only resetting the USB,
 *   if there is a problem with the connection,
 *   you can use the method of resetting the MCU
 *
 * @return None
 */
void USB30_BusReset(void)
{
  USB30D_init(DISABLE); //USB3.0 initialization
  bsp_wait_ms_delay(20);
  USB30D_init(ENABLE); //USB3.0 initialization
}


static int USB30_device_init(void)
{
  USBSS->LINK_CFG = 0x140;
  USBSS->LINK_CTRL = 0x12;
  uint32_t t = 0x4c4b41;
  while(USBSS->LINK_STATUS&4)
  {
  t--;
  if(t == 0)
  return -1;
  }
  for(int i = 0; i < 8; i++)
  {
  SS_TX_CONTRL(i) = 0;
  SS_RX_CONTRL(i) = 0;
  }
  USBSS->USB_STATUS = 0x13;

  USBSS->USB_CONTROL = 0x30021;
  USBSS->UEP_CFG = 0;

  USBSS->LINK_CFG |= 2;

  USBSS->LINK_INT_CTRL = 0x10bc7d;

  USBSS->LINK_CTRL = 2;
  return 0;
}

/*******************************************************************************
 * @fn   USB30D_init
 *
 * @brief  USB3.0 device initialization
 *
 * @return None
 */
void USB30D_init(FunctionalState sta)
{
  int s;
  if(sta)
  {
  /* Clear EndPoint1 Transmit DMA Buffer */
  memset((uint8_t*)endp1Tbuff, 0, DEF_ENDP1_MAX_SIZE);
  // Enable USB
  s = USB30_device_init();
  if(s)
  {
  cprintf("USB30_device_init err\n");
  while(1);
  }

  USBSS->UEP0_DMA = (uint32_t)(uint8_t *)endp0RTbuff;
  USBSS->UEP1_TX_DMA = (uint32_t)(uint8_t *)endp1Tbuff;
  USBSS->UEP2_TX_DMA = (uint32_t)(uint8_t *)endp2Txbuff;

  USBSS->UEP2_RX_DMA = (uint32_t)(uint8_t *)endp2RTbuff;
  USBSS->UEP_CFG = EP0_R_EN | EP0_T_EN | EP1_T_EN | EP2_R_EN | EP2_T_EN; // set end point rx/tx enable
  USB30_OUT_set(ENDP_2, ACK, 1); // endpoint2 receive setting

  }
  else
  {
  // Disable USB
  USB30_switch_powermode(POWER_MODE_2);
  USBSS->LINK_CFG = PIPE_RESET | LFPS_RX_PD;
  USBSS->LINK_CTRL = GO_DISABLED | POWER_MODE_3;
  USBSS->LINK_INT_CTRL = 0;
  USBSS->USB_CONTROL = USB_FORCE_RST | USB_ALL_CLR;
  }
}

/*******************************************************************************
 * @fn   USB30_NonStandardReq
 *
 * @brief  USB3.0 non-standard request (called from USBSS_IRQHandler())
 *
 * @return length
 */
 
 int setcode = 0;
 
 
uint16_t USB30_NonStandardReq()
{
  uint8_t endp_dir;
  struct usb_cdc_line_coding coding;
  SetupReqCode = UsbSetupBuf->bRequest;
  SetupLen = UsbSetupBuf->wLength;
  endp_dir = UsbSetupBuf->bRequestType & 0x80;
  uint16_t len = 0;
  coding.bitrate  = 115200; 
  coding.stopbits  =   0; 
  coding.parity  =   0; 
  coding.databits  =  8;   

#if DEBUG_USB3_REQ
  cprintf("length %d\n", UsbSetupBuf->wLength);

  cprintf("NSU3:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", endp0RTbuff[0], endp0RTbuff[1], endp0RTbuff[2], endp0RTbuff[3], endp0RTbuff[4], endp0RTbuff[5], endp0RTbuff[6], endp0RTbuff[7], endp0RTbuff[8], endp0RTbuff[9], endp0RTbuff[10], endp0RTbuff[11], endp0RTbuff[12], endp0RTbuff[13], endp0RTbuff[14], endp0RTbuff[15], endp0RTbuff[16], endp0RTbuff[17], endp0RTbuff[18], endp0RTbuff[19], endp0RTbuff[20], endp0RTbuff[21], endp0RTbuff[22], endp0RTbuff[23], endp0RTbuff[24], endp0RTbuff[25], endp0RTbuff[26],  endp0RTbuff[27], endp0RTbuff[28], endp0RTbuff[29], endp0RTbuff[30], endp0RTbuff[31], endp0RTbuff[32], endp0RTbuff[33], endp0RTbuff[34],  endp0RTbuff[35], endp0RTbuff[36], endp0RTbuff[37], endp0RTbuff[38], endp0RTbuff[39], endp0RTbuff[40], endp0RTbuff[41], endp0RTbuff[42], endp0RTbuff[43], endp0RTbuff[44], endp0RTbuff[45], endp0RTbuff[46],  endp0RTbuff[47], endp0RTbuff[48], endp0RTbuff[49], endp0RTbuff[50], endp0RTbuff[65], endp0RTbuff[66], endp0RTbuff[67], endp0RTbuff[68], endp0RTbuff[69],  endp0RTbuff[70]);

cprintf("wIndex.bw.bb1 %d\n", UsbSetupBuf->wIndex.bw.bb1);
cprintf("wValue %08x\n", UsbSetupBuf->wValue);
cprintf("bRequest %02x\n", UsbSetupBuf->bRequest);
cprintf("bRequestType %02x\n", UsbSetupBuf->bRequestType);
#endif

  switch(UsbSetupBuf->bRequest)
  {
  /*  Open the serial port and send the baud rate  */
  case 0x20:
    USB30_OUT_set(ENDP_0, ACK, 1 );
  break;
  /*  Read the current serial port configuration  */
  case 0x21:
    *(uint32_t  *)&endp0RTbuff[50] = coding.bitrate;;
    endp0RTbuff[54]=coding.stopbits;endp0RTbuff[55]=coding.parity;endp0RTbuff[56]=coding.databits;
    SetupLen = 7;
  break;
  /*  Close uart  */
  case 0x22:
    CDC_Variable_Clear();
  break;

  case CDC_SEND_BREAK:
//  if(cdc_dev->on_set_break){
//  cdc_dev->on_set_break(cdc_dev, spacket.wValue);
//  }
  return 0;
  break;

  case 0x01: // Microsoft OS 2.0 Descriptors
    switch(UsbSetupBuf->wIndex.bw.bb1)
    {
  case 0x04: // Compat ID Descriptor
    if(SetupLen>sizeof(USB_CompatId)) SetupLen = sizeof(USB_CompatId);
    pDescr = (uint8_t*)USB_CompatId;
    //cprintf(" 0104:CompactId\n");
    break;
  case 0x05: // Ext Props
    if(SetupLen>sizeof(USB_PropertyHeader)) SetupLen = sizeof(USB_PropertyHeader);
    pDescr = (uint8_t*)USB_PropertyHeader;
    //cprintf(" 0105:PropertyHeader\n");
  break;
  case 0x06:
  break;
  case 0x09:
  break;
  case 0x07:
  if(SetupLen>sizeof(USB_MSOS20DescriptorSet)) SetupLen = sizeof(USB_MSOS20DescriptorSet);
    pDescr = (uint8_t*)USB_MSOS20DescriptorSet;
    //cprintf(" 0107:MSOS20DescriptorSet\n");
  break;
  case 0x08:
  break;
  case 0xc0:
  break;
  default:
#if DEBUG_USB3_REQ
  cprintf(" non standard 01D:INVALID_REQ_CODE\n");
#endif
  SetupReqCode = INVALID_REQ_CODE;
  return USB_DESCR_UNSUPPORTED;
  break;
  }
  break;
  case 0x02: // user-defined command
  switch(UsbSetupBuf->wIndex.bw.bb1)
  {
  default:
#if DEBUG_USB3_REQ
  cprintf("non standard  02D:INVALID_REQ_CODE\n");
#endif
  SetupReqCode = INVALID_REQ_CODE;
  return USB_DESCR_UNSUPPORTED;
  break;
  }
  break;
  default:
#if DEBUG_USB3_REQ
    cprintf("non standard  D:INVALID_REQ_CODE\n");
#endif
    SetupReqCode = INVALID_REQ_CODE;
  return USB_DESCR_UNSUPPORTED;
  break;
  }

  len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen; // The length of this transmission
  if(endp_dir)
  {
    memcpy(endp0RTbuff, pDescr, len); // device  /* load upload data */
    pDescr += len;
  }
  SetupLen -= len;
  return len;
}

/*******************************************************************************
 * @fn   USB30_StandardReq
 *
 * @brief  USB3.0 standard request (called from USBSS_IRQHandler())
 *
 * @return len (send length)
 */
uint16_t USB30_StandardReq()
{
  SetupReqCode = UsbSetupBuf->bRequest;
  SetupLen = UsbSetupBuf->wLength;
  uint16_t len = 0;

#if DEBUG_USB3_REQ
  cprintf("SU3:%02x %02x %02x %02x %02x %02x %02x %02x\n", endp0RTbuff[0], endp0RTbuff[1],
  endp0RTbuff[2], endp0RTbuff[3], endp0RTbuff[4], endp0RTbuff[5],
  endp0RTbuff[6], endp0RTbuff[7]);
#endif
  switch(SetupReqCode)
  {
  case USB_GET_DESCRIPTOR:
  switch(UsbSetupBuf->wValue.bw.bb0)
  {
  case USB_DESCR_TYP_DEVICE: /* Get device descriptor */
  if(SetupLen > sizeof(USB_SS_DeviceDescriptor)) SetupLen = sizeof(USB_SS_DeviceDescriptor);
    pDescr = (uint8_t*)USB_SS_DeviceDescriptor;
  break;
  case USB_DESCR_TYP_CONFIG: /* Get configuration descriptor */
  if(SetupLen > sizeof(USB_SS_ConfigDescriptor)) SetupLen = sizeof(USB_SS_ConfigDescriptor);
    pDescr = (uint8_t*)USB_SS_ConfigDescriptor;
  break;
  case USB_DESCR_TYP_BOS: /* Get BOS Descriptor */
    //cprintf(" BOSDescriptor\n");
  if(SetupLen > sizeof(USB_BOSDescriptor)) SetupLen = sizeof(USB_BOSDescriptor);
    pDescr = (uint8_t*)USB_BOSDescriptor;
  break;
  case USB_DESCR_TYP_STRING: /* Get string descriptor */
  switch(UsbSetupBuf->wValue.bw.bb1)
  {
  case USB_DESCR_LANGID_STRING: /* Language string descriptor */
  if(SetupLen > sizeof(USB_StringLangID)) SetupLen = sizeof(USB_StringLangID);
    pDescr = (uint8_t*)USB_StringLangID;
  break;
  case USB_DESCR_VENDOR_STRING: /* Manufacturer String Descriptor */
  if(SetupLen > sizeof(USB_StringVendor)) SetupLen = sizeof(USB_StringVendor);
    pDescr = (uint8_t*)USB_StringVendor;
  break;
  case USB_DESCR_PRODUCT_STRING: /* Product String Descriptor */
  if(SetupLen > sizeof(USB_SS_StringProduct)) SetupLen = sizeof(USB_SS_StringProduct);
    pDescr = (uint8_t*)USB_SS_StringProduct;
  break;
  case USB_DESCR_SERIAL_STRING: /* Serial String Descriptor */
  if(SetupLen > sizeof(USB_StringSerial)) SetupLen = sizeof(USB_StringSerial);
    pDescr = (uint8_t*)USB_StringSerial;
  break;
  case USB_DESCR_OS_STRING: /* Microsoft OS String Descriptor */
    //cprintf(" OSStringDescriptor\n");
  if(SetupLen > sizeof(USB_OSStringDescriptor)) SetupLen = sizeof(USB_OSStringDescriptor);
    pDescr = (uint8_t*)USB_OSStringDescriptor;
  break;
  default:
    len = USB_DESCR_UNSUPPORTED; // Unsupported descriptor
    SetupReqCode = INVALID_REQ_CODE; // Invalid request code
  break;
  }
  break;
  default:
    len = USB_DESCR_UNSUPPORTED; //unsupported descriptor
    SetupReqCode = INVALID_REQ_CODE;
  break;
  }
  if(len != USB_DESCR_UNSUPPORTED)
  {
    len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen; //The length of this transmission
    memcpy(endp0RTbuff, pDescr, len);         // device  /* load upload data */
    SetupLen -= len;
    pDescr += len;
  }
  break;
  case USB_SET_ADDRESS: /* Set Address */
    SetupLen = UsbSetupBuf->wValue.bw.bb1; // Temporarily store the address of the USB device
  break;
  /*
  // TODO USB30_StandardReq() code USB_GET_CONFIGURATION
  case USB_GET_CONFIGURATION: // Get configuration value
  // endp0RTbuff[ 0 ] = g_devInfo.dev_config_value;
  //SetupLen = 1;
  break;
   */
  case USB_SET_CONFIGURATION: /* TODO USB30_StandardReq() code USB_SET_CONFIGURATION */
  break;
  case USB_CLEAR_FEATURE: /* TODO USB30_StandardReq() code USB_CLEAR_FEATURE */
  break;
  case USB_SET_FEATURE: /* TODO USB30_StandardReq() code USB_SET_FEATURE */
  break;
  case USB_GET_INTERFACE: /* TODO USB30_StandardReq() code USB_GET_INTERFACE */
  break;
  case USB_SET_INTERFACE: /* TODO USB30_StandardReq() code USB_SET_INTERFACE */
  break;
  case USB_GET_STATUS: /* TODO USB30_StandardReq() better code USB_GET_STATUS */
  len = 2;
  endp0RTbuff[0] = 0x01;
  endp0RTbuff[1] = 0x00;
  SetupLen = 0;
  break;
  case 0x30: // Not documented TODO USB30_StandardReq() 0x30 remove or document it
  break;
  case 0x31: // Not documented TODO USB30_StandardReq() 0x31 remove or document it
    SetupLen = UsbSetupBuf->wValue.bw.bb1; // Temporarily store the address of the USB device
  break;
  default:
    len = USB_DESCR_UNSUPPORTED; // return stall, unsupported command
    SetupReqCode = INVALID_REQ_CODE;
#if DEBUG_USB3_REQ
    cprintf(" stall\n");
#endif
  break;
  }
  return len;
}
/*
void uart_set_parity(uint32_t uart)
{
//  uint32_t reg32;
  uint8_t temp;
  uint8_t parity;
  temp = line_coding.parity;
  volatile uint32_t reg32 = R8_UART_LCR((volatile uint32_t)UART2);
  if (temp = 0x0) {
  parity = UART_PARITY_CLR;
  } else if (temp = 0x01) {
  parity = UART_PARITY_ODD;
  } else if (temp = 0x02) {
  parity = UART_PARITY_EVEN;
  }
  reg32 |= UART_PARITY_EN;
//  R8_UART_LCR((volatile uint32_t)uart) = reg32;
//  reg32 = R8_UART_LCR((volatile uint32_t)uart);
  reg32 = (reg32 & ~UART_PARITY_CLR) | (parity << 4);
  R8_UART2_LCR = reg32;
}
*/
/*******************************************************************************
 * @fn   EP0_IN_Callback
 *
 * @brief  USB3.0 endpoint 0 IN transaction callback called from USBSS_IRQHandler (Send data to Host).
 *
 * @return len (Send length)
 */
uint16_t EP0_IN_Callback(void)
{
  uint16_t len = 0;
#if DEBUG_USB3_EP0
  cprintf("USB3 EP0 IN\n");
#endif
  switch(SetupReqCode)
  {
  case USB_GET_DESCRIPTOR:
  len = SetupLen >= ENDP0_MAXPACK ? ENDP0_MAXPACK : SetupLen;
#if 0
//DEBUG_USB3_EP0
  cprintf("USB3 EP0 IN: GetDesc len=%d SetupLen=%d pDescr=0x%08X Before\n", len, SetupLen, pDescr);
#endif
  memcpy(endp0RTbuff, pDescr, len);
  SetupLen -= len;
  pDescr += len;
#if 0
//DEBUG_USB3_EP0
  cprintf("USB3 EP0 IN: GetDesc len=%d SetupLen=%d pDescr=0x%08X After\n", len, SetupLen, pDescr);
#endif
  break;
  /* Note USB_SET_ADDRESS is done in USB30_Setup_Status() */
  }
  return len;
}

/*******************************************************************************
 * @fn   EP0_OUT_Callback
 *
 * @brief  USB3.0 endpoint 0 OUT callback from USBSS_IRQHandler (Receive data from Host).
 *
 * @return length
 */

void CDC_reinit( uint32_t baudrate )
{
    uint32_t x;
    uint32_t t = FREQ_SYS;
    struct usb_cdc_line_coding coding;
    PFIC_DisableIRQ( UART2_IRQn );
    UART2_Reset();
    x = 10 * t * 2 / 16 / baudrate;
    x = ( x + 5 ) / 10;
    R8_UART2_DIV = 1;
    R16_UART2_DL = x;
    R8_UART2_FCR = RB_FCR_FIFO_TRIG | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
    R8_UART2_IER = RB_IER_TXD_EN;

    cprintf("databits %02x %d\n", R8_UART2_LCR, R8_UART2_LCR);

    UART2_ByteTrigCfg( UART_7BYTE_TRIG );
    UART2_INTCfg( ENABLE, RB_IER_RECV_RDY|RB_IER_LINE_STAT );
    PFIC_EnableIRQ( UART2_IRQn );
}

volatile int linechange = 0;
 
uint16_t EP0_OUT_Callback(void)
{
struct usb_cdc_line_coding coding;
#if DEBUG_USB3_EP0
  cprintf("USB3 EP0 OUT\n");

cprintf("out \n");
cprintf("NSU3:%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \n", endp0RTbuff[0], endp0RTbuff[1], endp0RTbuff[2], endp0RTbuff[3], endp0RTbuff[4], endp0RTbuff[5], endp0RTbuff[6], endp0RTbuff[7], endp0RTbuff[8], endp0RTbuff[9], endp0RTbuff[10], endp0RTbuff[11], endp0RTbuff[12], endp0RTbuff[13], endp0RTbuff[14], endp0RTbuff[15], endp0RTbuff[16], endp0RTbuff[17], endp0RTbuff[18], endp0RTbuff[19], endp0RTbuff[20], endp0RTbuff[21], endp0RTbuff[22], endp0RTbuff[23], endp0RTbuff[24], endp0RTbuff[25], endp0RTbuff[26],  endp0RTbuff[27], endp0RTbuff[28], endp0RTbuff[29], endp0RTbuff[30], endp0RTbuff[31], endp0RTbuff[32], endp0RTbuff[33], endp0RTbuff[34],  endp0RTbuff[35], endp0RTbuff[36], endp0RTbuff[37], endp0RTbuff[38], endp0RTbuff[39], endp0RTbuff[40], endp0RTbuff[41], endp0RTbuff[42], endp0RTbuff[43], endp0RTbuff[44], endp0RTbuff[45], endp0RTbuff[46],  endp0RTbuff[47], endp0RTbuff[48], endp0RTbuff[49], endp0RTbuff[50], endp0RTbuff[65], endp0RTbuff[66], endp0RTbuff[67], endp0RTbuff[68], endp0RTbuff[69],  endp0RTbuff[70]);
#endif
  coding.bitrate = (uint32_t)(endp0RTbuff[0] | (endp0RTbuff[1] << 8) | (endp0RTbuff[2] << 16) | (endp0RTbuff[3] << 24));
  vitrul_buad = coding.bitrate;
  coding.stopbits = (endp0RTbuff[4]);
  if (coding.stopbits == 2) {
    coding.stopbits = 1;
    }
        
  cprintf("first 8 of endp buf = %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", endp0RTbuff[0], endp0RTbuff[1], endp0RTbuff[2], endp0RTbuff[3], endp0RTbuff[4], endp0RTbuff[5], endp0RTbuff[6], endp0RTbuff[7], endp0RTbuff[8], endp0RTbuff[9], endp0RTbuff[10]);      
        
//  R8_UART2_LCR = R8_UART2_LCR | (coding.stopbits << 2);
  coding.parity = endp0RTbuff[5];

  cprintf("parity %02x %d\n", coding.parity, coding.parity);


//    R8_UART2_LCR = (R8_UART2_LCR |= (coding.stopbits << 2));
//  usbuart_set_line_coding(&coding);

    cprintf("databits %02x %d\n", R8_UART2_LCR, R8_UART2_LCR);
//    R8_UART2_LCR = (R8_UART2_LCR |= (coding.stopbits << 2));
    coding.databits = (coding.databits - 0x05);
  
  CDC_reinit(vitrul_buad);
  
  R8_UART2_LCR = R8_UART2_LCR & ~((uint8_t)coding.databits << RB_LCR_WORD_SZ);
  R8_UART2_LCR = R8_UART2_LCR | ((uint8_t)coding.databits << 0);
  R8_UART2_LCR = R8_UART2_LCR | ((uint8_t)coding.stopbits << RB_LCR_STOP_BIT);
  if (coding.parity) {
    R8_UART2_LCR = R8_UART2_LCR | (1 << 3);
    coding.databits = (endp0RTbuff[6]);
    R8_UART2_LCR = R8_UART2_LCR | (coding.parity << 4);
  } else {
    coding.databits = (endp0RTbuff[6]);
    R8_UART2_LCR = R8_UART2_LCR | (0 << 3);
  }
    
  cprintf("databits %02x %d\n", R8_UART2_LCR, R8_UART2_LCR);
cprintf("parity %02x %d\n", (R8_UART2_LCR & 2), (R8_UART2_LCR & 2));
  uint16_t len=0;
  return len;
}

/*******************************************************************************
 * @fn   USB30_Setup_Status
 *
 * @brief  USB3.0 Control Transfer Status Phase Callback
 *
 * @return None
 */
void USB30_Setup_Status(void)
{
#if DEBUG_USB3_EP
//  cprintf("USB3 Setup: SetupReqCode=0x%02X\n", SetupReqCode);
#endif
  switch(SetupReqCode)
  {
  case USB_SET_ADDRESS:
  g_DeviceUsbType = USB_U30_SPEED;
  g_DeviceConnectstatus = USB_INT_CONNECT_ENUM;
  USB30_device_setaddress(SetupLen); // SET ADDRESS
  break;
  }
}

/*******************************************************************************
 * @fn   TMR0_IRQHandler
 *
 * @brief  USB3.0 connection failure timeout processing
 *
 * @return None
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void TMR0_IRQHandler(void)
{
  R8_TMR0_INT_FLAG = RB_TMR_IF_CYC_END;
  if(link_sta == 1)
  {
  link_sta = 0;
  PFIC_DisableIRQ(USBSS_IRQn);
  PFIC_DisableIRQ(LINK_IRQn);
  USB30D_init(DISABLE);
  // cprintf("USB3.0 disable\n");
  return;
  }
  if(link_sta != 3)
  {
  PFIC_DisableIRQ(USBSS_IRQn);
  PFIC_DisableIRQ(LINK_IRQn);
  USB30D_init(DISABLE);
  // cprintf("USB2.0\n");
  R32_USB_CONTROL = 0;
  PFIC_EnableIRQ(USBHS_IRQn);
  USB20_Device_Init(ENABLE);
  }
  link_sta = 1;
  R8_TMR0_INTER_EN = 0;
  PFIC_DisableIRQ(TMR0_IRQn);
  R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
  return;
}

/*******************************************************************************
 * @fn   LINK_IRQHandler
 *
 * @brief  USB3.0 Link Interrupt Handler.
 *
 * @return None
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void LINK_IRQHandler(void)
{
  if(USBSS->LINK_INT_FLAG & LINK_Ux_EXIT_FLAG) // device enter U2
  {
  USBSS->LINK_CFG = CFG_EQ_EN | DEEMPH_CFG | TERM_EN;
  USB30_switch_powermode(POWER_MODE_0);
  USBSS->LINK_INT_FLAG = LINK_Ux_EXIT_FLAG;
  }
  if(USBSS->LINK_INT_FLAG & LINK_RDY_FLAG) // POLLING SHAKE DONE
  {
  USBSS->LINK_INT_FLAG = LINK_RDY_FLAG;
  if(tx_lmp_port) // LMP, TX PORT_CAP & RX PORT_CAP
  {
  USBSS->LMP_TX_DATA0 = LINK_SPEED | PORT_CAP | LMP_HP;
  USBSS->LMP_TX_DATA1 = UP_STREAM | NUM_HP_BUF;
  USBSS->LMP_TX_DATA2 = 0x0;
  tx_lmp_port = 0;
  g_DeviceConnectstatus = USB_INT_CONNECT;
  g_DeviceUsbType = USB_U30_SPEED;
  }
  // Successful USB3.0 communication
  link_sta = 3;
  PFIC_DisableIRQ(TMR0_IRQn);
  R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
  R8_TMR0_INTER_EN = 0;
  PFIC_DisableIRQ(USBHS_IRQn);
  USB20_Device_Init(DISABLE);
  }

  if(USBSS->LINK_INT_FLAG & LINK_INACT_FLAG)
  {
  USBSS->LINK_INT_FLAG = LINK_INACT_FLAG;
  USB30_switch_powermode(POWER_MODE_2);
  }
  if(USBSS->LINK_INT_FLAG & LINK_DISABLE_FLAG) // GO DISABLED
  {
  USBSS->LINK_INT_FLAG = LINK_DISABLE_FLAG;
  link_sta = 1;
  USB30D_init(DISABLE);
  PFIC_DisableIRQ(USBSS_IRQn);
  R8_TMR0_CTRL_MOD = RB_TMR_ALL_CLEAR;
  R8_TMR0_INTER_EN = 0;
  PFIC_DisableIRQ(TMR0_IRQn);
  PFIC_EnableIRQ(USBHS_IRQn);
  USB20_Device_Init(ENABLE);
  }
  if(USBSS->LINK_INT_FLAG & LINK_RX_DET_FLAG)
  {
  USBSS->LINK_INT_FLAG = LINK_RX_DET_FLAG;
  USB30_switch_powermode(POWER_MODE_2);
  }
  if(USBSS->LINK_INT_FLAG & TERM_PRESENT_FLAG) // term present , begin POLLING
  {
  USBSS->LINK_INT_FLAG = TERM_PRESENT_FLAG;
  if(USBSS->LINK_STATUS & LINK_PRESENT)
  {
  USB30_switch_powermode(POWER_MODE_2);
  USBSS->LINK_CTRL |= POLLING_EN;
  }
  else
  {
  USBSS->LINK_INT_CTRL = 0;
  bsp_wait_us_delay(2000);
  USB30_BusReset();
  g_DeviceConnectstatus = USB_INT_DISCONNECT;
  g_DeviceUsbType = 0;
  }
  }
  if(USBSS->LINK_INT_FLAG & LINK_TXEQ_FLAG) // POLLING SHAKE DONE
  {
  tx_lmp_port = 1;
  USBSS->LINK_INT_FLAG = LINK_TXEQ_FLAG;
  USB30_switch_powermode(POWER_MODE_0);
  }
  if(USBSS->LINK_INT_FLAG & WARM_RESET_FLAG)
  {
  USBSS->LINK_INT_FLAG = WARM_RESET_FLAG;
  USB30_switch_powermode(POWER_MODE_2);
  USB30_BusReset();
  USB30_device_setaddress(0);
  USBSS->LINK_CTRL |= TX_WARM_RESET;
  while(USBSS->LINK_STATUS & RX_WARM_RESET)
  ;
  USBSS->LINK_CTRL &= ~TX_WARM_RESET;
  bsp_wait_us_delay(2);
  // cprintf("warm reset\n");
  }
  if(USBSS->LINK_INT_FLAG & HOT_RESET_FLAG) // The host may issue a hot reset, pay attention to the configuration of the endpoint
  {
  USBSS->USB_CONTROL |= ((uint32_t)1) << 31;
  USBSS->LINK_INT_FLAG = HOT_RESET_FLAG; // HOT RESET begin
  USBSS->UEP0_TX_CTRL = 0;
  USB30_IN_set(ENDP_1, DISABLE, NRDY, 0, 0);
  USB30_IN_set(ENDP_2, DISABLE, NRDY, 0, 0);

  USB30_OUT_set(ENDP_1, NRDY, 0);
  USB30_OUT_set(ENDP_2, NRDY, 0);

  USB30_device_setaddress(0);
  USBSS->LINK_CTRL &= ~TX_HOT_RESET; // HOT RESET end
  }
  if(USBSS->LINK_INT_FLAG & LINK_GO_U1_FLAG) // device enter U1
  {
  USB30_switch_powermode(POWER_MODE_1);
  USBSS->LINK_INT_FLAG = LINK_GO_U1_FLAG;
  }
  if(USBSS->LINK_INT_FLAG & LINK_GO_U2_FLAG) // device enter U2
  {
  USB30_switch_powermode(POWER_MODE_2);
  USBSS->LINK_INT_FLAG = LINK_GO_U2_FLAG;
  }
  if(USBSS->LINK_INT_FLAG & LINK_GO_U3_FLAG) // device enter U2
  {
  USB30_switch_powermode(POWER_MODE_2);
  USBSS->LINK_INT_FLAG = LINK_GO_U3_FLAG;
  }
  return;
}
/***************Endpointn IN Transaction Processing*******************/

/*******************************************************************************
 * @fn   EP1_IN_Callback
 *
 * @brief  USB3.0 endpoint1 in callback called from USBSS_IRQHandler (Send data to Host).
 *
 * @return None
 */
void EP1_IN_Callback(void)
{
//  int nump;
#if DEBUG_USB3_EPX
//  cprintf("USB3 EP1 IN: nump=%d\n", nump);
#endif
#if 0
  cprintf("EP1-%d TX Before\n", nump);
  bsp_wait_ms_delay(10); // Simulate a Delay to retrieve data before to send
  cprintf("EP1-%d TX After\n", nump);
#endif

}

/*******************************************************************************
 * @fn   EP2_IN_Callback
 *
 * @brief  USB3.0 endpoint2 in callback called from USBSS_IRQHandler (Send data to Host).
 *
 * @return None
 */
void EP2_IN_Callback(void)
{
//  uint8_t nump;
//  nump = USB30_IN_nump(ENDP_2); //nump: number of remaining packets to be sent
#if DEBUG_USB3_EPX
//  cprintf("USB3 EP2 IN: nump=%d\n", nump);
#endif
  USB30_IN_clearIT( ENDP_2 );
  USB30_IN_set( ENDP_2 , ENABLE , NRDY , 0 , 0);
  UploadPoint2_Busy = 0;
}

/*******************************************************************************
 * @fn   EP3_IN_Callback
 *
 * @brief  USB3.0 endpoint3 in callback called from USBSS_IRQHandler (Send data to Host).
 *
 * @return None
 */
void EP3_IN_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP3 IN\n");
#endif
}

/*******************************************************************************
 * @fn   EP4_IN_Callback
 *
 * @brief  USB3.0 endpoint4 in callback called from USBSS_IRQHandler (Send data to Host).
 *
 * @return None
 */
void EP4_IN_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP4 IN\n");
#endif
}
/*******************************************************************************
 * @fn   EP5_IN_Callback
 *
 * @brief  USB3.0 endpoint5 in callback called from USBSS_IRQHandler (Send data to Host).
 *
 * @return None
 */
void EP5_IN_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP5 IN\n");
#endif
}
/*******************************************************************************
 * @fn   EP6_IN_Callback
 *
 * @brief  USB3.0 endpoint6 in callback called from USBSS_IRQHandler (Send data to Host).
 *
 * @return None
 */
void EP6_IN_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP6 IN\n");
#endif
}
/*******************************************************************************
 * @fn   EP7_IN_Callback
 *
 * @brief  USB3.0 endpoint7 in callback called from USBSS_IRQHandler (Send data to Host).
 *
 * @return None
 */
void EP7_IN_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP7 IN\n");
#endif
}
/*************** Endpointn OUT Transaction Processing *******************/

/*******************************************************************************
 * @fn   EP1_OUT_Callback
 *
 * @brief  USB3.0 endpoint1 out callback called from USBSS_IRQHandler (Receive data from Host).
 *
 * @return None
 */
void EP1_OUT_Callback(void)
{
//  uint16_t rx_len;
//  uint8_t  nump;
//  uint8_t  status;

//  USB30_OUT_status(ENDP_1, &nump, &rx_len, &status); // Get the number of received packets rxlen is the packet length of the last packet
#if DEBUG_USB3_EPX
//  cprintf("USB3 EP1 OUT nump=%d rx_len=%d status=%d\n", nump, rx_len, status);
#endif

}

/*******************************************************************************
 * @fn   EP2_OUT_Callback
 *
 * @brief  USB3.0 endpoint2 out callback called from USBSS_IRQHandler (Receive data from Host).
 *
 * @return None
 */
void EP2_OUT_Callback(void)
{
  uint16_t rx_len;
  uint8_t  nump;
  uint8_t  status;

  USB30_OUT_status(ENDP_2, &nump, &rx_len, &status); // Get the number of received packets rxlen is the packet length of the last packet
#if DEBUG_USB3_EPX
  cprintf("USB3 EP1 OUT nump=%d rx_len=%d status=%d\n", nump, rx_len, status);
#endif

  USBByteCount = rx_len;
  USB30_OUT_clearIT(ENDP_2);
  USB30_OUT_set(ENDP_2, NRDY , 0 );
  DownloadPoint2_Busy = 0;
}
/*******************************************************************************
 * @fn   EP3_OUT_Callback
 *
 * @brief  USB3.0 endpoint3 out callback called from USBSS_IRQHandler (Receive data from Host).
 *
 * @return None
 */
void EP3_OUT_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP3 OUT\n");
#endif
}
/*******************************************************************************
 * @fn   EP4_OUT_Callback
 *
 * @brief  USB3.0 endpoint4 out callback called from USBSS_IRQHandler (Receive data from Host).
 *
 * @return None
 */
void EP4_OUT_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP4 OUT\n");
#endif
}
/*******************************************************************************
 * @fn   EP5_OUT_Callback
 *
 * @brief  USB3.0 endpoint5 out callback called from USBSS_IRQHandler (Receive data from Host).
 *
 * @return None
 */
void EP5_OUT_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP5 OUT\n");
#endif
}
/*******************************************************************************
 * @fn   EP6_OUT_Callback
 *
 * @brief  USB3.0 endpoint6 out callback called from USBSS_IRQHandler (Receive data from Host).
 *
 * @return None
 */
void EP6_OUT_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP6 OUT\n");
#endif
}
/*******************************************************************************
 * @fn   EP7_OUT_Callback
 *
 * @brief  USB3.0 endpoint7 out callback called from USBSS_IRQHandler (Receive data from Host).
 *
 * @return None
 */
void EP7_OUT_Callback(void)
{
#if DEBUG_USB3_EPX
  cprintf("USB3 EP7 OUT\n");
#endif
}

/*******************************************************************************
 * @fn   USB30_ITP_Callback
 *
 * @brief  USB3.0 Isochronous Timestamp Packet (ITP) Callback
 *
 * @return None
 */
void USB30_ITP_Callback(uint32_t ITPCounter)
{
#if DEBUG_USB3_EPX
  cprintf("USB30_ITP_Callback\n");
#endif
}

/*******************************************************************************
 * @fn   USBSS_IRQHandler
 *
 * @brief  USB3.0 Interrupt Handler.
 *
 * @return None
 */
__attribute__((interrupt("WCH-Interrupt-fast"))) void USBSS_IRQHandler(void)
{
  static uint32_t count = 0;

  uint32_t usb_status = USBSS->USB_STATUS;
  if((usb_status & 1) == 0)
  {
  if((usb_status & 2) != 0)
  {
  if((USBSS->LMP_RX_DATA0 & 0x1e0) == 0xa0)
  {
    USBSS->LMP_TX_DATA0 = 0x2c0;
    USBSS->LMP_TX_DATA1 = 0;
    USBSS->LMP_TX_DATA2 = 0;
  }
  USBSS->USB_STATUS = 2;
  return;
  }
  if ((usb_status & 8) == 0)
  {
  return;
  }
  USB30_ITP_Callback(USBSS->USB_ITP);
  USBSS->USB_STATUS = 8;
  return;
  }
  // USB3 EP1 to EP7 management
  uint8_t ep = (usb_status >> 8) & 7;
  uint32_t ep_in = (usb_status >> 0xc) & 1;
  if(ep != 0)
  {
  if(ep_in != 0)
  {
  switch(ep)
  {
  case 1:
    EP1_IN_Callback();
  return;
  case 2:
    EP2_IN_Callback();
  break;
  case 3:
    EP3_IN_Callback();
  return;
  case 4:
    EP4_IN_Callback();
  return;
  case 5:
    EP5_IN_Callback();
  return;
  case 6:
    EP6_IN_Callback();
  return;
  case 7:
    EP7_IN_Callback();
  return;
  }
  return;
  }
  switch(ep)
  {
  case 1:
    EP1_OUT_Callback();
  break;
  case 2:
    EP2_OUT_Callback();
  return;
  case 3:
    EP3_OUT_Callback();
  return;
  case 4:
    EP4_OUT_Callback();
  return;
  case 5:
    EP5_OUT_Callback();
  return;
  case 6:
    EP6_OUT_Callback();
  return;
  case 7:
    EP7_OUT_Callback();
  return;
  }
  return;
  }
  // USB3 EP0 management
  uint16_t req_len;
  if(ep_in != 0)
  {
  req_len = EP0_IN_Callback();
  count = count + 1;
  if(req_len == 0)
  {
    count = 0;
    USBSS->USB_FC_CTRL = 0x41;
    USBSS->UEP0_RX_CTRL = 0x4010000;
    USBSS->UEP0_TX_CTRL = 0x14010000;
  return;
  }
    USBSS->USB_FC_CTRL = 0x41;
    USBSS->UEP0_RX_CTRL = 0;
    USBSS->UEP0_TX_CTRL = (USBSS->UEP0_TX_CTRL & 0xec1ffc00) | (count * 0x200000) | req_len | 0x14010000;
  return;
  }

  uint32_t uep0_rx_ctrl = USBSS->UEP0_RX_CTRL;
  if(-1 < ((int32_t)(uep0_rx_ctrl << 1)) )
  {
  if( ((int32_t)(uep0_rx_ctrl << 2)) < 0)
  {
    USB30_Setup_Status();
    USBSS->UEP0_RX_CTRL = 0;
    USBSS->UEP0_TX_CTRL = 0;
  return;
  }
    EP0_OUT_Callback();
    USBSS->USB_FC_CTRL = 0x41;
    USBSS->UEP0_RX_CTRL = 0x4010000;
  return;
  }
    USBSS->UEP0_RX_CTRL = 0;
  uint8_t data_req = *((uint8_t*)endp0RTbuff);
  if ((data_req & 0x60) == 0)
  {
    req_len = USB30_StandardReq();
  }
  else
  {
    req_len = USB30_NonStandardReq();
  }
  if ((char)data_req < '\0')
  {
  if (req_len == 0xffff)
  {
    USBSS->USB_FC_CTRL = 0x41;
    USBSS->UEP0_TX_CTRL = 0x8010000;
  return;
  }
  if (req_len != 0)
  {
  if (0x200 < req_len)
  {
  return;
  }
    USBSS->USB_FC_CTRL = 0x41;
    USBSS->UEP0_TX_CTRL = req_len | 0x14010000;
  return;
  }
  }
  else
  {
  if (req_len == 0xffff)
  {
    USBSS->USB_FC_CTRL = 0x41;
    USBSS->UEP0_TX_CTRL = 0x8010000;
  return;
  }
  if (req_len != 0)
  {
  if (0x200 < ep_in)
  {
  return;
  }
    USBSS->USB_FC_CTRL = 0x10001;
    USBSS->UEP0_RX_CTRL = 0x4010000;
  return;
  }
  }
    USBSS->USB_FC_CTRL = 0x41;
    USBSS->UEP0_RX_CTRL = 0x4010000;
  return;
}
