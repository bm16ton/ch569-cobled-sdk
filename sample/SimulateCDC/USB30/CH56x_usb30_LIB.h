/********************************** (C) COPYRIGHT *******************************
* File Name          : CH56x_usb30_lib.h
* Author             : WCH
* Version            : V1.1
* Date               : 2023/02/16
*
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef USB30_CH56X_USB30_LIB_H_
#define USB30_CH56X_USB30_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56x_common.h"

// link CFG
#define TERM_EN                 (1<<1)
#define PIPE_RESET              (1<<3)
#define LFPS_RX_PD              (1<<5)
#define CFG_EQ_EN               (1<<6)
#define DEEMPH_CFG              (1<<8)

#define POWER_MODE_0            ((uint32_t)0x00000000)
#define POWER_MODE_1            ((uint32_t)0x00000001)
#define POWER_MODE_2            ((uint32_t)0x00000002)
#define POWER_MODE_3            ((uint32_t)0x00000003)

#define LINK_PRESENT            (1<<0)
#define RX_WARM_RESET           ((uint32_t)1<<1)

#define LINK_TXEQ               (1<<6)
#define GO_DISABLED             (1<<4)
#define POLLING_EN              (1<<12)

#define TX_HOT_RESET            ((uint32_t)1<<16)
#define RX_HOT_RESET            ((uint32_t)1<<24)

#define TX_WARM_RESET           ((uint32_t)1<<8)
#define TX_Ux_EXIT              ((uint32_t)1<<9)
// link int flag
#define LINK_RDY_FLAG           (1<<0)
#define LINK_RECOV_FLAG         (1<<1)
#define LINK_INACT_FLAG         (1<<2)
#define LINK_DISABLE_FLAG       (1<<3)
#define LINK_GO_U3_FLAG         (1<<4)
#define LINK_GO_U2_FLAG         (1<<5)
#define LINK_GO_U1_FLAG         (1<<6)
#define LINK_GO_U0_FLAG         (1<<7)
#define LINK_U3_WAKE_FLAG       (1<<8)
#define LINK_Ux_REJECT_FLAG     (1<<9)
#define TERM_PRESENT_FLAG       (1<<10)
#define LINK_TXEQ_FLAG          (1<<11)
#define LINK_Ux_EXIT_FLAG       (1<<12)
#define WARM_RESET_FLAG         (1<<13)
#define U3_WAKEUP_FLAG          (1<<14)
#define HOT_RESET_FLAG          (1<<15)
#define LINK_RX_DET_FLAG        (1<<20)

#define EP0_R_EN                (1<<0)
#define EP1_R_EN                (1<<1)
#define EP2_R_EN                (1<<2)
#define EP3_R_EN                (1<<3)
#define EP4_R_EN                (1<<4)
#define EP5_R_EN                (1<<5)
#define EP6_R_EN                (1<<6)
#define EP7_R_EN                (1<<7)

#define EP0_T_EN                (1<<8)
#define EP1_T_EN                (1<<9)
#define EP2_T_EN                (1<<10)
#define EP3_T_EN                (1<<11)
#define EP4_T_EN                (1<<12)
#define EP5_T_EN                (1<<13)
#define EP6_T_EN                (1<<14)
#define EP7_T_EN                (1<<15)

#define USB_FORCE_RST           (1<<2)
#define USB_ALL_CLR             (1<<1)
// LMP
#define LMP_HP                  (0)
#define LMP_SUBTYPE_MASK        (0xf<<5)
#define SET_LINK_FUNC           (0x1<<5)
#define U2_INACT_TOUT           (0x2<<5)
#define VENDOR_TEST             (0x3<<5)
#define PORT_CAP                (0x4<<5)
#define PORT_CFG                (0x5<<5)
#define PORT_CFG_RES            (0x6<<5)

#define LINK_SPEED              (1<<9)

#define NUM_HP_BUF              (4<<0)
#define DOWN_STREAM             (1<<16)
#define UP_STREAM               (2<<16)
#define TIE_BRK                 (1<<20)



/**********standard request command***********/
typedef struct __PACKED
{
    uint8_t bRequestType;
    uint8_t bRequest;
    uint8_t wValueL;
    uint8_t wValueH;
    uint8_t wIndexL;
    uint8_t wIndexH;
    uint16_t wLength;
} *PUSB_SETUP;

#define UsbSetupBuf     ((PUSB_SETUP)endp0RTbuff)//endpoint 0


#define ENDP0_MAXPACK       512

// status response
#define NRDY                0
#define ACK                 0x01
#define STALL               0x02
#define INVALID             0x03
// number of NUMP
#define NUMP_0              0x00
#define NUMP_1              0x01
#define NUMP_2              0x02
#define NUMP_3              0x03
#define NUMP_4              0x04
#define NUMP_5              0x05
#define NUMP_6              0x06
/* USB endpoint direction */
#define OUT                 0x00
#define IN                  0x80
/* USB endpoint serial number */
#define ENDP_0              0x00
#define ENDP_1              0x01
#define ENDP_2              0x02
#define ENDP_3              0x03
#define ENDP_4              0x04
#define ENDP_5              0x05
#define ENDP_6              0x06
#define ENDP_7              0x07

#define USB_DESCR_TYP_BOS       0x0f
#define USB_DESCR_UNSUPPORTED   0xffff
#define INVALID_REQ_CODE 0xFF

/* string descriptor type */
#ifndef USB_DESCR_STRING
    #define USB_DESCR_LANGID_STRING    0x00
    #define USB_DESCR_VENDOR_STRING    0x01
    #define USB_DESCR_PRODUCT_STRING   0x02
    #define USB_DESCR_SERIAL_STRING    0x03
    #define USB_DESCR_OS_STRING        0xee
#endif


extern void USB30_Device_forceclr();
/*******************************************************************************
 * @fn      USB30_Device_Init
 *
 * @brief   USB3.0 Device initialization
 *
 * @return   None
 */
extern uint8_t USB30_Device_Init(void);

/*******************************************************************************
 * @fn      USB30_Lib_Getversion
 *
 * @brief   USB3.0 Device Lib initialization
 *
 * @return   None
 */
extern uint8_t USB30_Lib_Getversion(void);

/*******************************************************************************
 * @fn      USB30_ISO_Setendp
 *
 * @brief   Configure synchronization endpoint
 *
 * @return   None
 */
extern void USB30_ISO_Setendp(uint8_t num,FunctionalState Status );

/*******************************************************************************
 * @fn       USB30_ISO_Setdelay( uint32_t dly )
 *
 * @brief   Set synchronization delay time
 *
 * @param dly - delay time
 *
 * @return   None
 */
extern void USB30_ISO_Setdelay( uint32_t dly );

/*******************************************************************************
 * @fn       USB30_ITP_Enable
 *
 * @brief   USB ITP enable
 *
 * @param Status - enable/disable
 *
 * @return   None
 */
extern void USB30_ITP_Enable(FunctionalState Status);

/*******************************************************************************
 * @fn     USB30_OUT_Status
 *
 * @brief  Get the length of endpoint received data
 *
 * @param  endp - Endpoint number
 *         nump - The remaining number of packets that can be received by the endpoint
 *         len -  The length of data received by the endpoint. For burst transmission,
 *                it indicates the length of the last packet received by the endpoint.
 *         status -  Indicates whether the host still has data packets to be distributed,
 *                   1 - the end of the burst received non-full packets,
 *                   0 - the host still has data packets to be distributed.
 *
 * @return   None
 */
extern void USB30_OUT_Status(uint8_t endp,uint8_t *nump,uint16_t *len,uint8_t *status);

/*******************************************************************************
 * @fn     USB30_OUT_set
 *
 * @brief  Endpoint receive settings
 *
 * @param  endp - Set endpoint number
 *         nump - The remaining number of packets that can be received by the endpoint
 *         status - Endpoint Status :0-NRDY,1-ACK,2-STALL
 *
 * @return   None
 */
extern void USB30_OUT_set(uint8_t endp,uint8_t status,uint8_t nump);

/*******************************************************************************
 * @fn     USB30_OUT_clearIT
 *
 * @brief  Clear the OUT transaction completion interrupt, and only keep the package serial number
 *
 * @param  endp - Set endpoint number
 *
 * @return   None
 */
extern void USB30_OUT_clearIT(uint8_t endp);

/*******************************************************************************
 * @fn     USB30_OUT_clearPendingIT
 *
 * @brief  Clear the OUT transaction completion interrupt, and keep other endpoint configurations
 *
 * @param  endp - Set endpoint number
 *
 * @return   None
 */
extern void USB30_OUT_clearPendingIT(uint8_t endp);

/*******************************************************************************
 * @fn     USB30_OUT_ITflag
 *
 * @brief  Get the OUT transaction completion interrupt flag
 *
 * @param  endp - Set endpoint number
 *
 * @return   1 - interrupt 0 - non-interrupt
 */
extern uint8_t USB30_OUT_ITflag(uint8_t endp);

/*******************************************************************************
 * @fn      USB30_IN_Set
 *
 * @brief  Endpoint sending settings
 *
 * @param  endp - endpoint number
 *         lpf -  end of burst:   1-enable 0-disable
 *         nump -  The number of packets that the endpoint can send
 *         status -  endpoint status:  0-NRDY,1-ACK,2-STALL
 *         TxLen - The data length of the last packet sent by the endpoint
 *
 * @return   None
 */
extern void USB30_IN_Set(uint8_t endp,FunctionalState lpf,uint8_t status,uint8_t nump,uint16_t TxLen);

/*******************************************************************************
 * @fn     USB30_IN_ClearPendingIT
 *
 * @brief  Clear the IN transaction completion interrupt and keep the other configurations of the endpoint
 *
 * @param  endp - endpoint number
 *
 * @return   None
 */
extern void USB30_IN_ClearPendingIT(uint8_t endp);

/*******************************************************************************
 * @fn      USB30_IN_ClearIT
 *
 * @brief  Clear the IN transaction interrupt and the remaining status of the endpoint,
 *          and only retain the packet serial number.
 *
 * @param  endp - endpoint number
 *
 * @return   None
 */
extern void USB30_IN_ClearIT(uint8_t endp);

/*******************************************************************************
 * @fn       USB30_IN_ITflagT
 *
 * @brief    Get IN transaction completion interrupt flag
 *
 * @param    endp - Set endpoint number
 *
 * @return   None
 */
extern uint8_t USB30_IN_ITflag(uint8_t endp);

/*******************************************************************************
 * @fn       USB30_IN_Nump
 *
 * @brief   Get the remaining number of packets to be sent by the endpoint
 *
 * @param  endp - endpoint number
 *
 * @return   Remaining number of packets to be sent
 */
extern uint8_t USB30_IN_Nump(uint8_t endp);

/*******************************************************************************
 * @fn      USB30_Send_ERDY
 *
 * @brief    send ERDY packet
 *
 * @param  endp - endpoint number
 *         nump - Number of packets received or sent by the endpoint
 *
 * @return   None
 */
extern void USB30_Send_ERDY(uint8_t endp,uint8_t nump);

/*******************************************************************************
 * @fn       USB30_device_setaddress
 *
 * @brief    Set device address
 *
 * @param    address - device address
 *
 * @return   None
 */
extern void USB30_device_setaddress( uint32_t address );

/*******************************************************************************
 * @fn      USB30_IN_Nump
 *
 * @brief   Get the remaining number of packets to be sent by the endpoint
 *
 * @return  Control the length of data sent by the host when the transmission data stage is OUT
 */
extern uint16_t USB30_Setup_OutData(void);

/*******************************************************************************
 * @fn      USB30_device_setaddress
 *
 * @brief   USB30_device_setaddress
 *
 * @return   None
 */
extern void USB30_device_setaddress();

/*******************************************************************************
 * @fn      USB30_StandardReq
 *
 * @brief   USB device mode standard request command processing
 *
 * @return   The length of data sent by the host request device
 */
extern uint16_t USB30_StandardReq();

/*******************************************************************************
 * @fn      USB30_NonStandardReq
 *
 * @brief   USB device mode non-standard request command processing
 *
 * @return  The length of data sent by the host request device
 */
extern uint16_t USB30_NonStandardReq();

/*******************************************************************************
 * @fn      EP0_IN_Callback
 *
 * @brief   endpoint 0 IN Transfer completion callback function
 *
 * @return  Data length
 */
extern uint16_t EP0_IN_Callback();

/*******************************************************************************
 * @fn      EP0_OUT_Callback
 *
 * @brief   endpoint 0 OUT Transfer completion callback function
 *
 * @return   None
 */
extern uint16_t EP0_OUT_Callback();

/*******************************************************************************
 * @fn       USB30_Setup_Status
 *
 * @brief    Control transmission status stage
 *
 * @return   None
 */
extern void USB30_Setup_Status();

/*******************************************************************************
 * @fn      USB30_ITP_Callback
 *
 * @brief   ITP Callback function
 *
 * @return   None
 */
extern void USB30_ITP_Callback(uint32_t ITPCounter);

/*******************************************************************************
 * @fn      USB30_switch_pwr_mode
 *
 * @brief   switch USB3.0 Power mode
 *
 * @return   None
 */
extern void USB30_switch_powermode( uint8_t pwr_mode );

/*******************************************************************************
 * @fn      EPn_IN_Callback()
 *
 * @brief   endpointN IN transaction callback function
 *
 * @return   None
 */
extern void  EP1_IN_Callback(void);
extern void  EP2_IN_Callback(void);
extern void  EP3_IN_Callback(void);
extern void  EP4_IN_Callback(void);
extern void  EP5_IN_Callback(void);
extern void  EP6_IN_Callback(void);
extern void  EP7_IN_Callback(void);

/*******************************************************************************
 * @fn      EPn_IN_Callback()
 *
 * @brief   endpointN OUT transaction callback function
 *
 * @return   None
 */
extern void  EP1_OUT_Callback(void);
extern void  EP2_OUT_Callback(void);
extern void  EP3_OUT_Callback(void);
extern void  EP4_OUT_Callback(void);
extern void  EP5_OUT_Callback(void);
extern void  EP6_OUT_Callback(void);
extern void  EP7_OUT_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* USB30_CH56X_USB30_LIB_H_ */

