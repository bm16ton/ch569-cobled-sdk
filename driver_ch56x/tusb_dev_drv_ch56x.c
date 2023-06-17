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
 * Copyright (c) 2021 XToolBox - admin@xtoolbox.org
 * www.tusb.org
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

/**
 * TeenyUSB device driver need implement these functions:
 *
 * tusb_dev_drv_open
 * tusb_dev_drv_close
 * tusb_dev_drv_setup_endpoint
 * tusb_dev_drv_get_context
 * tusb_dev_drv_get_speed
 * tusb_dev_drv_set_address
 * tusb_dev_drv_send_data
 * tusb_dev_drv_cancel_send
 * tusb_dev_drv_set_stall
 * tusb_dev_drv_clear_stall
 * tusb_dev_drv_set_recv_buffer
 * tusb_dev_drv_set_rx_valid
 *
 * TeenyUSB device driver need call these functions:
 *
 * tusb_device_ep_xfer_done
 * tusb_device_reset
 *
 */

#include "teeny_usb_device.h"
#include "teeny_usb_device_driver.h"
#include "teeny_usb_util.h"
#include "teeny_usb_def.h"
#include "CH56x_common.h"
#include "CH56xUSB30_LIB.h"
#include "CH56x_usb20.h"
#include "CH56x_usb30.h"
#include "CH56xUSB20_lib.h"
#include "tusb_dev_drv_ch56x.h"
#include "string.h"
#include "CH56x_usb30_devbulk_LIB.h"
#define CH_LINK_STATE_INIT  0
#define CH_LINK_STATE_20    1
#define CH_LINK_STATE_30    3

typedef struct _tusb_ep_data
{
    const uint8_t *tx_buf;           /**< data transmit buffer for IN endpoint */
    uint8_t *rx_buf;                 /**< data receive buffer for OUT endpoint */
    int32_t  tx_remain_size;         /**< data reamin size in transmit buffer */
    int32_t  tx_total_size;          /**< total transmit data length */
    int32_t  rx_size;                /**< data buffer total size in receive buffer */
    int32_t  rx_count;               /**< current received data length */
    uint16_t tx_last_size;           /**< last transmit data length */
    uint8_t  tx_need_zlp;            /**< need transmit zero length packet */
    uint8_t  rx_packet_count;        /**< packet count for burst transfer */
} tusb_ep_data;

static __attribute__ ((aligned(16))) uint8_t ep0_buff[512]  __attribute__((section(".DMADATA"))); // endpoint 0 tx/rx buffer
struct _tusb_device_driver
{
    void *context;
    tusb_ep_data Ep[CH56x_MAX_EP_COUNT];
    uint16_t mps_in[CH56x_MAX_EP_COUNT];
    uint16_t mps_out[CH56x_MAX_EP_COUNT];
    uint8_t  burst_in[CH56x_MAX_EP_COUNT];
    uint8_t  burst_out[CH56x_MAX_EP_COUNT];
    const tusb_descriptors_t* usb20_descs;
    const tusb_descriptors_t* usb30_descs;
    volatile uint8_t tx_lmp_port;
    volatile uint8_t link_sta;
    volatile uint8_t ep0_tx_toggle;
};

static tusb_device_driver_t ch56x_dev_drv;

int tusb_dev_drv_open(tusb_device_driver_t **pdrv, const tusb_hardware_param_t *param, void *context)
{
    tusb_device_driver_t * drv = &ch56x_dev_drv;
    drv->context = context;
    drv->usb30_descs = 0;
    if(param){
        drv->usb20_descs = (const tusb_descriptors_t*)param;
    }
    if(pdrv){
        *pdrv = drv;
    }
    R32_USB_CONTROL = 0;
    PFIC_EnableIRQ(USBSS_IRQn);
    PFIC_EnableIRQ(LINK_IRQn);
    PFIC_EnableIRQ(TMR0_IRQn);
    R8_TMR0_INTER_EN = 1;
    TMR0_TimerInit( 67000000 ); // about 0.5s
    ch56x_dev_drv.link_sta = CH_LINK_STATE_INIT;
    ch56x_dev_drv.tx_lmp_port = 0;

    int res = USB30_Device_Open();
    // reset with log data in IRQ will cause warm reset
    // so reset here to config the endpoint 0
    tusb_device_reset(&ch56x_dev_drv);
    return res;
}

void *tusb_dev_drv_get_context(tusb_device_driver_t *drv)
{
    return drv->context;
}

int tusb_dev_drv_close(tusb_device_driver_t *drv)
{
    if(drv->link_sta == CH_LINK_STATE_20){
        USBHS->USB_CONTROL = USB_ALL_CLR | USB_FORCE_RST;
        return 0;
    }
    USB30_Device_Close();
    PFIC_DisableIRQ(USBSS_IRQn);
    PFIC_DisableIRQ(LINK_IRQn);
    PFIC_DisableIRQ(TMR0_IRQn);
    return 0;
}

int tusb_dev_drv_setup_endpoint(tusb_device_driver_t *drv, const tusb_ep_config *ep_config, int count, int is_reset)
{
    for(int i=0;i<count;i++){
        uint8_t ep = ep_config->addr & 7;
        if(ep_config->addr & 0x80){
            if(drv->link_sta == CH_LINK_STATE_20){
                if(ep == 0){
                    USBHS->UEP0_CTRL |= EP_T_RES_NAK;
                }else{
                    USBHS->USB_BUF_MODE |= EP_TX_EN(ep);
                    (&USBHS->UEP0_CTRL)[ep] |= (EP_T_RES_NAK | EP_T_AUTO_TOG);
                }
            }else{
                USBSS->UEP_CFG |= 1 << (ep + 8);
                if(ep == 0){
                    USBSS->UEP0_DMA = (uint32_t)ep0_buff;
                }
            }
            drv->mps_in[ep] = ep_config->mps;
            drv->burst_in[ep] = ep_config->burst_count;
        }else{
            if(drv->link_sta == CH_LINK_STATE_20){
                (&USBHS->UEP0_MAX_LEN)[ep] = ep_config->mps;
                (&USBHS->UEP0_DMA)[ep] = (uint32_t)ep0_buff;
                if(ep == 0){
                    USBHS->UEP0_CTRL |= EP_R_RES_ACK;
                }else{
                    USBHS->USB_BUF_MODE |= EP_RX_EN(ep);
                    (&USBHS->UEP0_CTRL)[ep] |= (EP_R_RES_NAK | EP_R_AUTO_TOG);
                }
            }else{
                USBSS->UEP_CFG |= 1 << (ep + 0);
                if(ep == 0){
                    USBSS->UEP0_DMA = (uint32_t)ep0_buff;
                }
            }
            drv->mps_out[ep] = ep_config->mps;
            drv->burst_out[ep] = ep_config->burst_count;
        }
        TUSB_LOGD("EP %02x, mps:%4d, burst:%d\n", ep_config->addr, ep_config->mps, ep_config->burst_count);
        ep_config++;
    }
    return 0;
}

int tusb_dev_drv_get_speed(tusb_device_driver_t *drv)
{
    if(drv->link_sta == CH_LINK_STATE_20){
        return PORT_SPEED_HIGH;
    }else{
        return PORT_SPEED_SUPER;
    }
}

int tusb_dev_drv_set_address(tusb_device_driver_t *drv, uint8_t addr, uint8_t after_status_out)
{
    if (after_status_out)
    {
        if(drv->link_sta == CH_LINK_STATE_20){
            USB20_Device_Setaddress(addr);
        }else{
            USB30_Device_Setaddress(addr);
        }
    }
    return 0;
}

#define DMA_START_ADDR  0x20020000ul
#define DMA_END_ADD     (0x20020000ul + 32*1024ul)

static int tusb30_do_tx(tusb_device_driver_t *drv, uint8_t EPn)
{
    EPn &= 0x7f;
    tusb_ep_data* ep = &drv->Ep[EPn];
    uint8_t nump = 0;
    int len = ep->tx_remain_size;
    if(len > 0){
        uint32_t pkt_size = drv->mps_in[EPn] * drv->burst_in[EPn];
        if(len > pkt_size){
            len = pkt_size;
        }
        nump = (len + drv->mps_in[EPn]-1) /drv->mps_in[EPn];
        if(EPn == 0){
            memcpy(ep0_buff, ep->tx_buf + ep->tx_total_size, len);
            ep->tx_total_size += len;
            ep->tx_remain_size -= len;
            ep->tx_last_size = len;
        }else{
            volatile uint32_t* dma = &USBSS->UEP1_TX_DMA;
            dma = dma + (EPn-1)*4;
            *dma = (uint32_t)ep->tx_buf + ep->tx_total_size;
            ep->tx_total_size += len;
            ep->tx_remain_size -= len;
            ep->tx_last_size = len;
        }
        len = len - ( (nump-1) * drv->mps_in[EPn]);
    }else{
        len = 0;
        nump = 1;
        ep->tx_last_size = 0;
    }
    if(len == 1024){
        len = 0;
        nump++;
    }
    if(EPn == 0){
        if(len == 0){
            USB30_OUT_Set( 0 , ACK , nump );
        }
        USB30_EP0_IN_set(ACK, nump, len, drv->ep0_tx_toggle);
        drv->ep0_tx_toggle ^= 1;
        USB30_Send_ERDY( 0 | 0x80 , nump);
        return len;
    }
    USB30_IN_Set( EPn , ENABLE , ACK , nump , len);
    USB30_Send_ERDY( EPn | 0x80 , nump);
    return len;
}
static int tusb20_do_tx(tusb_device_driver_t *drv, uint8_t EPn);
int tusb_dev_drv_send_data(tusb_device_driver_t *drv, uint8_t EPn, const void *data, int len, uint8_t option)
{
    EPn &= 0x7f;
    tusb_ep_data* ep = &drv->Ep[EPn];
    if(EPn != 0){
        if((uint32_t)data & 15 ){
            TUSB_LOGE("CH56x USB tx buffer not 16 bytes aligned\n");
            return -1;
        }
        if( (uint32_t)data < DMA_START_ADDR || (uint32_t)data >= DMA_END_ADD){
            TUSB_LOGE("CH56x USB tx buffer must in RAMX\n");
            return -1;
        }
    }
    if(data == 0) {
        data = ep0_buff;
    }
    ep->tx_buf = (const uint8_t*)data;
    ep->tx_remain_size = len;
    ep->tx_total_size = 0;
    ep->tx_need_zlp = option & TUSB_TXF_ZLP ? 1 : 0;
    if(EPn == 0){
        drv->ep0_tx_toggle = 0;
    }
    if(drv->link_sta == CH_LINK_STATE_20){
        return tusb20_do_tx(drv, EPn);
    }else{
        return tusb30_do_tx(drv, EPn);
    }
}

int tusb_dev_drv_cancel_send(tusb_device_driver_t *drv, uint8_t EPn)
{
    (void)drv;
    (void)EPn;
    TUSB_LOGW("CH56x not support cancel send\n");
    return -1;
}

int tusb_dev_drv_set_recv_buffer(tusb_device_driver_t *drv, uint8_t EPn, void *data, int len)
{
    EPn &= 0x7f;
    tusb_ep_data* ep = &drv->Ep[EPn];
    if(EPn != 0){
        if((uint32_t)data & 15 ){
            TUSB_LOGE("CH56x USB rx buffer not 16 bytes aligned\n");
            return -1;
        }
        if( (uint32_t)data < DMA_START_ADDR || (uint32_t)data >= DMA_END_ADD){
            TUSB_LOGE("CH56x USB rx buffer must in RAMX\n");
            return -1;
        }
    }
    ep->rx_buf = (uint8_t*)data;
    ep->rx_size = len;
    ep->rx_count = 0;
    return 0;
}

void tusb_dev_drv_set_rx_valid(tusb_device_driver_t *drv, uint8_t EPn)
{
    EPn &= 0x7f;
    tusb_ep_data* ep = &drv->Ep[EPn];
    uint32_t len = ep->rx_size - ep->rx_count;
    if(drv->link_sta == CH_LINK_STATE_20){
        if(EPn != 0){
            volatile uint32_t* dma = &USBHS->UEP1_RX_DMA;
            dma = dma + (EPn-1)*4;
            *dma  = (uint32_t)ep->rx_buf + ep->rx_count;
        }
        (&USBHS->UEP0_CTRL)[EPn] = ((&USBHS->UEP0_CTRL)[EPn] & (~EP_R_RES_MASK)) | EP_R_RES_ACK;
    }else{
        uint32_t nump = len + drv->mps_out[EPn] - 1 / drv->mps_out[EPn];
        if(nump > drv->burst_out[EPn]){
            nump = drv->burst_out[EPn];
        }
        ep->rx_packet_count = nump;
        if(EPn != 0){
            volatile uint32_t* dma = &USBSS->UEP1_RX_DMA;
            dma = dma + (EPn-1)*4;
            *dma  = (uint32_t)ep->rx_buf + ep->rx_count;
        }
        USB30_OUT_Set( EPn , ACK , nump);
        USB30_Send_ERDY( EPn , nump);
    }
}

static void usb30_xfer_ready(tusb_device_driver_t *drv, uint8_t EPn, int is_in)
{
    EPn &= 0x7f;
    tusb_ep_data* ep = &drv->Ep[EPn];
    if(is_in){
        USB30_IN_Nump(EPn); // if we don't do this here, it will cause some issue
        USB30_IN_ClearIT(EPn);
        uint32_t pkt_size = drv->mps_in[EPn] * drv->burst_in[EPn];
        if(ep->tx_remain_size || (ep->tx_need_zlp && ep->tx_last_size == pkt_size)){
            tusb30_do_tx(drv, EPn);
        }else{
            tusb_device_ep_xfer_done(drv, EPn | 0x80, ep->tx_buf, ep->tx_total_size, 0);
        }
    }else{
        uint8_t xfered_packet = 0;
        uint8_t status = 0;
        uint16_t last_packet_len;
        USB30_OUT_Status(EPn,&xfered_packet,&last_packet_len,&status);
        uint32_t xfer_len = (ep->rx_packet_count - xfered_packet - 1) * drv->mps_out[EPn];
        xfer_len += last_packet_len;
        if(EPn == 0x00){
            memcpy(ep->rx_buf + ep->rx_count, ep0_buff, xfer_len);
        }else{
            USB30_OUT_ClearIT(EPn);
        }
        ep->rx_count += xfer_len;
        if(xfer_len < drv->mps_out[EPn]*drv->burst_out[EPn] || ep->rx_count == ep->rx_size){
            // short packet
            int res = tusb_device_ep_xfer_done(drv, EPn, ep->rx_buf, ep->rx_count, 0);
            ep->rx_count = 0;
            if(res == 0){
                tusb_dev_drv_set_rx_valid(drv, EPn);
            }
        }else{
            if(ep->rx_count < ep->rx_size){
                tusb_dev_drv_set_rx_valid(drv, EPn);
            }
        }
    }
}

static int tusb20_do_tx(tusb_device_driver_t *drv, uint8_t EPn)
{
    EPn &= 0x7f;
    tusb_ep_data* ep = &drv->Ep[EPn];
    int len = ep->tx_remain_size;
    if(len > 0){
        uint32_t pkt_size = drv->mps_in[EPn];
        if(len > pkt_size){
            len = pkt_size;
        }
        if(EPn == 0){
            memcpy(ep0_buff, ep->tx_buf + ep->tx_total_size, len);
            ep->tx_total_size += len;
            ep->tx_remain_size -= len;
            ep->tx_last_size = len;
        }else{
            volatile uint32_t* dma = &USBHS->UEP1_TX_DMA;
            dma = dma + (EPn-1)*4;
            *dma = (uint32_t)ep->tx_buf + ep->tx_total_size;
            ep->tx_total_size += len;
            ep->tx_remain_size -= len;
            ep->tx_last_size = len;
        }
    }else{
        len = 0;
        ep->tx_last_size = 0;
    }
    if(EPn == 0){
        USBHS->UEP0_CTRL = EP_T_RES_ACK | (drv->ep0_tx_toggle?EP_T_TOG_0:EP_T_TOG_1) | len;
        drv->ep0_tx_toggle ^= 1;
        return len;
    }
    (&USBHS->UEP0_CTRL)[EPn] = ((&USBHS->UEP0_CTRL)[EPn] & ((~EP_T_RES_MASK) & 0xffff0000) ) | EP_T_RES_ACK | len;
    return len;
}

static void usb20_xfer_ready(tusb_device_driver_t *drv, uint8_t EPn, int is_in)
{
    EPn &= 0x7f;
    tusb_ep_data* ep = &drv->Ep[EPn];
    if(is_in){
        (&USBHS->UEP0_CTRL)[EPn] = ((&USBHS->UEP0_CTRL)[EPn] & (~EP_T_RES_MASK)) | EP_T_RES_NAK;
        uint32_t pkt_size = drv->mps_in[EPn] * drv->burst_in[EPn];
        if(ep->tx_remain_size || (ep->tx_need_zlp && ep->tx_last_size == pkt_size)){
            tusb20_do_tx(drv, EPn);
        }else{
            tusb_device_ep_xfer_done(drv, EPn | 0x80, ep->tx_buf, ep->tx_total_size, 0);
        }
    }else{
        uint32_t xfer_len = USBHS->USB_RX_LEN;
        (&USBHS->UEP0_CTRL)[EPn] = ((&USBHS->UEP0_CTRL)[EPn] & (~EP_R_RES_MASK)) | EP_R_RES_NAK;
        if(EPn == 0x00){
            memcpy(ep->rx_buf + ep->rx_count, ep0_buff, xfer_len);
        }
        ep->rx_count += xfer_len;
        if(xfer_len < drv->mps_out[EPn] || ep->rx_count == ep->rx_size){
            // short packet
            int res = tusb_device_ep_xfer_done(drv, EPn, ep->rx_buf, ep->rx_count, 0);
            ep->rx_count = 0;
            if(res == 0){
                tusb_dev_drv_set_rx_valid(drv, EPn);
            }
        }else{
            if(ep->rx_count < ep->rx_size){
                tusb_dev_drv_set_rx_valid(drv, EPn);
            }
        }
    }
}

void tusb_dev_drv_set_stall(tusb_device_driver_t *drv, uint8_t EPn)
{
    if(drv->link_sta == CH_LINK_STATE_20){
        if(EPn & 0x80){
            (&USBHS->UEP0_CTRL)[EPn&0x7f] =
                    ((&USBHS->UEP0_CTRL)[EPn&0x7f] & (~EP_T_RES_MASK)) | EP_T_RES_STALL;
        }else{
            (&USBHS->UEP0_CTRL)[EPn] = ((&USBHS->UEP0_CTRL)[EPn] & (~EP_R_RES_MASK)) | EP_R_RES_STALL;
        }
    }else{
        if(EPn & 0x80){
            USB30_IN_Set( EPn & 0x7f , ENABLE , STALL , 1 , 0);
            USB30_Send_ERDY( EPn & 0x7f , 1);
        }else{
            USB30_OUT_Set(EPn, STALL, 1);
            USB30_Send_ERDY( EPn, 1);
        }
    }
}

void tusb_dev_drv_clear_stall(tusb_device_driver_t *drv, uint8_t EPn)
{
    if(drv->link_sta == CH_LINK_STATE_20){
        if(EPn & 0x80){
            (&USBHS->UEP0_CTRL)[EPn&0x7f] = ((&USBHS->UEP0_CTRL)[EPn&0x7f] & (~EP_T_RES_MASK)) | EP_T_RES_NAK;
        }else{
            (&USBHS->UEP0_CTRL)[EPn] = ((&USBHS->UEP0_CTRL)[EPn] & (~EP_R_RES_MASK)) |EP_R_RES_NAK;
        }
    }else{
        if(EPn & 0x80){
            USB30_IN_Set( EPn & 0x7f , ENABLE , NRDY , 1 , 0);
            USB30_Send_ERDY( EPn & 0x7f , 1);
        }else{
            USB30_OUT_Set(EPn, NRDY, 1);
            USB30_Send_ERDY( EPn, 1);
        }
    }
}

