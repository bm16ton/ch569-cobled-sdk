#ifndef BULKSPI_H
#define BULKSPI_H

#include <libopencm3/usb/usbd.h>
void i2c_init(void);
void spi_start(void);
void usbspi_set_config(usbd_device *dev, uint16_t wValue);
uint8_t my_spi_flush(unsigned long spi);
void my_spi_send8(unsigned long spi,unsigned char d);
void usb_set_config(usbd_device *usbd_dev, uint16_t wValue);
//void exti1_isr(void);
extern int irqtype;
#endif
