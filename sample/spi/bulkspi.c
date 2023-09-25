#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "cdc.h"

#include "vars.h"
#include "bulkspi.h"


#define GPIO1_PORT   GPIOA
#define GPIO1_PIN    GPIO8
#define GPIO2_PORT   GPIOC
#define GPIO2_PIN    GPIO7
#define GPIO3_PORT   GPIOG
#define GPIO3_PIN    GPIO7
#define GPIO4_PORT   GPIOC
#define GPIO4_PIN    GPIO7
#define GPIO5_PORT   GPIOK
#define GPIO5_PIN    GPIO1
#define GPIO6_PORT
#define GPIO6_PIN
#define GPIO7_PORT
#define GPIO7_PIN
#define GPIO8_PORT
#define GPIO8_PIN

#define IRQ_TYPE_NONE		0
#define IRQ_TYPE_EDGE_RISING	0x00000001
#define IRQ_TYPE_EDGE_FALLING	0x00000002
#define IRQ_TYPE_EDGE_BOTH	IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING
#define IRQ_TYPE_LEVEL_HIGH	0x00000004
#define IRQ_TYPE_LEVEL_LOW	0x00000008

int CSPORT = GPIO1_PORT;
int CSPIN = GPIO1_PIN;

int irqfire = 1;
int irqtype = EXTI_TRIGGER_FALLING;
int cshigh = 0;

static void my_delay_2( void )
{
	for (unsigned i = 0; i < 20000; i++)
	  {
		__asm__("nop");
	  }
}

static void my_delay_1( void )
{
   for (unsigned i = 0; i < 800000; i++)
     {
        __asm__("nop");
     }
}


static void irq_pin_init(void)
{
//    nvic_disable_irq(NVIC_EXTI0_IRQ);
    gpio_toggle(GPIOK, GPIO6);
	my_delay_2();
    nvic_enable_irq(NVIC_EXTI1_IRQ);
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	my_delay_2();
	exti_select_source(EXTI1, GPIO5_PORT);
    exti_set_trigger(EXTI1, irqtype);
	exti_enable_request(EXTI1);
	irqfire = 1;
}

static void irq_none(void)
{
    gpio_toggle(GPIOK, GPIO5);
//    nvic_disable_irq(NVIC_EXTI1_IRQ);
	my_delay_2();
//	exti_disable_request(EXTI1);
//	my_delay_2();
	irqfire = 0;
}

static void usbgpio_output(int gpio)
{
	if (gpio == 1) {
	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1_PIN);
	gpio_set_output_options(GPIO1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
							GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
    } else if (gpio == 2) {
	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	gpio_set_output_options(GPIO2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_set_output_options(GPIO3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set_output_options(GPIO4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	gpio_set_output_options(GPIO5_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}

    my_delay_1();
}

static void usbgpio_input(int gpio)
{

	if (gpio == 1) {
//	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1_PIN);
//	gpio_set(GPIO1_PORT, GPIO1_PIN);
    ;
	} else if (gpio == 2) {
	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}

	my_delay_1();
}


#define CMD_TXZEROS    63
#define SPION          74
#define SPIOFF         75
#define CMD_RX         65
#define CMD_TXRX       66
#define CMD_HZ         67
#define CMD_MODE       68
#define CMD_BPW        69
#define CMD_CSON       71
#define CMD_CSOFF      72
#define CMD_CSMODE     73
#define CMD_CSNUM      76
#define CMD_LSB        160
#define CMD_i2cspeed     81
#define CMD_i2cspeedrpt 82
#define STATUS_IDLE	   0
uint8_t spibuf[64];
static int configured;

uint8_t spibuf512[1024] = {0x00};

int diffr = 0;
//void recusb_sendspi(uint16_t len);
static void setupcs(void);

int recspi_sendusb(usbd_device *dev, uint16_t len);

static uint8_t status = STATUS_IDLE;

unsigned char my_spi_read8(unsigned long spi);

void upd_spi_mode(int mode);

void upd_spi_bpw(int bpw);

void upd_spi_lsb(int lsb);

unsigned char my_spi_read8(unsigned long spi)
{
unsigned char d;
d = spi_read(spi);
return d;
}

int firstboot = 1;
int offset = 0;

uint8_t my_spi_flush(unsigned long spi)
{
uint8_t temp = 0;
while((SPI_SR(spi) & SPI_SR_RXWNE) || ( (SPI_SR(spi) & (SPI_SR_RXPLVL_MASK<<SPI_SR_RXPLVL_SHIFT)) !=0 ) )

temp = SPI_RXDR8(spi);
return temp;
}

void my_spi_send8(unsigned long spi,unsigned char d)
{
SPI_CR1(spi) |= SPI_CR1_CSTART;
while(!(SPI_SR(spi) & SPI_SR_TXP));
SPI_TXDR8(spi)=d;
while( !( SPI_SR(spi) & SPI_SR_TXC));
}

void upd_spi_div(int spihz);
int is_first_closest(int hz);

static int nums[9] = {9996,4900,2400,1150,615,302,146,77,38};

int is_first_closest(int hz) {
    int clock = rcc_get_spi_clk_freq(SPI2);
    int divis = 2;
    int ii = 0;
    float result = 000000000;
    for (divis = 2; divis < 513; divis = divis * 2) {
    result = (clock / ((double)divis));
    nums[ii] = ((result / 10000) - 2);
    ii++;
    }
    for (int i = 0; i < 9; ++i) {
        if (hz > nums[i]) {
        printf("clock set to %d\r\n", (nums[i]  + 2)* 10000);
            return i;
        }
    }
    return 1;
}

void upd_spi_div(int spihz) {
    int result;
    spi_disable(SPI2);
    result = is_first_closest(spihz);
    spi_set_baudrate_prescaler(SPI2, result);
    printf("spi divider set to %d\r\n", result);
    spi_enable(SPI2);
}

void upd_spi_mode(int mode) {
    spi_disable(SPI2);
    spi_set_standard_mode(SPI2, mode);
    spi_enable(SPI2);
}

void upd_spi_bpw(int bpw) {
    spi_disable(SPI2);
    spi_set_data_size(SPI2, bpw);
    spi_enable(SPI2);
}

void upd_spi_lsb(int lsb) {
    spi_disable(SPI2);
    if (lsb == 0x08) {
        spi_send_lsb_first(SPI2);
        } else {
        spi_send_msb_first(SPI2);
        }
    spi_enable(SPI2);
}

static enum usbd_request_return_codes spi_control_request(
    usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)len;
    (void) buf;
//	(void)usbd_dev;
	(void)complete;
    static uint8_t reply_buf[64];
    static uint8_t test[64];
	uint8_t cmd = req->bRequest;
	uint8_t cscmd = req->wIndex;
	uint8_t size = req->wValue;
	uint8_t spidir = req->wLength;
	uint8_t nsize = size / 8;
	uint8_t dest2[] = {00};
	int ret;
	int actsz;
	static int p = 0;
	uint8_t temp;

   (*len) = 1;
   (*buf)[0] = 1; //success

    if (req->bRequest > 50) {
	switch (req->bRequest) {

    case CMD_BPW:
        upd_spi_bpw(req->wValue - 1);
        return USBD_REQ_HANDLED;

    case CMD_MODE:
        upd_spi_mode(req->wValue);
        return USBD_REQ_HANDLED;

    case CMD_i2cspeed:
        i2cspeed = req->wValue;
        i2c_init();
        return USBD_REQ_HANDLED;

    case CMD_i2cspeedrpt:
        (*buf)[0] = 1;
        (*buf)[1] = i2cspeed;
		*len = 2;
        return USBD_REQ_HANDLED;

    case CMD_HZ:
        upd_spi_div(req->wValue);
        return USBD_REQ_HANDLED;

    case CMD_CSON:
        gpio_clear(GPIOA, GPIO8);
       for (unsigned i = 0; i < 80; i++)
        {
        __asm__("nop");
        }
        return USBD_REQ_HANDLED;

    case CMD_CSOFF:
        gpio_set(GPIOA, GPIO8);
       for (unsigned i = 0; i < 80; i++)
        {
        __asm__("nop");
        }
        return USBD_REQ_HANDLED;

    case CMD_CSMODE:
        cshigh = req->wValue;
        setupcs();
        return USBD_REQ_HANDLED;

    case CMD_CSNUM:
        if (req->wValue == 0) {
        CSPORT = GPIO1_PORT;
        CSPIN = GPIO1_PIN;
        } else if (req->wValue == 1) {
        CSPORT = GPIO2_PORT;
        CSPIN = GPIO2_PIN;
        } else if (req->wValue == 2) {
        CSPORT = GPIO3_PORT;
        CSPIN = GPIO3_PIN;
        } else if (req->wValue == 3) {
        CSPORT = GPIO4_PORT;
        CSPIN = GPIO4_PIN;
        }
        return USBD_REQ_HANDLED;

    case CMD_TXZEROS:
	    dest2[0] = 0x00;
	    gpio_clear(CSPORT, CSPIN);
	    for (uint16_t i = 0; i < req->wValue; i++)
        {
            gpio_clear(CSPORT, CSPIN);
            __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
            while (!(SPI_SR(SPI2) & SPI_SR_TXP)) {
            ;
            }
           	my_spi_send8(SPI2, dest2[0]);
           	while (!(SPI_SR(SPI2) & SPI_SR_TXP)) {
            ;
            }
		    spibuf512[i] = my_spi_flush(SPI2);
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    gpio_set(CSPORT, CSPIN);
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
		    __asm__("nop");
	    }
        gpio_set(CSPORT, CSPIN);
		return USBD_REQ_HANDLED;

	case CMD_RX:
        ret = recspi_sendusb(usbd_dev, (uint16_t)req->wValue);
        if (ret) {
        ;
        }
	    offset = 0;
		return USBD_REQ_HANDLED;

	case SPION:
		spi_enable(SPI2);
	    SPI_CR1(SPI2) |= SPI_CR1_CSTART;
	    printf("enabling spi\r\n");
	    return USBD_REQ_HANDLED;

	case SPIOFF:
		spi_disable(SPI2);
	    printf("disabling spi\r\n");
	    return USBD_REQ_HANDLED;
    }
   } else {
   if (req->wValue == 1)
     {
        if ( req->wIndex == 0 )
			{
				usbgpio_input(1);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				usbgpio_input(2);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 2 )
			{
				usbgpio_input(3);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 3 )
			{
				usbgpio_input(4);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				usbgpio_input(5);
				return USBD_REQ_HANDLED;
			}
      }
   else if (req->wValue == 2)
     {
     if ( req->wIndex == 0 )
			{
				usbgpio_output(1);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				usbgpio_output(2);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 2 )
			{
				usbgpio_output(3);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 3 )
			{
				usbgpio_output(4);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				usbgpio_output(5);
				return USBD_REQ_HANDLED;
			}
      }
   else if (req->wValue == 3)
     {
     if ( req->wIndex == 0 )
			{
            if (gpio_get(GPIO1_PORT, GPIO1_PIN)) {
            	(*buf)[0] = 1;
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1;
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
            if (gpio_get(GPIO2_PORT, GPIO2_PIN)) {
            	(*buf)[0] = 1;
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1;
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
		    return USBD_REQ_HANDLED;
		   }
	    else if ( req->wIndex == 2 )
			{
            if (gpio_get(GPIO3_PORT, GPIO3_PIN)) {
            	(*buf)[0] = 1;
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1;
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
		 else if ( req->wIndex == 3 )
			{
            if (gpio_get(GPIO4_PORT, GPIO4_PIN)) {
            	(*buf)[0] = 1;
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1;
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
            if (gpio_get(GPIO5_PORT, GPIO5_PIN)) {
            	(*buf)[0] = 1;
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = 4;
			    return 1;
			} else {
				(*buf)[0] = 0;
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = 4;
			    return 1;
			}
			return USBD_REQ_HANDLED;
			}
         }
  else if (req->wValue == 0)
     {
	 if (req->bRequest == 1)
        {
        if ( req->wIndex == 0 )
			{
				gpio_set(GPIO1_PORT, GPIO1_PIN);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				gpio_set(GPIO2_PORT, GPIO2_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 2 )
			{
				gpio_set(GPIO3_PORT, GPIO3_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 3 )
			{
				gpio_set(GPIO4_PORT, GPIO4_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				gpio_set(GPIO5_PORT, GPIO5_PIN);
				return USBD_REQ_HANDLED;
			}
        }
   else if (req->bRequest == 0)
     {
     if (req->wIndex == 0)
			{
				gpio_clear(GPIO1_PORT, GPIO1_PIN);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				gpio_clear(GPIO2_PORT, GPIO2_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 2 )
			{
				gpio_clear(GPIO3_PORT, GPIO3_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 3 )
			{
				gpio_clear(GPIO4_PORT, GPIO4_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				gpio_clear(GPIO5_PORT, GPIO5_PIN);
				return USBD_REQ_HANDLED;
			}
	  }
    }
    else if (req->wValue == 9)
     {
     gpio_toggle(GPIOK, GPIO7);
     if ( req->wIndex == 1 ) {
        irqtype = IRQ_TYPE_NONE;
        irq_none();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 2 ) {
        irqtype = IRQ_TYPE_LEVEL_HIGH;
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 3 ) {
        irqtype = IRQ_TYPE_LEVEL_LOW;
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 4 ) {
//        irqtype = IRQ_TYPE_EDGE_BOTH;
        irqtype = EXTI_TRIGGER_BOTH;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 5 ) {
//        irqtype = IRQ_TYPE_EDGE_RISING;
        irqtype = EXTI_TRIGGER_RISING;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 6 ) {
//        irqtype = IRQ_TYPE_EDGE_FALLING;
        irqtype = EXTI_TRIGGER_FALLING;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     }

//	default:
//		break;
    }
//	}

	return USBD_REQ_NEXT_CALLBACK;
}

int recspi_sendusb(usbd_device *dev, uint16_t len) {
	uint16_t x;
    uint8_t buf[BULK_EP_MAXPACKET + 1] __attribute__ ((aligned(2)));

    memcpy(buf, spibuf512, len);
    x = usbd_ep_write_packet(dev, 0x83, buf, len);
    if (x != BULK_EP_MAXPACKET) {
	;
	}
	return 0;
}

static void spi_ss_out_cb(usbd_device *dev, uint8_t ep);
static void spi_ss_in_cb(usbd_device *dev, uint8_t ep);
static void setcsend(void);
static void setcsend2(void);
static void setcsend3(void);
static void setcsstart(void);
static void setcsstart2(void);
static void setcsstart3(void);


static void setcsend(void) {
// gpio set
GPIO_BSRR(CSPORT) = CSPIN;
}

static void setcsend2(void) {
// gpio clear
GPIO_BSRR(CSPORT) = (CSPIN << 16);
}

static void setcsend3(void) {
// nada dont nuthin
;
}

static void setcsstart2(void) {
// gpio set
GPIO_BSRR(CSPORT) = CSPIN;
}

static void setcsstart(void) {
// gpio clear
GPIO_BSRR(CSPORT) = (CSPIN << 16);
}

static void setcsstart3(void) {
// nada dont nuthin
;
}

static void ( *pfsetcsend) (void) = &setcsend;
static void ( *pfsetcstart) (void) = &setcsstart;

static void setupcs(void) {
if (cshigh == 1) {
    pfsetcsend = &setcsend2;
    pfsetcstart = &setcsstart2;
    } else if (cshigh == 0) {
    pfsetcsend = &setcsend;
    pfsetcstart = &setcsstart;
    } else {
    pfsetcsend = &setcsend3;
    pfsetcstart = &setcsstart3;
    }
}


void spi_ss_out_cb(usbd_device *dev, uint8_t ep)
{
	uint16_t x;
	uint8_t buf[BULK_EP_MAXPACKET + 1] __attribute__ ((aligned(2)));
	uint8_t *dest;
	uint8_t tempy;
	dest = buf;

	x = usbd_ep_read_packet(dev, ep, dest, BULK_EP_MAXPACKET);
    if (x) {
        ;
        }



    usbd_ep_nak_set(dev, ep, 1);
        if (x > 0) {
/*        if (cshigh == 0) {
        gpio_clear(CSPORT, CSPIN);
        } else {
        gpio_set(CSPORT, CSPIN);
        }
*/
        (*pfsetcstart)();

        for (uint16_t i = 0; i < x; i++)
        {
            while (!(SPI_SR(SPI2) & SPI_SR_TXP));  //todo reading/writing should be one op
            my_spi_send8(SPI2, dest[i]);

            while (!(SPI_SR(SPI2) & SPI_SR_TXP)) {
                ;
            }
		    spibuf512[i] = my_spi_flush(SPI2);
		        }
/*		     if (cshigh == 0) {
		     gpio_set(CSPORT, CSPIN);
		     } else {
		     gpio_clear(CSPORT, CSPIN);
		     }
*/
            (*pfsetcsend)();
        }
	usbd_ep_nak_set(dev, ep, 0);

}

void spi_ss_in_cb(usbd_device *dev, uint8_t ep)
{
	(void) dev;
	(void) ep;
 	uint16_t x;
 	uint16_t t;
}

void usbspi_set_config(usbd_device *dev, uint16_t wValue)
{
    configured = wValue;

        usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, 9, NULL);
		usbd_ep_setup(dev, 0x03, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			spi_ss_out_cb);
		usbd_ep_setup(dev, 0x83, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			spi_ss_in_cb);
		usbd_register_control_callback(
			dev,
			USB_REQ_TYPE_VENDOR,
			USB_REQ_TYPE_TYPE,
			spi_control_request);

		spi_ss_in_cb(dev, 0x83);
}




