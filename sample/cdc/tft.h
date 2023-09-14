
#include "CH56x_common.h"
#include "fonts/bitmap_typedefs.h"

#define ST_USE_SPI_DMA
#define HAS_CS
#ifdef HAS_CS
	#define USE_CS
#endif

extern int lcddma;

#define swap(a, b) { int16_t t = a; a = b; b = t; }

void spi_init(void);

#define ST7789_NOP          0x00
#define ST7789_SWRESET		0x01
#define ST7789_RDDID		0x04
#define ST7789_RDDST		0x09

#define ST7789_SLPIN		0x10
#define ST7789_SLPOUT  		0x11
#define ST7789_PTLON   		0x12
#define ST7789_NORON   		0x13

#define ST7789_INVOFF  		0x20
#define ST7789_INVON   		0x21
#define ST7789_DISPOFF 		0x28
#define ST7789_DISPON  		0x29
#define ST7789_CASET   		0x2A
#define ST7789_RASET   		0x2B
#define ST7789_RAMWR   		0x2C
#define ST7789_RAMRD   		0x2E

#define ST7789_PTLAR   		0x30
#define ST7789_COLMOD  		0x3A
#define ST7789_MADCTL  		0x36

#define ST7789_MADCTL_MY  0x80  // Page Address Order
#define ST7789_MADCTL_MX  0x40  // Column Address Order
#define ST7789_MADCTL_MV  0x20  // Page/Column Order
#define ST7789_MADCTL_ML  0x10  // Line Address Order
#define ST7789_MADCTL_MH  0x04  // Display Data Latch Order
#define ST7789_MADCTL_RGB 0x00
#define ST7789_MADCTL_BGR 0x08

#define ST7789_RDID1   		0xDA
#define ST7789_RDID2   		0xDB
#define ST7789_RDID3   		0xDC
#define ST7789_RDID4   		0xDD

// color modes
#define ST7789_COLOR_MODE_65K      0x50
#define ST7789_COLOR_MODE_262K     0x60
#define ST7789_COLOR_MODE_12BIT    0x03
#define ST7789_COLOR_MODE_16BIT    0x05
#define ST7789_COLOR_MODE_18BIT    0x06
#define ST7789_COLOR_MODE_16M      0x07

#define TFT_MADCTL  0x36
#define TFT_MAD_MY  0x80
#define TFT_MAD_MX  0x40
#define TFT_MAD_MV  0x20
#define TFT_MAD_ML  0x10
#define TFT_MAD_BGR 0x08
#define TFT_MAD_MH  0x04
#define TFT_MAD_SS  0x02
#define TFT_MAD_GS  0x01
#define TFT_MAD_RGB 0x00
/*
#define ILI_PWCTR6  0xFC
*/

#define TFT_CASET   0x2A
#define TFT_PASET   0x2B
#define TFT_RAMWR   0x2C

// Color definitions

#define	ST_R_POS_RGB   11	// Red last bit position for RGB display
#define	ST_G_POS_RGB   5 	// Green last bit position for RGB display
#define	ST_B_POS_RGB   0	// Blue last bit position for RGB display

#define	ST_RGB(R,G,B) \
	(((uint16_t)(R >> 3) << ST_R_POS_RGB) | \
	((uint16_t)(G >> 2) << ST_G_POS_RGB) | \
	((uint16_t)(B >> 3) << ST_B_POS_RGB))

#define ST_COLOR_BLACK       ST_RGB(0,     0,   0)
#define ST_COLOR_NAVY        ST_RGB(0,     0, 123)
#define ST_COLOR_DARKGREEN   ST_RGB(0,   125,   0)
#define ST_COLOR_DARKCYAN    ST_RGB(0,   125, 123)
#define ST_COLOR_MAROON      ST_RGB(123,   0,   0)
#define ST_COLOR_PURPLE      ST_RGB(123,   0, 123)
#define ST_COLOR_OLIVE       ST_RGB(123, 125,   0)
#define ST_COLOR_LIGHTGREY   ST_RGB(198, 195, 198)
#define ST_COLOR_DARKGREY    ST_RGB(123, 125, 123)
#define ST_COLOR_BLUE        ST_RGB(0,     0, 255)
#define ST_COLOR_GREEN       ST_RGB(0,   255,   0)
#define ST_COLOR_CYAN        ST_RGB(0,   255, 255)
#define ST_COLOR_RED         ST_RGB(255,   0,   0)
#define ST_COLOR_MAGENTA     ST_RGB(255,   0, 255)
#define ST_COLOR_YELLOW      ST_RGB(255, 255,   0)
#define ST_COLOR_WHITE       ST_RGB(255, 255, 255)
#define ST_COLOR_ORANGE      ST_RGB(255, 165,   0)
#define ST_COLOR_GREENYELLOW ST_RGB(173, 255,  41)
#define ST_COLOR_PINK        ST_RGB(255, 130, 198)


#define DC_CMD  {R32_PB_CLR |= 1 << 10;}  //dc pb10 low
#define DC_DATA {R32_PB_OUT |= 1 << 10;}  //dc pb10 high
#define CS_IDLE {R32_PB_OUT |= 1 << 11;}  //cs pb11 high
#define CS_ACTIVE {R32_PB_CLR |= 1 << 11;}  //cs pb11 low


__attribute__((always_inline)) static inline void _write_command_8bit(uint8_t cmd)
{

#ifdef USE_CS
	CS_ACTIVE;
#endif
	DC_CMD;
    SPI1_MasterSendByte(cmd);
#ifdef USE_CS
	CS_IDLE;
#endif
	DC_DATA;
}

__attribute__((always_inline)) static inline void _write_data_8bit(uint8_t dat)
{

#ifdef USE_CS
	CS_ACTIVE;
#endif
	DC_DATA;
	SPI1_MasterSendByte(dat);
#ifdef USE_CS
	CS_IDLE;
#endif

}

void st_set_address_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

/**
 * Fills `len` number of pixels with `color`.
 * Call st_set_address_window() before calling this function.
 * @param color 16-bit RGB565 color value
 * @param len 32-bit number of pixels
 */
void st_fill_color(uint16_t color, uint32_t len);

/**
 * Fills `len` number of pixels with `color`.
 * Call st_set_address_window() before calling this function.
 * @param color_arr pointer to uint8_t array. Each 16-bit color is seperated into two 8-bit `high` and `low` components
 * @param bytes 32-bit number of bytes in the array (= no. of pixels x2)
 */

void st_fill_color_array(uint8_t *color_arr, uint32_t bytes);

/**
 * Draw a line from (x0,y0) to (x1,y1) with `width` and `color`.
 * @param x0 start column address.
 * @param y0 start row address.
 * @param x1 end column address.
 * @param y1 end row address.
 * @param width width or thickness of the line
 * @param color 16-bit RGB565 color of the line
 */
void st_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t width, uint16_t color);

/**
 * Experimental
 * Draw a rectangle without filling it
 * @param x start column address.
 * @param y start row address
 * @param w Width of rectangle
 * @param h height of rectangle
 */
void st_draw_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/*
 * Called by st_draw_line().
 * User need not call it
 */
void _st_plot_line_low(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t width, uint16_t color);

/*
 * Called by st_draw_line().
 * User need not call it
 */
void _st_plot_line_high(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t width, uint16_t color);

/*
 * Called by st_draw_line().
 * User need not call it
 */
void _st_draw_fast_h_line(uint16_t x0, uint16_t y0, uint16_t x1, uint8_t width, uint16_t color);

/*
 * Called by st_draw_line().
 * User need not call it
 */
void _st_draw_fast_v_line(uint16_t x0, uint16_t y0, uint16_t y1, uint8_t width, uint16_t color);

/**
 * Rotate the display clockwise or anti-clockwie set by `rotation`
 * @param rotation Type of rotation. Supported values 0, 1, 2, 3
 */
void st_rotate_display(uint8_t rotation);

/**
 * Initialize the display driver
 */
void st_init(void);

/**
 * Fills a rectangular area with `color`.
 * Before filling, performs area bound checking
 * @param x Start col address
 * @param y Start row address
 * @param w Width of rectangle
 * @param h Height of rectangle
 * @param color 16-bit RGB565 color
 */
void st_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/*
 * Same as `st_fill_rect()` but does not do bound checking, so it's slightly faster
 */
void st_fill_rect_fast(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

/**
 * Fill the entire display (screen) with `color`
 * @param color 16-bit RGB565 color
 */
void st_fill_screen(uint16_t color);

/*
 * Render a character glyph on the display. Called by `st_draw_string_main()`
 * User need NOT call it
 */
void _st_render_glyph(uint16_t x, uint16_t y, uint16_t fore_color, uint16_t back_color, const tImage *glyph, uint8_t is_bg);

/**
 * Renders a string by drawing each character glyph from the passed string.
 * Called by `st_draw_string()` and `st_draw_string_withbg()`.
 * Text is wrapped automatically if it hits the screen boundary.
 * x_padding and y_padding defines horizontal and vertical distance (in px) between two characters
 * is_bg=1 : Text will habe background color,   is_bg=0 : Text will have transparent background
 * User need NOT call it.
 */
void _st_draw_string_main(uint16_t x, uint16_t y, char *str, uint16_t fore_color, uint16_t back_color, const tFont *font, uint8_t is_bg);

/**
 * Draws a character at a given position, fore color, back color.
 * @param x Start col address
 * @param y Start row address
 * @param character the ASCII character to be drawn
 * @param fore_color foreground color
 * @param back_color background color
 * @param font Pointer to the font of the character
 * @param is_bg Defines if character has background or not (transparent)
 */
void st_draw_char(uint16_t x, uint16_t y, char character, uint16_t fore_color, uint16_t back_color, const tFont *font, uint8_t is_bg);

/**
 * Draws a string on the display with `font` and `color` at given position.
 * Background of this string is transparent
 * @param x Start col address
 * @param y Start y address
 * @param str pointer to the string to be drawn
 * @param color 16-bit RGB565 color of the string
 * @param font Pointer to the font of the string
 */
void st_draw_string(uint16_t x, uint16_t y, char *str, uint16_t color, const tFont *font);

/**
 * Draws a string on the display with `font`, `fore_color`, and `back_color` at given position.
 * The string has background color
 * @param x Start col address
 * @param y Start y address
 * @param str pointer to the string to be drawn
 * @param foe_color 16-bit RGB565 color of the string
 * @param back_color 16-bit RGB565 color of the string's background
 * @param font Pointer to the font of the string
 */
void st_draw_string_withbg(uint16_t x, uint16_t y, char *str, uint16_t fore_color, uint16_t back_color, const tFont *font);

/**
 * Draw a bitmap image on the display
 * @param x Start col address
 * @param y Start row address
 * @param bitmap Pointer to the image data to be drawn
 */
void st_draw_bitmap(uint16_t x, uint16_t y, const tImage *bitmap);
//void st_draw_bitmap_old(uint16_t x, uint16_t y, const tImage16bit *bitmap);

/**
 * Draw a pixel at a given position with `color`
 * @param x Start col address
 * @param y Start row address
 */
void st_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

void st_fill_triangle(int16_t x0, int16_t y0,
		      int16_t x1, int16_t y1,
		      int16_t x2, int16_t y2, uint16_t color);

void st_draw_triangle(int16_t x0, int16_t y0,
		      int16_t x1, int16_t y1,
		      int16_t x2, int16_t y2, uint16_t color);

void st_fill_round_rect(int16_t x, int16_t y, int16_t w,
		       int16_t h, int16_t r, uint16_t color);

void st_draw_round_rect(int16_t x, int16_t y, int16_t w,
		       int16_t h, int16_t r, uint16_t color);

void st_fill_circle_helper(int16_t x0, int16_t y0, int16_t r,
			  uint8_t cornername, int16_t delta, uint16_t color);

void st_fill_circle(int16_t x0, int16_t y0, int16_t r,
		    uint16_t color);

void st_draw_circle_helper(int16_t x0, int16_t y0,
			  int16_t r, uint8_t cornername, uint16_t color);

void st_draw_circle(int16_t x0, int16_t y0, int16_t r,
		    uint16_t color);

void gfx_drawLine(int16_t x0, int16_t y0,
			    int16_t x1, int16_t y1,
			    uint16_t color);

			    void _st_draw_fast_rounded_h_line(int16_t x, int16_t y,
		       int16_t w, uint16_t color);

void gfx_drawFastVLine(int16_t x, int16_t y,
		       int16_t h, uint16_t color);

void gfx_drawFastHLine(int16_t x, int16_t y,
		       int16_t w, uint16_t color);

void st_draw_bitmap_nodma(uint16_t x, uint16_t y, const tImage *bitmap);
void st_fill_color_nodma(uint16_t color, uint32_t len);
void st_fill_rect_nodma(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void st_fill_screen_nodma(uint16_t color);
