/*
 * lcd.c
 *
 *  Created on: Aug 17, 2022
 *      Author: a
 *  Based on Adafruit GFX Library
 */

#include "lcd.h"
#include "font.h"
#include "cmsis_os.h"
#include <stm32f4xx_hal.h>
#include <stdlib.h>		// abs

#define swap16(a,b) { uint16_t t = (a); (a) = (b); (b) = (t); }

extern SPI_HandleTypeDef hspi1;
extern osSemaphoreId_t semDMACpltHandle;

const uint8_t
Rcmd[] = {                          // 7735R init, part 1 (red or green tab)
    21,                             // 15 commands in list:
    ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
      150,                          //     150 ms delay
    ST77XX_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, 0 args, w/delay
      255,                          //     500 ms delay
    ST7735_FRMCTR1, 3,              //  3: Framerate ctrl - normal mode, 3 arg:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3,              //  4: Framerate ctrl - idle mode, 3 args:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6,              //  5: Framerate - partial mode, 6 args:
      0x01, 0x2C, 0x2D,             //     Dot inversion mode
      0x01, 0x2C, 0x2D,             //     Line inversion mode
    ST7735_INVCTR,  1,              //  6: Display inversion ctrl, 1 arg:
      0x07,                         //     No inversion
    ST7735_PWCTR1,  3,              //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                         //     -4.6V
      0x84,                         //     AUTO mode
    ST7735_PWCTR2,  1,              //  8: Power control, 1 arg, no delay:
      0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
    ST7735_PWCTR3,  2,              //  9: Power control, 2 args, no delay:
      0x0A,                         //     Opamp current small
      0x00,                         //     Boost frequency
    ST7735_PWCTR4,  2,              // 10: Power control, 2 args, no delay:
      0x8A,                         //     BCLK/2,
      0x2A,                         //     opamp current small & medium low
    ST7735_PWCTR5,  2,              // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1,  1,              // 12: Power control, 1 arg, no delay:
      0x0E,
    ST77XX_INVOFF,  0,              // 13: Don't invert display, no args
    ST77XX_MADCTL,  1,              // 14: Mem access ctl (directions), 1 arg:
      0xC8,                         //     row/col addr, bottom-top refresh
    ST77XX_COLMOD,  1,              // 15: set color mode, 1 arg, no delay:
      0x05,
    ST77XX_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F,                   //     XEND = 127
    ST77XX_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x9F,                   //     XEND = 159
    ST7735_GMCTRP1, 16      ,       //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
      0x02, 0x1c, 0x07, 0x12,       //     (Not entirely necessary, but provides
      0x37, 0x32, 0x29, 0x2d,       //      accurate colors)
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      ,       //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
      0x03, 0x1d, 0x07, 0x06,       //     (Not entirely necessary, but provides
      0x2E, 0x2C, 0x29, 0x2D,       //      accurate colors)
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST77XX_NORON,     ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST77XX_DISPON,    ST_CMD_DELAY, //  4: Main screen turn on, no args w/delay
      100 };                        //     100 ms delay

static uint8_t rotation;
static uint16_t _xstart, _ystart;
static uint16_t _width, _height;
static uint8_t _colstart, _rowstart;
static uint16_t cursor_y = 0, cursor_x = 0;
static uint16_t textsize_x = 1, textsize_y = 1;
static uint16_t textcolor = ST77XX_WHITE;
static uint16_t textbgcolor = ST77XX_BLACK;
static uint8_t wrap = 1;

static void lcd_start_write(void) {
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

static void lcd_end_write(void) {
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void lcd_start_command(void) {
	HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_RESET);
}

static void lcd_start_data(void) {
	HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_SET);
}

static void lcd_spi_send_com(uint8_t com) {
	HAL_StatusTypeDef stat;
	lcd_start_command();
	stat = HAL_SPI_Transmit(&hspi1, &com, 1, 1000);
	if (stat != HAL_OK) {
		while (1) {}
	}
}

static void lcd_spi_send_data(uint8_t *data, size_t size) {
	HAL_StatusTypeDef stat;
	if (size) {
		lcd_start_data();
		stat = HAL_SPI_Transmit(&hspi1, data, size, 1000);
		if (stat != HAL_OK) {
			while (1) {}
		}
	}
}

static void lcd_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
	x += _xstart;
	y += _ystart;

	uint8_t xa[4] = {x >> 8, x, (x+w-1)>>8, (x+w-1) };
	uint8_t ya[4] = {y >> 8, y, (y+h-1)>>8, (y+h-1) };

	lcd_spi_send_com(ST77XX_CASET);
	lcd_spi_send_data(xa, 4);
	lcd_spi_send_com(ST77XX_RASET);
	lcd_spi_send_data(ya, 4);
	lcd_spi_send_com(ST77XX_RAMWR);
}

/**
 * -------------------------------------------------------------------
 */
int lcd_init(void) {
	uint8_t numCommands, cmd, numArgs;
	uint16_t ms;

	const uint8_t *ptr = Rcmd;

	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
	osDelay(100);
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
	osDelay(100);
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
	osDelay(100);

	numCommands = *ptr++;
	while (numCommands--) {
		cmd = *ptr++;
		numArgs = *ptr++;
		ms = numArgs & ST_CMD_DELAY;
		numArgs &= ~ST_CMD_DELAY;
		lcd_start_write();
		lcd_spi_send_com(cmd);
		lcd_spi_send_data((uint8_t*)ptr, numArgs);
		lcd_end_write();
		ptr += numArgs;

		if (ms) {
			ms = *ptr++;
			if (ms == 255)
				ms = 500;
			osDelay(ms);
		}
	}

	lcd_start_write();
	lcd_set_rotation(2);
	lcd_end_write();

	return 0;
}

int lcd_set_rotation(int m) {
	uint8_t madctl = 0;

	rotation = m & 3; // can't be higher than 3

	switch (rotation) {
	case 0:
		madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
		_height = ST7735_TFTHEIGHT_160;
		_width = ST7735_TFTWIDTH_128;
		_xstart = _colstart;
		_ystart = _rowstart;
		break;
	case 1:
		madctl = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
		_width = ST7735_TFTHEIGHT_160;
		_height = ST7735_TFTWIDTH_128;
		_ystart = _colstart;
		_xstart = _rowstart;
		break;
	case 2:
		madctl = ST77XX_MADCTL_RGB;
		_height = ST7735_TFTHEIGHT_160;
		_width = ST7735_TFTWIDTH_128;
		_xstart = _colstart;
		_ystart = _rowstart;
		break;
	case 3:
		madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST7735_MADCTL_BGR;
		_width = ST7735_TFTHEIGHT_160;
		_height = ST7735_TFTWIDTH_128;
		_ystart = _colstart;
		_xstart = _rowstart;
		break;
	}

	lcd_spi_send_com(ST77XX_MADCTL);
	lcd_spi_send_data(&madctl, 1);
	return 0;
}

void lcd_pixel(int16_t x, int16_t y, uint16_t color) {
	if ((x >= 0) && (x < _width) && (y >= 0) && (y < _height)) {
		lcd_start_write();
		lcd_set_window(x, y, 1, 1);
		lcd_start_data();
		uint8_t c[2] = { color >> 8, color};
		lcd_spi_send_data(c, 2);
		lcd_end_write();
	}
}

void lcd_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
	lcd_start_write();
	lcd_set_window(x, y, 1, h);
	lcd_start_data();
	uint8_t c[2] = { color >> 8, color};
	do
		lcd_spi_send_data(c, 2);
	while (h--);
	lcd_end_write();
}

void lcd_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
	lcd_start_write();
	lcd_set_window(x, y, w, 1);
	lcd_start_data();
	uint8_t c[2] = { color >> 8, color};
	do
		lcd_spi_send_data(c, 2);
	while (w--);
	lcd_end_write();
}

void lcd_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
	lcd_vline(x, y, h, color);
	lcd_vline(x+w, y, h, color);
	lcd_hline(x, y, w, color);
	lcd_hline(x, y+h, w, color);
}

void lcd_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
	if (x < 0) x = 0;
	if (y < 0) y = 0;
	if (x + w > _width) w = _width - x;
	if (y + h > _height) h = _height - y;
	lcd_start_write();
	lcd_set_window(x, y, w, h);
	uint8_t c[2] = { color >> 8, color};
	uint32_t cnt = w * h;
	while (cnt--)
		lcd_spi_send_data(c, 2);
	lcd_end_write();
}

void lcd_fill(uint16_t color) {
	lcd_fill_rect(0, 0, _width, _height, color);
}

void lcd_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap16(x0, y0);
		swap16(x1, y1);
	}

	if (x0 > x1) {
		swap16(x0, x1);
		swap16(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0 <= x1; x0++) {
		if (steep) {
			lcd_pixel(y0, x0, color);
		} else {
			lcd_pixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

void lcd_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  lcd_start_write();
  lcd_pixel(x0, y0 + r, color);
  lcd_pixel(x0, y0 - r, color);
  lcd_pixel(x0 + r, y0, color);
  lcd_pixel(x0 - r, y0, color);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    lcd_pixel(x0 + x, y0 + y, color);
    lcd_pixel(x0 - x, y0 + y, color);
    lcd_pixel(x0 + x, y0 - y, color);
    lcd_pixel(x0 - x, y0 - y, color);
    lcd_pixel(x0 + y, y0 + x, color);
    lcd_pixel(x0 - y, y0 + x, color);
    lcd_pixel(x0 + y, y0 - x, color);
    lcd_pixel(x0 - y, y0 - x, color);
  }
  lcd_end_write();
}

void lcd_fill_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;
	int16_t px = x;
	int16_t py = y;

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		if (x < (y + 1)) {
			lcd_vline(x0 + x, y0 - y, 2 * y + 1, color);
			lcd_vline(x0 - x, y0 - y, 2 * y + 1, color);
		}
		if (y != py) {
			lcd_vline(x0 + py, y0 - px, 2 * px + 1, color);
			lcd_vline(x0 - py, y0 - px, 2 * px + 1, color);
			py = y;
		}
		px = x;
	}
	lcd_vline(x0, y0 - r, 2 * r + 1, color);
}

void lcd_set_text_color(uint16_t color) {
	textcolor = color;
}

void lcd_set_text_bg_color(uint16_t color) {
	textbgcolor = color;
}

void lcd_char(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y) {

	if ((x >= _width) ||
		(y >= _height) ||
		((x + 6 * size_x - 1) < 0) ||
		((y + 8 * size_y - 1) < 0))
		return;

	if (c >= 176)
		c++;

	for (int8_t i = 0; i < 5; i++) {
		uint8_t line = font[c * 5 + i];
		for (int8_t j = 0; j < 8; j++, line >>= 1) {
			if (line & 1) {
				if (size_x == 1 && size_y == 1)
					lcd_pixel(x + i, y + j, color);
				else
					lcd_fill_rect(x + i * size_x, y + j * size_y, size_x, size_y,
				color);
			} else if (bg != color) {
				if (size_x == 1 && size_y == 1)
					lcd_pixel(x + i, y + j, bg);
				else
					lcd_fill_rect(x + i * size_x, y + j * size_y, size_x, size_y, bg);
			}
		}
	}
	if (bg != color) {
		if (size_x == 1 && size_y == 1)
			lcd_vline(x + 5, y, 8, bg);
		else
			lcd_fill_rect(x + 5 * size_x, y, size_x, 8 * size_y, bg);
	}
}

void lcd_putchar(char c) {
	if (c == '\n') {
		cursor_x = 0;
		cursor_y += textsize_y * 8;
	} else if (c != '\r') {
		if (wrap && ((cursor_x + textsize_x * 6) > _width)) {
			cursor_x = 0;
			cursor_y += textsize_y * 8;
		}
		lcd_char(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x, textsize_y);
		cursor_x += textsize_x * 6;
	}
}

void lcd_print(char *text) {
	while (*text) {
		lcd_putchar(*text++);
	}
}

void lcd_set_cursor(int16_t x, int16_t y) {
  cursor_x = x * textsize_x * 6;
  cursor_y = y * textsize_y * 8;
}

void lcd_set_text_size(int16_t size_x, int16_t size_y) {
  textsize_x = size_x;
  textsize_y = size_y;
}
