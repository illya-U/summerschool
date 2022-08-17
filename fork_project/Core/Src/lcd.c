/*
 * lcd.c
 *
 *  Created on: Aug 17, 2022
 *      Author: Admin
 */

#include "lcd.h"

extern SPI_HandleTypeDef hspi1;

static const uint8_t
Bcmd[] = {                          // Init commands for 7735B screens
    18,                             // 18 commands in list:
    ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, no args, w/delay
      50,                           //     50 ms delay
    ST77XX_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, no args, w/delay
      255,                          //     255 = max (500 ms) delay
    ST77XX_COLMOD,  1+ST_CMD_DELAY, //  3: Set color mode, 1 arg + delay:
      0x05,                         //     16-bit color
      10,                           //     10 ms delay
    ST7735_FRMCTR1, 3+ST_CMD_DELAY, //  4: Frame rate control, 3 args + delay:
      0x00,                         //     fastest refresh
      0x06,                         //     6 lines front porch
      0x03,                         //     3 lines back porch
      10,                           //     10 ms delay
    ST77XX_MADCTL,  1,              //  5: Mem access ctl (directions), 1 arg:
      0x08,                         //     Row/col addr, bottom-top refresh
    ST7735_DISSET5, 2,              //  6: Display settings #5, 2 args:
      0x15,                         //     1 clk cycle nonoverlap, 2 cycle gate
                                    //     rise, 3 cycle osc equalize
      0x02,                         //     Fix on VTL
    ST7735_INVCTR,  1,              //  7: Display inversion control, 1 arg:
      0x0,                          //     Line inversion
    ST7735_PWCTR1,  2+ST_CMD_DELAY, //  8: Power control, 2 args + delay:
      0x02,                         //     GVDD = 4.7V
      0x70,                         //     1.0uA
      10,                           //     10 ms delay
    ST7735_PWCTR2,  1,              //  9: Power control, 1 arg, no delay:
      0x05,                         //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3,  2,              // 10: Power control, 2 args, no delay:
      0x01,                         //     Opamp current small
      0x02,                         //     Boost frequency
    ST7735_VMCTR1,  2+ST_CMD_DELAY, // 11: Power control, 2 args + delay:
      0x3C,                         //     VCOMH = 4V
      0x38,                         //     VCOML = -1.1V
      10,                           //     10 ms delay
    ST7735_PWCTR6,  2,              // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16,              // 13: Gamma Adjustments (pos. polarity), 16 args + delay:
      0x09, 0x16, 0x09, 0x20,       //     (Not entirely necessary, but provides
      0x21, 0x1B, 0x13, 0x19,       //      accurate colors)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+ST_CMD_DELAY, // 14: Gamma Adjustments (neg. polarity), 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E,       //     (Not entirely necessary, but provides
      0x22, 0x1D, 0x18, 0x1E,       //      accurate colors)
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                           //     10 ms delay
    ST77XX_CASET,   4,              // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,                   //     XSTART = 2
      0x00, 0x81,                   //     XEND = 129
    ST77XX_RASET,   4,              // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,                   //     XSTART = 1
      0x00, 0x81,                   //     XEND = 160
    ST77XX_NORON,     ST_CMD_DELAY, // 17: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST77XX_DISPON,    ST_CMD_DELAY, // 18: Main screen turn on, no args, delay
      255 };                       //     255 = max (500 ms) delay

static void lcd_spi_send_mult(uint8_t com, uint8_t *data, size_t size) {
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef stat;
  stat = HAL_SPI_Transmit(&hspi1, &com, 1, 1000);
  if (stat != HAL_OK) {
    while (1) {}
  }
  if (size != 0) {
    HAL_GPIO_WritePin(LCD_A0_GPIO_Port, LCD_A0_Pin, GPIO_PIN_SET);
    stat = HAL_SPI_Transmit(&hspi1, data, size, 1000);
    if (stat != HAL_OK) {
      while (1) {}
    }
  }
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

int lcd_init(void) {
  uint8_t numCommands, cmd, numArgs;
  uint16_t ms;
  const uint8_t *ptr = Bcmd;

  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  numCommands = *ptr;
  while (numCommands--) {                // For each command...
    cmd = *ptr++;               // Read command
    numArgs = *ptr++;           // Number of args to follow
    ms = numArgs & ST_CMD_DELAY;         // If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            // Mask out delay bit
    lcd_spi_send_mult(cmd, (uint8_t*)ptr, numArgs);
    ptr += numArgs;

    if (ms) {
      ms = *ptr++; // Read post-command delay time (ms)
      if (ms == 255)
        ms = 500; // If 255, delay for 500 ms
      HAL_Delay(ms);
    }
  }
  return 0;
}
