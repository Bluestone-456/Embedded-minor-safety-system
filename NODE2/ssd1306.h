#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f4xx.h"
#include "fonts.h"

#define SSD1306_I2C_ADDR        0x78
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64
#define SSD1306_COLOR_WHITE     1

void SSD1306_Init(void);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
char SSD1306_DrawChar(char ch, FontDef Font, uint8_t color);
char SSD1306_DrawString(char* str, FontDef Font, uint8_t color);
void SSD1306_SetCursor(uint8_t x, uint8_t y);

#endif