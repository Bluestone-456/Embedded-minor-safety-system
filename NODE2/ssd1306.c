#include "ssd1306.h"
#include <string.h>
#include "stm32f4xx.h"

// Reference to the function in main.c
extern void I2C1_WriteData(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t size);

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static struct { uint16_t CurrentX; uint16_t CurrentY; } SSD1306;

void SSD1306_WriteCommand(uint8_t cmd) {
    I2C1_WriteData(SSD1306_I2C_ADDR, 0x00, &cmd, 1);
}

void SSD1306_WriteData(uint8_t* data, uint16_t size) {
    I2C1_WriteData(SSD1306_I2C_ADDR, 0x40, data, size);
}

void SSD1306_Init(void) {
    for(volatile int i=0; i<100000; i++); // Startup Delay
    SSD1306_WriteCommand(0xAE); SSD1306_WriteCommand(0x20); SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(0xB0); SSD1306_WriteCommand(0xC8); SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(0x10); SSD1306_WriteCommand(0x40); SSD1306_WriteCommand(0x81);
    SSD1306_WriteCommand(0xFF); SSD1306_WriteCommand(0xA1); SSD1306_WriteCommand(0xA6);
    SSD1306_WriteCommand(0xA8); SSD1306_WriteCommand(0x3F); SSD1306_WriteCommand(0xA4);
    SSD1306_WriteCommand(0xD3); SSD1306_WriteCommand(0x00); SSD1306_WriteCommand(0xD5);
    SSD1306_WriteCommand(0xF0); SSD1306_WriteCommand(0xD9); SSD1306_WriteCommand(0x22);
    SSD1306_WriteCommand(0xDA); SSD1306_WriteCommand(0x12); SSD1306_WriteCommand(0xDB);
    SSD1306_WriteCommand(0x20); SSD1306_WriteCommand(0x8D); SSD1306_WriteCommand(0x14);
    SSD1306_WriteCommand(0xAF);
    SSD1306_Clear();
    SSD1306_UpdateScreen();
}

void SSD1306_Clear(void) { memset(SSD1306_Buffer, 0, sizeof(SSD1306_Buffer)); }

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    if (color == 1) SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    else            SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}

void SSD1306_SetCursor(uint8_t x, uint8_t y) { SSD1306.CurrentX = x; SSD1306.CurrentY = y; }

char SSD1306_DrawChar(char ch, FontDef Font, uint8_t color) {
    uint32_t i, b, j;
    if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) || SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight)) return 0;
    for (i = 0; i < Font.FontHeight; i++) {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++) {
            if ((b << j) & 0x8000) SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), color);
            else                   SSD1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), !color);
        }
    }
    SSD1306.CurrentX += Font.FontWidth;
    return ch;
}

char SSD1306_DrawString(char* str, FontDef Font, uint8_t color) {
    while (*str) { SSD1306_DrawChar(*str, Font, color); str++; }
    return *str;
}

void SSD1306_UpdateScreen(void) {
    for (uint8_t i = 0; i < 8; i++) {
        SSD1306_WriteCommand(0xB0 + i); SSD1306_WriteCommand(0x00); SSD1306_WriteCommand(0x10);
        I2C1_WriteData(SSD1306_I2C_ADDR, 0x40, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
    }
}