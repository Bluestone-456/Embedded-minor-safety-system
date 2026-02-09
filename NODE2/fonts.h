#ifndef FONTS_H
#define FONTS_H

#include <stdint.h>
#include <stddef.h> // Required for NULL

// --- 1. ENABLE THE FONTS YOU WANT ---
// These defines tell fonts.c to compile these specific fonts.
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

// --- 2. DEFINE THE STRUCTURE ---
// This matches the format used in your fonts.c snippet
typedef struct {
    uint8_t FontWidth;    // Font width in pixels
    uint8_t FontHeight;   // Font height in pixels
    const uint16_t *data; // Pointer to data array
    const uint8_t *char_width; // Pointer to char widths (can be NULL)
} SSD1306_Font_t;

// Alias "FontDef" to "SSD1306_Font_t" so the driver code works
typedef SSD1306_Font_t FontDef;

// --- 3. EXPORT FONTS ---
extern const SSD1306_Font_t Font_7x10;
extern const SSD1306_Font_t Font_11x18;
extern const SSD1306_Font_t Font_16x26;

#endif