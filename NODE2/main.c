/*  NODE 2: RECEIVER  */
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "ssd1306.h"
#include "fonts.h"




volatile float rem_Temp = 0.0f;
volatile float rem_Press = 0.0f;
volatile uint32_t rem_Gas = 0;
volatile uint32_t rem_Sound = 0;
volatile uint8_t rem_Vibration = 0;


#define RX_SIZE 100
char rxBuffer[RX_SIZE];
volatile uint8_t rxIndex = 0;
volatile uint8_t dataReady = 0;

volatile uint32_t msTicks = 0;


void SystemClock_Config(void);
void SysTick_Init(void);
void GPIO_Init(void);
void I2C1_Init(void);
void SER_Init(void);
void Delay(uint32_t ms);
void Parse_Data(void);
void DrawVerticalBars(uint32_t gas, uint32_t sound);


void I2C1_WriteData(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t size);

int main(void) {
    SystemClock_Config();
    SysTick_Init();
    GPIO_Init();
    I2C1_Init();
    SER_Init();
    
    SSD1306_Init();
    
    char buf[32];
    uint32_t vib_timer = 0;

    while(1) {
        
        if (dataReady) {
            Parse_Data();
            dataReady = 0; // 
           
            if (rem_Vibration) vib_timer = 30; 
        }

        
        SSD1306_Clear();

         
        if (rem_Temp > 30.0f || rem_Gas > 2000 || rem_Sound > 700) {
            SSD1306_SetCursor(0, 0);
            SSD1306_DrawString("! WARNING !", Font_11x18, SSD1306_COLOR_WHITE);
            
            if (rem_Gas > 2000) {
                SSD1306_SetCursor(0, 25);
                SSD1306_DrawString("GAS HIGH!", Font_11x18, SSD1306_COLOR_WHITE);
                sprintf(buf, "Lvl: %lu", rem_Gas);
            } else if (rem_Sound > 700) {
                SSD1306_SetCursor(0, 25);
                SSD1306_DrawString("TOO LOUD!", Font_11x18, SSD1306_COLOR_WHITE);
                sprintf(buf, "Vol: %lu", rem_Sound);
            } else {
                SSD1306_SetCursor(0, 25);
                SSD1306_DrawString("TEMP HIGH!", Font_11x18, SSD1306_COLOR_WHITE);
                sprintf(buf, "%.2f C", rem_Temp);
            }
            SSD1306_SetCursor(0, 45);
            SSD1306_DrawString(buf, Font_7x10, SSD1306_COLOR_WHITE);
            
        } else {
            
            sprintf(buf, "%.1f C", rem_Temp);
            SSD1306_SetCursor(0, 0); SSD1306_DrawString(buf, Font_11x18, SSD1306_COLOR_WHITE);
            
            sprintf(buf, "%.0f hPa", rem_Press / 100.0f);
            SSD1306_SetCursor(0, 20); SSD1306_DrawString(buf, Font_7x10, SSD1306_COLOR_WHITE);
            
            sprintf(buf, "Gas: %lu", rem_Gas);
            SSD1306_SetCursor(0, 35); SSD1306_DrawString(buf, Font_7x10, SSD1306_COLOR_WHITE);
            
            sprintf(buf, "Snd: %lu", rem_Sound);
            SSD1306_SetCursor(0, 48); SSD1306_DrawString(buf, Font_7x10, SSD1306_COLOR_WHITE);
        }

        
        if(vib_timer > 0) {
             SSD1306_SetCursor(100, 0);
             SSD1306_DrawString("VIB", Font_7x10, SSD1306_COLOR_WHITE);
             vib_timer--;
        }

        DrawVerticalBars(rem_Gas, rem_Sound);
        SSD1306_UpdateScreen();
        
        
        GPIOD->ODR ^= (1 << 12);
        Delay(50); 
    }
}


void Parse_Data(void) {
    
    sscanf(rxBuffer, "%f,%f,%lu,%lu,%hhu", 
           &rem_Temp, &rem_Press, &rem_Gas, &rem_Sound, &rem_Vibration);
}


void UART4_IRQHandler(void) {
    if (UART4->SR & (1<<5)) { 
        char c = UART4->DR;
        if (c == '\n') {
            rxBuffer[rxIndex] = '\0'; 
            dataReady = 1;
            rxIndex = 0;
        } else {
            if (rxIndex < RX_SIZE - 1) rxBuffer[rxIndex++] = c;
        }
    }
}


void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
    RCC->PLLCFGR = (8 << 0) | (336 << 6) | (0 << 16) | (7 << 24) | RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
void SysTick_Init(void) { SysTick_Config(168000000 / 1000); }
void SysTick_Handler(void) { msTicks++; }
void Delay(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms);
}
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    
    GPIOB->MODER |= (2<<12)|(2<<14); GPIOB->OTYPER |= (1<<6)|(1<<7); GPIOB->OSPEEDR |= (3<<12)|(3<<14); GPIOB->PUPDR |= (1<<12)|(1<<14); GPIOB->AFR[0] |= (4<<24)|(4<<28);
    
    GPIOD->MODER |= (1<<24);
}
void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; I2C1->CR1 |= I2C_CR1_SWRST; I2C1->CR1 &= ~I2C_CR1_SWRST; 
    I2C1->CR2 = 42; I2C1->CCR = 35 | I2C_CCR_FS; I2C1->TRISE = 43; I2C1->CR1 |= I2C_CR1_PE;
}
void I2C1_WriteData(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t size) {
    I2C1->CR1 |= I2C_CR1_START; while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = addr; while (!(I2C1->SR1 & I2C_SR1_ADDR)); (void)I2C1->SR2;
    I2C1->DR = reg; while (!(I2C1->SR1 & I2C_SR1_TXE));
    for (uint16_t i = 0; i < size; i++) { I2C1->DR = data[i]; while (!(I2C1->SR1 & I2C_SR1_TXE)); }
    while (!(I2C1->SR1 & I2C_SR1_BTF)); I2C1->CR1 |= I2C_CR1_STOP;
}
void SER_Init(void) {
    RCC->APB1ENR |= (1UL << 19); RCC->AHB1ENR |= (1UL << 2);
    GPIOC->MODER &= ~(0xF << 20); GPIOC->MODER |= (0xA << 20);
    GPIOC->AFR[1] |= (0x8 << 8) | (0x8 << 12);
    UART4->BRR = 0x1117; 
    UART4->CR1 = 0x2024; 
    NVIC_EnableIRQ(UART4_IRQn);
}

void DrawVerticalBars(uint32_t gas, uint32_t sound) {
    const uint8_t MAX_H = 45; const uint8_t BOT_Y = 63;
    uint32_t hGas = (gas * MAX_H) / 4096; if(hGas > MAX_H) hGas = MAX_H;
    uint32_t hSnd = (sound * 30 * MAX_H) / 4096; if(hSnd > MAX_H) hSnd = MAX_H;

    
    for (uint8_t x = 112; x < 118; x++) 
        for (uint8_t i = 0; i < hGas; i++) SSD1306_DrawPixel(x, BOT_Y - 4 - i, 1);
        
    
    for (uint8_t x = 122; x < 128; x++) 
        for (uint8_t i = 0; i < hSnd; i++) SSD1306_DrawPixel(x, BOT_Y - 4 - i, 1);
        
    
    for(uint8_t y = BOT_Y - MAX_H - 4; y < 64; y++) SSD1306_DrawPixel(120, y, 1);
}