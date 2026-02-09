/* NODE 1: SENDER  */
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>




volatile uint32_t msTicks = 0;
float live_Temp = 0.0f;
float live_Press = 0.0f;
uint32_t live_Gas = 0;
uint32_t live_Sound = 0;
uint8_t live_Vibration = 0;


typedef struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} BMP280_CalibData;
BMP280_CalibData calibData;
int32_t t_fine;


void SystemClock_Config(void);
void SysTick_Init(void);
void GPIO_Init(void);
void SPI1_Init(void);
void ADC1_Init(void);
void SER_Init(void);
void SER_SendString(char* str);
void Delay(uint32_t ms);


uint8_t SPI1_Transfer(uint8_t data);
void BMP280_WriteReg(uint8_t reg, uint8_t val);
void BMP280_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);
uint16_t ADC1_Read(uint8_t channel);
uint32_t Get_Sound_Amplitude(void);
uint8_t Read_Vibration(void);
void BMP280_Init_Sensor(void);
float BMP280_GetTemp(void);
float BMP280_GetPress(void);

int main(void) {
    
    SystemClock_Config();
    SysTick_Init();
    GPIO_Init();
    SPI1_Init();
    ADC1_Init();
    SER_Init(); 
    
    
    BMP280_Init_Sensor();
    
    char xbeeBuf[64];
    uint32_t lastTx = 0;

    while(1) {
        
        live_Temp = BMP280_GetTemp();
        live_Press = BMP280_GetPress();
        live_Gas = ADC1_Read(1);      
        live_Sound = Get_Sound_Amplitude(); 
        live_Vibration = Read_Vibration(); 
        
        
        if (msTicks - lastTx >= 500) {
            lastTx = msTicks;
            
            
            sprintf(xbeeBuf, "%.2f,%.1f,%u,%u,%d\n", 
                    live_Temp, live_Press, live_Gas, live_Sound, live_Vibration);
            
            SER_SendString(xbeeBuf);
            
            
            GPIOD->ODR ^= (1 << 12); 
        }
    }
}




void SER_Init(void) {
    RCC->APB1ENR |= (1UL << 19); 
    RCC->AHB1ENR |= (1UL << 2);  
    GPIOC->MODER &= ~(0xF << 20); GPIOC->MODER |= (0xA << 20); 
    GPIOC->AFR[1] |= (0x8 << 8) | (0x8 << 12);
    UART4->BRR = 0x1117; 
    UART4->CR1 = 0x2008;  
}

void SER_SendString(char* str) {
    while (*str) {
        
        while (!(UART4->SR & (1<<7))) {} 
        UART4->DR = *str++;
    }
}


void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {} 
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
    RCC->PLLCFGR = (8 << 0) | (336 << 6) | (0 << 16) | (7 << 24) | RCC_PLLCFGR_PLLSRC_HSE;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {} // FIXED
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {} 
}
void SysTick_Init(void) { SysTick_Config(168000000 / 1000); }
void SysTick_Handler(void) { msTicks++; }
void Delay(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) {} 
}
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    
    GPIOA->MODER |= (2<<10)|(2<<12)|(2<<14); GPIOA->OSPEEDR |= (3<<10)|(3<<12)|(3<<14); GPIOA->AFR[0] |= (5<<20)|(5<<24)|(5<<28);
    
    GPIOA->MODER |= (1<<8); GPIOA->ODR |= (1<<4);
    
    GPIOA->MODER |= (3 << 2); GPIOA->MODER |= (3 << 0);
    
    GPIOC->MODER |= (1<<0); GPIOD->MODER |= (1<<24);
    
    GPIOB->MODER &= ~(3U << 0); GPIOB->PUPDR |= (1U << 0);
}
void SPI1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (4<<3); SPI1->CR1 |= SPI_CR1_SPE;
}
void ADC1_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; ADC->CCR |= (1<<16); 
    ADC1->SMPR2 |= (4<<0); ADC1->SMPR2 |= (4<<3); ADC1->CR2 |= ADC_CR2_ADON;
}
uint16_t ADC1_Read(uint8_t channel) {
    ADC1->SQR3 = channel; ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) {} 
    return ADC1->DR;
}
uint32_t Get_Sound_Amplitude(void) {
    uint32_t min = 4095; uint32_t max = 0;
    for(int i=0; i<1000; i++) {
        uint16_t val = ADC1_Read(0);
        if(val < min) min = val; if(val > max) max = val;
    }
    return (max - min);
}
uint8_t Read_Vibration(void) { return (GPIOB->IDR & (1 << 0)) ? 1 : 0; }
uint8_t SPI1_Transfer(uint8_t data) {
    SPI1->DR = data; 
    while (!(SPI1->SR & SPI_SR_TXE)) {}
    while (!(SPI1->SR & SPI_SR_RXNE)) {} 
    while (SPI1->SR & SPI_SR_BSY) {} 
    return SPI1->DR;
}
void BMP280_WriteReg(uint8_t reg, uint8_t val) {
    GPIOA->BSRR = (1<<20); SPI1_Transfer(reg & 0x7F); SPI1_Transfer(val); GPIOA->BSRR = (1<<4);
}
void BMP280_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len) {
    GPIOA->BSRR = (1<<20); SPI1_Transfer(reg | 0x80); for(int i=0; i<len; i++) buf[i] = SPI1_Transfer(0x00); GPIOA->BSRR = (1<<4);
}
void BMP280_Init_Sensor(void) {
    uint8_t b[24]; BMP280_ReadRegs(0x88, b, 24);
    calibData.dig_T1 = (b[1]<<8)|b[0]; calibData.dig_T2 = (b[3]<<8)|b[2]; calibData.dig_T3 = (b[5]<<8)|b[4];
    calibData.dig_P1 = (b[7]<<8)|b[6]; calibData.dig_P2 = (b[9]<<8)|b[8]; calibData.dig_P3 = (b[11]<<8)|b[10];
    calibData.dig_P4 = (b[13]<<8)|b[12]; calibData.dig_P5 = (b[15]<<8)|b[14]; calibData.dig_P6 = (b[17]<<8)|b[16];
    calibData.dig_P7 = (b[19]<<8)|b[18]; calibData.dig_P8 = (b[21]<<8)|b[20]; calibData.dig_P9 = (b[23]<<8)|b[22];
    BMP280_WriteReg(0xF5, 0x90); BMP280_WriteReg(0xF4, 0x57); 
}
float BMP280_GetTemp(void) {
    uint8_t r[3]; BMP280_ReadRegs(0xFA, r, 3);
    int32_t adc = (r[0]<<12)|(r[1]<<4)|(r[2]>>4);
    double v1 = (((double)adc)/16384.0 - ((double)calibData.dig_T1)/1024.0) * ((double)calibData.dig_T2);
    double v2 = ((((double)adc)/131072.0 - ((double)calibData.dig_T1)/8192.0) * (((double)adc)/131072.0 - ((double)calibData.dig_T1)/8192.0)) * ((double)calibData.dig_T3);
    t_fine = (int32_t)(v1 + v2);
    return (float)((v1 + v2) / 5120.0);
}
float BMP280_GetPress(void) {
    BMP280_GetTemp(); uint8_t r[3]; BMP280_ReadRegs(0xF7, r, 3);
    int32_t adc = (r[0]<<12)|(r[1]<<4)|(r[2]>>4);
    double v1 = ((double)t_fine/2.0) - 64000.0;
    double v2 = v1 * v1 * ((double)calibData.dig_P6) / 32768.0;
    v2 = v2 + v1 * ((double)calibData.dig_P5) * 2.0;
    v2 = (v2/4.0) + (((double)calibData.dig_P4) * 65536.0);
    v1 = (((double)calibData.dig_P3) * v1 * v1 / 524288.0 + ((double)calibData.dig_P2) * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * ((double)calibData.dig_P1);
    if (v1 == 0.0) return 0;
    double p = 1048576.0 - (double)adc;
    p = (p - (v2 / 4096.0)) * 6250.0 / v1;
    v1 = ((double)calibData.dig_P9) * p * p / 2147483648.0;
    v2 = p * ((double)calibData.dig_P8) / 32768.0;
    p = p + (v1 + v2 + ((double)calibData.dig_P7)) / 16.0;
    return (float)p;
}