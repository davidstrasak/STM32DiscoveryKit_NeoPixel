#include "stm32f100xb.h"
#include "stdint.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void Delay(uint32_t nCount);


int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();

    while (1)
    {
        GPIOC->BSRR |= (1 << (8));
        Delay(1000);
        GPIOC->BSRR |= (1 << (8 + 16));
        Delay(1000);
        GPIOC->BSRR |= (1 << (9));
        Delay(1000);
        GPIOC->BSRR |= (1 << (9 + 16));
        Delay(1000);
    }
}

void GPIO_Configuration(void) {
    // PC7 - DIN neopixel LED
    GPIOC->CRL &= 0x0FFFFFFF;
    GPIOC->CRL |= (3 << 28);  // PP output
    // PC8 - Prvni LED na desce
    GPIOC->CRH &= ~(0xF << 0);
    GPIOC->CRH |= (3 << 0);  // PP output
    // PC9 - Druha LED na desce
    GPIOC->CRH &= ~(0xF << 4);
    GPIOC->CRH |= (3 << 4);  // PP output
}

void RCC_Configuration(void) {
    RCC->CR |= 0x10000; //HSE on - External high speed clock enabled
    while (!(RCC->CR & 0x20000)) {} // HSE ready - waiting until the external clock is ready

    // //flash access setup
    // FLASH->ACR &= 0x00000038;   //mask register
    // FLASH->ACR |= 0x00000002;   //flash 2 wait state

    // FLASH->ACR &= 0xFFFFFFEF;   //mask register
    // FLASH->ACR |= 0x00000010;   //enable Prefetch Buffer

    // Select external clock for PLL
    RCC->CFGR |= 0x10000;
    // HPRE set to zero
    RCC->CFGR &= ~(0xF << 4);
    //Predividers
    RCC->CFGR &= ~(0b1 << 17);
    RCC->CFGR &= ~(0b111 << 8);     // set low speed clock to 1x
    RCC->CFGR &= ~(0b111 << 11);    // set high speed clock to 1x
    // PLL multiplier config
    RCC->CFGR &= ~(0b1111 << 18);
    RCC->CFGR |= (0b0001 << 18);    // 3x multiplier
    // => Total CPU freq = 24MHz

    // turning PLL on and waiting
    RCC->CR |= (1 << 24);
    while (!(RCC->CR & (1 << 25))) {}

    // Setting the PLL clock as the system clock and waiting
    RCC->CFGR &= ~(0b11);
    RCC->CFGR |= 0b10;
    while (!(RCC->CFGR & 0x00000008)) {}

    // Enable clocku do portu A, B, C, D
    RCC->APB2ENR |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5);
}

/*Delay_ms smycka zpozduje zhruba o nCount 1 ms*/
void Delay(uint32_t nCount)
{
    for (volatile uint32_t i = 0; nCount != 0; nCount--) {
        for (i = 2000; i != 0; i--);
    }
}