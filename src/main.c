#include "stm32f100xb.h"
#include "stdint.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void Delay(uint32_t nCount);



int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();

    // full red neopixel
    for (int i = 0; i < 9; i++) {
        GPIOC->BSRR = (1 << (7));
        Delay(8);
        GPIOC->BSRR = (1 << (7 + 16));
        Delay(3);
    }
    // no green
    for (int i = 0; i < 9; i++) {
        GPIOC->BSRR = (1 << (7));
        Delay(3);
        GPIOC->BSRR = (1 << (7 + 16));
        Delay(8);
    }
    // no blue
    for (int i = 0; i < 9; i++) {
        GPIOC->BSRR = (1 << (7));
        Delay(3);
        GPIOC->BSRR = (1 << (7 + 16));
        Delay(8);
    }
    GPIOC->BSRR = (1 << (7 + 16));


    while (1)
    {
        GPIOC->BSRR |= (1 << (8));
        Delay(5000);
        GPIOC->BSRR |= (1 << (8 + 16));
        Delay(5000);
        GPIOC->BSRR |= (1 << (9));
        Delay(5000);
        GPIOC->BSRR |= (1 << (9 + 16));
        Delay(5000);
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
    RCC->CR |= 0x10000; //HSE on
    while (!(RCC->CR & 0x20000)) {}
    //flash access setup
    FLASH->ACR &= 0x00000038;   //mask register
    FLASH->ACR |= 0x00000002;   //flash 2 wait state

    FLASH->ACR &= 0xFFFFFFEF;   //mask register
    FLASH->ACR |= 0x00000010;   //enable Prefetch Buffer

    RCC->CFGR &= 0xFFC3FFFF;//maskovani PLLMUL
    RCC->CFGR |= 0x1 << 18;//Nastveni PLLMUL 3x
    RCC->CFGR |= 0x0 << 17;//nastaveni PREDIV1 1x
    RCC->CFGR |= 0x10000;//PLL bude clocovan z PREDIV1
    RCC->CFGR &= 0xFFFFFF0F;//HPRE=1x
    RCC->CFGR &= 0xFFFFF8FF;//PPRE2=1x
    RCC->CFGR &= 0xFFFFC7FF;//PPRE2=1x

    RCC->CR |= 0x01000000;//PLL on
    while (!(RCC->CR & 0x02000000)) {}//PLL stable??

    RCC->CFGR &= 0xFFFFFFFC;
    RCC->CFGR |= 0x2;//nastaveni PLL jako zdroj hodin pro SYSCLK

    while (!(RCC->CFGR & 0x00000008))//je SYSCLK nastaveno?
    {
    }

    RCC->APB2ENR |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5);
}

/*Delay smycka zpozduje zhruba o nCount 100 nanosekund*/
void Delay(uint32_t nCount)
{
    for (volatile uint32_t i = 0; nCount != 0; nCount--) {
        for (i = 600; i != 0; i--);
    }
}