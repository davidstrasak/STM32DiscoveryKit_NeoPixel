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
        Delay(500);
        GPIOC->BSRR |= (1 << (8 + 16));
        Delay(500);
        GPIOC->BSRR |= (1 << (9));
        Delay(500);
        GPIOC->BSRR |= (1 << (9 + 16));
        Delay(500);
    }
}

void GPIO_Configuration(void) {
    // PA10 - Napajeni pro level displej
    GPIOA->CRH &= ~(0xF << 8);
    GPIOA->CRH |= (3 << 8);  // PP output
    // PB5 - DSA pro pravej displej
    GPIOB->CRL &= ~(0xF << 20);
    GPIOB->CRL |= (3 << 20);  // PP output
    // PB6 - DSB pro pravej displej
    GPIOB->CRL &= ~(0xF << 24);
    GPIOB->CRL |= (3 << 24);  // PP output
    // PB7 - Tlacitko INCREASE (s externim pull-up)
    GPIOB->CRL &= 0x0FFFFFFF;
    GPIOB->CRL |= (0x4 << 28);  // Floating input
    // PB8 - Tlacitko DECREASE (s externim pull-up)
    GPIOB->CRH &= ~(0xF << 0);
    GPIOB->CRH |= (0x4 << 0);  // Floating input
    // PB9 - Tlacitko CONFIRM (s externim pull-up)
    GPIOB->CRH &= ~(0xF << 4);
    GPIOB->CRH |= (0x4 << 4);  // Floating input
    // PC6 - DSA pro levy displej
    GPIOC->CRL &= ~(0xF << 24);
    GPIOC->CRL |= (3 << 24);  // PP output
    // PC7 - DSB pro levy displej
    GPIOC->CRL &= 0x0FFFFFFF;
    GPIOC->CRL |= (3 << 28);  // PP output
    // PC8 - LED pro simulaci rele
    GPIOC->CRH &= ~(0xF << 0);
    GPIOC->CRH |= (3 << 0);  // PP output
    //PC9 - Druha LED na desce
    GPIOC->CRH &= ~(0xF << 4);
    GPIOC->CRH |= (3 << 4);  // PP output
    // PC12 - Clock
    GPIOC->CRH &= ~(0xF << 16);
    GPIOC->CRH |= (3 << 16);  // PP output
    // PD2 - Napajeni pro pravej displej
    GPIOD->CRL &= ~(0xF << 8);
    GPIOD->CRL |= (3 << 8);  // PP output
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

/*Delay smycka zpozduje zhruba o nCount milisekund*/
void Delay(uint32_t nCount)
{
    volatile uint32_t i;
    for (; nCount != 0; nCount--) {
        for (i = 6000; i != 0; i--);
    }
}