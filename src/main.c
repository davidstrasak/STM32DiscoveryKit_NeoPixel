#include "stm32f100xb.h"
#include "stdint.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void Delay(uint32_t nCount);
void Timer_Configuration(void);

#define LEFT_LED_ON 0
#define LEFT_LED_OFF 1
#define RIGHT_LED_ON 2
#define RIGHT_LED_OFF 3
uint8_t state = LEFT_LED_ON;

int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    Timer_Configuration();

    while (1)
    {

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

void Timer_Configuration(void) {
    RCC->APB2ENR |= (1 << 18); // Enable the clock for timer 17
    // Timer 17 configuration - the timer that switches the LED lights on the board
    TIM17->PSC = 2399; // 500kHz timer freq
    TIM17->ARR = 9999; // Counts for 1 second
    TIM17->DIER |= (1); // Enables the interrupt when counter overflows
    NVIC->ISER[0] |= (1 << TIM1_TRG_COM_TIM17_IRQn);


    TIM17->CR1 |= 1; // Enable the counter
}
/*Delay_ms smycka zpozduje zhruba o nCount 1 ms*/
void Delay(uint32_t nCount)
{
    for (volatile uint32_t i = 0; nCount != 0; nCount--) {
        for (i = 2000; i != 0; i--);
    }
}

void TIM1_TRG_COM_TIM17_IRQHandler(void) {
    TIM17->SR &= 0;
    switch (state) {
    case LEFT_LED_ON:
        GPIOC->BSRR |= (1 << (8));
        state = LEFT_LED_OFF;
        break;
    case LEFT_LED_OFF:
        GPIOC->BSRR |= (1 << (8 + 16));
        state = RIGHT_LED_ON;
        break;
    case RIGHT_LED_ON:
        GPIOC->BSRR |= (1 << (9));
        state = RIGHT_LED_OFF;
        break;
    case RIGHT_LED_OFF:
        GPIOC->BSRR |= (1 << (9 + 16));
        state = LEFT_LED_ON;
        break;
    default:
        state = LEFT_LED_ON;
        break;
    }
}