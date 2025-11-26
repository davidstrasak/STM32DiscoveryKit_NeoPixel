#include "stdint.h"
#include "stm32f100xb.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void Delay(uint32_t nCount);
void BlinkTimer_Configuration(void);
void NeoPixelTimerAndDMA_Configuration(void);
void setNeoPixelValue(uint8_t greenVal, uint8_t redVal, uint8_t blueVal);

// State machine defines for the blinking LEDs on the board
#define LEFT_LED_ON 0
#define LEFT_LED_OFF 1
#define RIGHT_LED_ON 2
#define RIGHT_LED_OFF 3
uint8_t state;

// NeoPixel definitions
uint16_t neopixel_buffer[25];
// NeoPixel PWM values
#define NEOPIXEL_ZERO 8
#define NEOPIXEL_ONE 17

int main(void) {
   RCC_Configuration();
   GPIO_Configuration();
   BlinkTimer_Configuration();
   NeoPixelTimerAndDMA_Configuration();

   volatile uint8_t redVal = 255;
   volatile uint8_t greenVal = 0;
   volatile uint8_t blueVal = 0;

   while (1) {
      while (DMA1_Channel6->CCR & 1) {
      }

      // Phase 1: Red -> Yellow (Green fades in)
      if (redVal == 255 && blueVal == 0 && greenVal < 255) {
         greenVal++;
      }
      // Phase 2: Yellow -> Green (Red fades out)
      else if (greenVal == 255 && blueVal == 0 && redVal > 0) {
         redVal--;
      }
      // Phase 3: Green -> Cyan (Blue fades in)
      else if (greenVal == 255 && redVal == 0 && blueVal < 255) {
         blueVal++;
      }
      // Phase 4: Cyan -> Blue (Green fades out)
      else if (blueVal == 255 && redVal == 0 && greenVal > 0) {
         greenVal--;
      }
      // Phase 5: Blue -> Magenta (Red fades in)
      else if (blueVal == 255 && greenVal == 0 && redVal < 255) {
         redVal++;
      }
      // Phase 6: Magenta -> Red (Blue fades out)
      else if (redVal == 255 && greenVal == 0 && blueVal > 0) {
         blueVal--;
      }
      Delay(1);

      setNeoPixelValue(redVal / 20, greenVal / 20,
                       blueVal / 20);  // dividing so it's less bright
   }
}

// GPIO configuration
void GPIO_Configuration(void) {
   // Enable clocku do portu A, B, C, D
   RCC->APB2ENR |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5);

   // PC6 - DIN neopixel LED
   GPIOC->CRL &= ~(0xF << 24);
   GPIOC->CRL |= (0b11 << 24);  // output 50MHz
   GPIOC->CRL |= (0b10 << 26);  // PP - alternate function
   // PC8 - Prvni LED na desce
   GPIOC->CRH &= ~(0xF << 0);
   GPIOC->CRH |= (0b11 << 0);  // output 50MHz
   // PC9 - Druha LED na desce
   GPIOC->CRH &= ~(0xF << 4);
   GPIOC->CRH |= (0b11 << 4);  // output 50MHz
}

// Basic clock configuration
void RCC_Configuration(void) {
   RCC->CR |= 0x10000;  // HSE on - External high speed clock enabled
   while (!(RCC->CR & 0x20000)) {
   }  // HSE ready - waiting until the external clock is ready

   // //flash access setup
   // FLASH->ACR &= 0x00000038;   //mask register
   // FLASH->ACR |= 0x00000002;   //flash 2 wait state

   // FLASH->ACR &= 0xFFFFFFEF;   //mask register
   // FLASH->ACR |= 0x00000010;   //enable Prefetch Buffer

   // Select external clock for PLL
   RCC->CFGR |= 0x10000;
   // HPRE set to zero
   RCC->CFGR &= ~(0xF << 4);
   // Predividers
   RCC->CFGR &= ~(0b1 << 17);
   RCC->CFGR &= ~(0b111 << 8);   // set low speed clock to 1x
   RCC->CFGR &= ~(0b111 << 11);  // set high speed clock to 1x
   // PLL multiplier config
   RCC->CFGR &= ~(0b1111 << 18);
   RCC->CFGR |= (0b0001 << 18);  // 3x multiplier
   // => Total CPU freq = 24MHz

   // turning PLL on and waiting
   RCC->CR |= (1 << 24);
   while (!(RCC->CR & (1 << 25))) {
   }

   // Setting the PLL clock as the system clock and waiting
   RCC->CFGR &= ~(0b11);
   RCC->CFGR |= 0b10;
   while (!(RCC->CFGR & 0x00000008)) {
   }
}

// The configuration for the timer which blinks the LEDs on the board
void BlinkTimer_Configuration(void) {
   RCC->APB2ENR |= (1 << 18);  // Enable the clock for timer 17
   // Timer 17 configuration - the timer that switches the LED lights on the
   // board
   TIM17->PSC = 2399;   // 10kHz timer freq
   TIM17->ARR = 9999;   // Counts for 1 second
   TIM17->DIER |= (1);  // Enables the interrupt when counter overflows
   NVIC->ISER[0] |= (1 << TIM1_TRG_COM_TIM17_IRQn);  // Enable the interrupt
   NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 1);

   TIM17->CR1 |= 1;  // Enable the counter
}

/*Delay_ms smycka zpozduje zhruba o nCount 1 ms*/
void Delay(uint32_t nCount) {
   for (volatile uint32_t i = 0; nCount != 0; nCount--) {
      for (i = 2000; i != 0; i--);
   }
}

// State machine interrupt for the blinking LEDs on the board
void TIM1_TRG_COM_TIM17_IRQHandler(void) {
   TIM17->SR &= 0;  // Clear the interrupt flag

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

// The configuration for the timers which run the neopixel LED
void NeoPixelTimerAndDMA_Configuration(void) {
   RCC->APB2ENR |= (1 << 0);  // Enable alternate functions for GPIO pins
   RCC->APB1ENR |= (1 << 1);  // Enable the clock for timer3

   // Alternate function remapping setup
   AFIO->MAPR &=
       ~(0b11 << 10);  // I know this is redundant -> it's rather rigorous
   AFIO->MAPR |=
       (0b11
        << 10);  // set alternate remappings so the timer3channel1 lives at PC6

   // Output compare channel 1 setup
   TIM3->CCMR1 &= ~(
       0xFF
       << 0);  // Clear channel 2 setup of timer3 (which also sets it as output)
   TIM3->CCMR1 |= (0b1 << 3);    // Enable preload
   TIM3->CCMR1 |= (0b110 << 4);  // Enable PWM on the timer3channel1
   TIM3->CCER |= (1 << 0);       // Enable timer3channel1

   TIM3->PSC = 0;  // Prescaler set so the frequency is 24Mhz - because if I
                   // count 30 times (set through ARR) then the period is 1.25us
                   // = NeoPixel period
   TIM3->ARR = 29;  // ARR is 29 because our clock ticks 29+1 times in the time
                    // the timer ticks once (24e6 / 8e5)
   TIM3->CCR1 = 0;  // Sets the PWM value for channel1. =8 for a 0 bit value
                    // (0.35/1.25*30) or =16 for a 1 bit value (0.7/1.25*30)

   TIM3->CR1 |= (1 << 0);  // Enable the timer

   // DMA1channel6 setup
   RCC->AHBENR |= (1 << 0);  // Enable DMA1 clock
   DMA1_Channel6->CPAR =
       (uint32_t)&TIM3->CCR1;  // DMA is writing to pwm value of timer3channel1
   DMA1_Channel6->CMAR =
       (uint32_t)neopixel_buffer;  // DMA is writing stuff from this buffer
   DMA1_Channel6->CNDTR =
       25;  // 24 transfers to complete + one zero it will write straight away
   DMA1_Channel6->CCR = 0;
   DMA1_Channel6->CCR |= (1 << 1);      // Enable transfer complete interrupt
   DMA1_Channel6->CCR |= (1 << 4);      // Read from buffer and write to timer
   DMA1_Channel6->CCR |= (1 << 7);      // Memory increment mode on
   DMA1_Channel6->CCR |= (0b01 << 8);   // Peripheral size 16 bits because
                                        // TIM3->CCR1 is a 16 bit register
   DMA1_Channel6->CCR |= (0b01 << 10);  // Peripheral size is also 16 bit
                                        // because the buffer is uint16_t

   TIM3->DIER |=
       (1 << 9);  // When capture compare event happens (timer resets),
                  // DMA1 request will fill the next pwm value

   NVIC->ISER[0] |=
       (1 << DMA1_Channel6_IRQn);  // Enable the DMA1channel6 interrupt

   // DMA1_Channel6->CCR |= (1 << 0); // Enable DMA1channel6
}

void DMA1_Channel6_IRQHandler(void) {
   // this IF triggers if DMA1channel6 threw an interrupt complete
   if (DMA1->ISR & (1 << 21)) {
      DMA1->IFCR |= (1 << 21);          // Clear the interrupt flag
      DMA1_Channel6->CCR &= ~(1 << 0);  // Disable it
      DMA1_Channel6->CNDTR = 25;        // 24 transfers to complete
      // Enabling the DMA1channel6 happens after values are sent into the
      // neopixel buffer.
   }
}

void setNeoPixelValue(uint8_t redVal, uint8_t greenVal, uint8_t blueVal) {
   for (uint8_t i = 0; i < 8; i++) {
      if (redVal & 0x80) {
         neopixel_buffer[i] = NEOPIXEL_ONE;
      } else {
         neopixel_buffer[i] = NEOPIXEL_ZERO;
      }

      if (greenVal & 0x80) {
         neopixel_buffer[i + 8] = NEOPIXEL_ONE;
      } else {
         neopixel_buffer[i + 8] = NEOPIXEL_ZERO;
      }

      if (blueVal & 0x80) {
         neopixel_buffer[i + 16] = NEOPIXEL_ONE;
      } else {
         neopixel_buffer[i + 16] = NEOPIXEL_ZERO;
      }

      neopixel_buffer[24] = 0;  // setting to zero because then the PWM pulse
                                // goes to zero straight away

      redVal <<= 1;
      greenVal <<= 1;
      blueVal <<= 1;
   }

   DMA1_Channel6->CCR |= (1 << 0);  // Enable DMA
}