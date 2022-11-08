#include <stdint.h>
#include "stm32f10x.h"

/* Interrupt handler */
void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_CC1IF) { //Compare
		GPIOC->BSRR = (1 << 13) << 16; //reset
		TIM2->SR &= ~TIM_SR_CC1IF;
	}
	if (TIM2->SR & TIM_SR_UIF) { //Overflow
		GPIOC->BSRR = (1 << 13); //set
		/* Toggle GPIO here */
		//GPIOC->ODR = ~(GPIOC->ODR & GPIO_ODR_ODR13) & GPIO_ODR_ODR13;
 		//Clear Interrupt flag
		TIM2->SR &= ~TIM_SR_UIF;
	}
}

int main(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0; //PC13 = output

	/* Main code */
   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
   RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
   RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
   TIM2->PSC = 1;//1023;
   TIM2->ARR = 4095;
   TIM2->CCR1 = 1;//между 0 и ARR
   TIM2->DIER |= TIM_DIER_UIE|TIM_DIER_CC1IE; // Enable Update Interrupt + Capture/compare
   NVIC_ClearPendingIRQ(TIM2_IRQn);
   NVIC_EnableIRQ(TIM2_IRQn); // Enable IRQ in NVIC
   TIM2->CR1 |= TIM_CR1_CEN; // Start timer
   while (1) {
       __asm volatile ("nop");
   }


}