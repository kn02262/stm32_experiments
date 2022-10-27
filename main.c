#include <stdint.h>
#include "stm32f10x.h"

int main(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0; //PC13 = output
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF14 | GPIO_CRH_MODE14) | GPIO_CRH_CNF14_1; //PC14 = Input
	GPIOC->ODR |= GPIO_ODR_ODR14; // Enable PC14 Pull-up
	uint32_t i;
	uint8_t button_state = 0xFF;
    
    while(1)
    {
		if(!(GPIOC->IDR & GPIO_IDR_IDR14)){ // Button is pressed
			button_state = ~button_state;
			for(i=0; i<300000; i++){ __NOP(); }; // Debouncing
			while(!(GPIOC->IDR & GPIO_IDR_IDR14)); // Wait the button to be released
		}

		if(button_state){
			GPIOC->ODR &= ~GPIO_ODR_ODR13;
			for(i=0; i<300000; i++){ __NOP(); };

			GPIOC->ODR |= GPIO_ODR_ODR13;
			for(i=0; i<300000; i++){ __NOP(); };
		}

    }
}