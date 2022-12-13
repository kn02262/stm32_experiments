#include <seglcd.h>
#include <stdint.h>
#include <stm32f10x.h>
#include <spi.h>

void cmd(uint8_t data){
    GPIOA->ODR &= ~GPIO_ODR_ODR3; // A0=0 --a command is being sent
    GPIOA->ODR &= ~GPIO_ODR_ODR4; // CS=0
    SPI1_Write(data);
    //delay(500);
    while (SPI1->SR & SPI_SR_BSY);
    GPIOA->ODR |= GPIO_ODR_ODR4; // CS=1
}

void dat(uint8_t data){
    GPIOA->ODR |= GPIO_ODR_ODR3; // A0=1 --data is being sent
    GPIOA->ODR &= ~GPIO_ODR_ODR4; // CS=0
    //delay(1000);
    SPI1_Write(data);
    while (SPI1->SR & SPI_SR_BSY); //delay(500); // Important!
    GPIOA->ODR |= GPIO_ODR_ODR4; // CS=1
}

void Seglcd_init(){
// Draw tetris display separator

// Draw vertical lines
for(int p=2; p<=7; p+=5){
	cmd(0xB0 | p); // Set page p
	cmd(0x10 | 0x00); // Set column address MSB (0x00...0x0F)
	cmd(0x01); // Set column address LSB (0x00...0x0F)
	for(int i=0; i<DOT_SZ*30; i++){
			dat(0b10000000);
	}
}

// Draw horizontal lines
for(int p=3; p<=7; p++){
	cmd(0xB0 | p); // Set page p
	cmd(0x10 | 0x00); // Set column address = 0
	cmd(0x00);
	dat(0xFF);
	cmd(0x10 | 0b0111); // Set column address = 120
	cmd(0b1000);
	dat(0xFF);
}



}



