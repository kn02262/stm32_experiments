#include <string.h>
#include <stdio.h>
#include <stm32f10x.h>
void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		__NOP();
	}
}

uint8_t LCD_Buf[8][128];

void SPI1_Init(void)
{
  //Включаем тактирование SPI1 и GPIOA
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;
  
  /**********************************************************/
  /*** Настройка выводов GPIOA на работу совместно с SPI1 ***/
  /**********************************************************/
  //PA7 - MOSI (=SI Slave In)
  //PA6 - MISO (Free)
  //PA5 - SCK (=SCL Clock)

  // Also used with the display
  //PA4 - CS (=CS Chip Select)
  //PA3 - RS (also called A0, data=1 or command=0)
  //PA2 - RSE (Reset=0, Standby=1)
  
  //Для начала сбрасываем все конфигурационные биты в нули
  GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2 
                | GPIO_CRL_CNF3 | GPIO_CRL_MODE3
                | GPIO_CRL_CNF4 | GPIO_CRL_MODE4
				        | GPIO_CRL_CNF5 | GPIO_CRL_MODE5 
                | GPIO_CRL_CNF6 | GPIO_CRL_MODE6
                | GPIO_CRL_CNF7 | GPIO_CRL_MODE7);

  //Настраиваем
  //SCK: MODE5 = 0x03 (11b); CNF5 = 0x02 (10b)
  GPIOA->CRL |= GPIO_CRL_MODE5 | GPIO_CRL_CNF5_1;
  
  //MISO: MODE6 = 0x00 (00b); CNF6 = 0x01 (01b)
  GPIOA->CRL |= GPIO_CRL_CNF6_0;
  
  //MOSI: MODE7 = 0x03 (11b); CNF7 = 0x02 (10b)
  GPIOA->CRL |= GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1;

  //CS: MODE4 = 0x03 (11b); CNF4 = 0x00 (00b)
  GPIOA->CRL |= GPIO_CRL_MODE4;

  //RS: MODE3 = 0x03 (11b); CNF3 = 0x00 (00b)
  GPIOA->CRL |= GPIO_CRL_MODE3;

  //RESET: MODE2 = 0x03 (11b); CNF2 = 0x00 (00b)
  GPIOA->CRL |= GPIO_CRL_MODE2;
  GPIOA->ODR |= GPIO_ODR_ODR2; // Set 'RESET' high (Standby mode)

  /**********************/
  /*** Настройка SPI1 ***/
  /**********************/
  
  SPI1->CR1 &= ~SPI_CR1_DFF; // DFF=0 Размер кадра 8 бит
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // LSBFIRST=0 MSB First
  SPI1->CR1 &= ~SPI_CR1_CRCEN; // Disable CRC
  SPI1->CR1 |= SPI_CR1_SSM; // Программное управление SS
  SPI1->CR1 |= SPI_CR1_SSI; // SS в высоком состоянии
  //SPI1->CR1 &= ~SPI_CR1_BR; // Clear BR[2:0] bits
  //SPI1->CR1 |= SPI_CR1_BR_2; // BR[2:0]=100, Скорость передачи: F_PCLK/32
  SPI1->CR1 |= SPI_CR1_BR;
  SPI1->CR1 |= SPI_CR1_MSTR; // Режим Master (ведущий)
  SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); //Режим работы SPI: CPOL=0 CPHA=0
  
  SPI1->CR1 |= SPI_CR1_SPE; //Включаем SPI
}

void SPI1_Write(uint8_t data)
{
  //Ждем, пока не освободится буфер передатчика
  while(!(SPI1->SR & SPI_SR_TXE));
  
  //заполняем буфер передатчика
  SPI1->DR = data;
}

uint8_t SPI1_Read(void)
{
  SPI1->DR = 0; //запускаем обмен
  
  //Ждем, пока не появится новое значение 
  //в буфере приемника
  while(!(SPI1->SR & SPI_SR_RXNE));
  
  //возвращаем значение буфера приемника
  return SPI1->DR;
}

void cmd(uint8_t data){
    GPIOA->ODR &= ~GPIO_ODR_ODR3; // A0=0 --a command is being sent
    GPIOA->ODR &= ~GPIO_ODR_ODR4; // CS=0
    //delay(1000);
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

void DrawPixel(uint8_t x, uint8_t y){ // x in [0..63], y in [0..127]
  uint8_t page_address = (x >> 3) & 0b00000111; // %8 guarantees page_address in [0..7]
  cmd(0xB0 | page_address); // Set Page page_address (Pages 0x00...0x0F)
  cmd(y & 0b00001111); // Set column address LSB
  cmd(0b00010000 | (y >> 4));
  dat(1 << (x % 8));
}

void DrawChess(){
  uint8_t i,j,k; // i=row number, j=col number, k=cycle inside a square
  uint8_t val = 0x00; // Color 0x00=White strip, 0xFF=black strip
  for(i=0; i<=7; i++){
    cmd(0xB0 | i); // Set Page i (Pages 0x00...0x0F)
    val = ~ val;
    for(j=0; j<=15; j++){
      for(k=0; k<=7; k++) dat(val);
      val = ~val;
    }
    cmd(0xEE);
  }
}

int main(void) {
	// Enable clock for GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	// Enable PC13 push-pull mode
	GPIOC->CRH &= ~GPIO_CRH_CNF13; //clear cnf bits
	GPIOC->CRH |= GPIO_CRH_MODE13_0; //Max speed = 10Mhz

  SPI1_Init();
  GPIOA->ODR &= ~GPIO_ODR_ODR4; // CS=0
  GPIOA->ODR &= ~GPIO_ODR_ODR2; // RESET=0
  delay(10000); // Wait for the power stabilized
  GPIOA->ODR |= GPIO_ODR_ODR2; // RESET=1
  delay(1000);

  cmd(0xA2); //LCD Drive set 1/9 bias
  cmd(0xA0); // RAM Address SEG Output normal
  cmd(0xC8); // Common outout mode selection
  cmd(0x28 | 0x07); // Power control mode
  cmd(0b00100000 | 0x05); // Voltage regulator
  cmd(0xA6); // Normal color, A7 = inverse color
  cmd(0xAF); // Display on
  cmd(0x40 | 0x00); // Set start line address (Lines 0x00...0x3F)
  memset(LCD_Buf, 0, sizeof(int8_t) * 8 * 128);
  for(int k=0; k<=7; k++){ // Clear DRAM
    cmd(0xB0 | k); // Set Page 0 (Pages 0x00...0x0F)
    for(int i=0; i<=127; i++){
        dat(LCD_Buf[k][i]);
    }
    cmd(0xEE); // End writing to the page, return the page address back
  }

  // Setting the page address:
  //cmd(0xB0 | 0x00); // Set Page 0 (Pages 0x00...0x0F)
  //
  // Setting the line address:
  //cmd(0x10 | 0x00); // Set column address MSB (0x00...0x0F)
  //cmd(0x00); // Set column address LSB (0x00...0x0F)

  /*for(int i=0; i<64; i++)
    for(int j=0; j<128; j++)
      DrawPixel(i,j);
  */

  DrawChess();
  
  //delay(10000000);
  //cmd(0x40 | 0x01); // Set start line address (Lines 0x00...0x3F)
  
    while (1) { // LED blinking
	    GPIOC->ODR |= (1U<<13U); //U -- unsigned suffix (to avoid syntax warnings in IDE)
		delay(1000000);
	    GPIOC->ODR &= ~(1U<<13U);
	    delay(1000000);
    }
}