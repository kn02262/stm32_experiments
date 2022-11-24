#ifndef __SPI_CMSIS_H__
#define __SPI_CMSIS_H__

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>

void SPI1_Init(void);

void SPI1_Write(uint8_t data);

uint8_t SPI1_Read(void);

#endif