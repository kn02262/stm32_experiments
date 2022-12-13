#ifndef __SEGLCD_H__
#define __SEGLCD_H__
#include <stdint.h>
#include <stdbool.h>

extern bool SegBuf[20][10];
#define DOT_SZ 4

void cmd(uint8_t data);
void dat(uint8_t data);

void Seglcd_init();

#endif
