#include <stdint.h>


void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		__asm volatile("nop");
	}
}

void delay_us(uint32_t us) { //8 ticks/iteration
	__asm volatile (
		"push {r0}\r\n"
		"mov R0, %0\r\n"
		"_loop:\r\n" //approx. 6ticks/iteration
			"cmp R0, #0\r\n"     //1
			"beq _exit\r\n"      //1 or 1+P (when condition is True)
			"sub R0, R0, #1\r\n" //1
			"nop\r\n" //1 allignment
			"b _loop\r\n" //1+P (pipeline refill) ~4 cycle
		"_exit:\r\n"
		"pop {r0}\r\n"
		:: "r"(9 * us) //for 72Mhz
	);
}

uint8_t bitrev(uint8_t b){
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}
