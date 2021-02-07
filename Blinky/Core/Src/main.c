#include "stm32f446xx.h"

// toggle the ODR for pin 5
void toggle_pin_5(void) {
	GPIOA->ODR ^= (0x1<<5);
}

// burn cpu cycles for time ms
void delay_ms(uint32_t time) {
	for (int i=0; i< time*1000+(time/4); i++) {
		// do nothing
	}
}

void enable_GPIOA(void) {
	//enable the  GPIOA clk
	 	RCC->AHB1ENR |= 0x1;

		// enable the GPIOA to be digital out
		GPIOA->MODER &= ~(3<<10); 	//0xFFFF_F3FF clears the bits
		GPIOA->MODER |= 1<<10;		//sets bit 10 to 1
}

int main(void) {

	enable_GPIOA();

	while(1) {
		toggle_pin_5();
		delay_ms(500);
	}
}

