/**
 * Lab 4: using interrupts
 * intended to run on the STMnucleof411re board
 */
#include "stm32f4xx.h"
#include "common_functions.h"

void set_sysTick_interrupt(void);
void handle_input(char c);

int output_pins[10] = {0,1,2,4,5,6,7,8,9,10}; // PB pin numbers mapped to the 10 segment display
int pins_length = sizeof(output_pins)/sizeof(output_pins[0]);


int main(void) {

	// Req 1: •	The on-board LED blinks every second based on SysTick timer running in interrupt mode.
	LED_init();
	__disable_irq(); // disables the global interrupt request
	set_sysTick_interrupt();
	// END Req 1

	/* Req 2: The USART2 triggers an interrupt whenever a character is received from the PC. The acceptable
	 * characters are ‘0’ to ‘9’ where each lights a single LED of the ‘0’ to ‘9’ in the 10-segment bargraph.
	 * Any other character should turn off all LEDs.
	 */
	// enable GPIOC clk
	RCC->AHB1ENR |= 2;

	RCC->AHB1ENR |=  1;             /* enable GPIOA clock */

	// configure all PB pins in output_pins to analog out
	for (int i=0; i<pins_length; i++) {
		PXx_OUT_MODER_config(output_pins[i],'B');
	}

	UART2_tx_init();
	UART2_rx_init();
	UART2_init();

	// set UART2 up for RX interrupt
	USART2->CR1 |= 0x0020; 			// enables the rx interrupt
	NVIC_EnableIRQ(USART2_IRQn);	// enable interrupt in NVIC

	__enable_irq();

	// END Req 2

	// REQ3:

	//END REQ3


	while(1) {

//		if ((USART2->SR)&(1<<5))  { // Check if there is a packet received
//
//			char input = (USART2->DR)&0xff;    // Reading the packet will automatically clear the RXNE
//			handle_input(input);
//		}

	}
}

/**
 * Sets up the SysTick interrupt to fire every 1 sec
 */
void set_sysTick_interrupt(void) {
	// config SysTick to be in interrupt mode
	SysTick->LOAD = 16000000-1; // set reload the number to 16 meg. This will run out once every second
	SysTick->VAL = 0;
	SysTick->CTRL = 0x7;			//enables the SysTick interrupt
}


// Interrupt Handlers

// Note I had to comment out the stm32f4xx_it.c
void SysTick_Handler(void) {
	toggle_pin_5();
	set_sysTick_interrupt();
}

// default interrupt handler - should trigger when rx packet is recieved
void USART2_IRQHandler(void) {
	if ((USART2->SR)&(1<<5))  { // Check if there is a packet received
		char input = (USART2->DR)&0xff;    // Reading the packet will automatically clear the RXNE
		handle_input(input);
	}
}

void handle_input(char c) {
	if (c >= '0' && c <='9') {
		int input_num = (int)(c - '0');
		int led_num = output_pins[input_num];
		turn_on_PXnum(led_num, 'B');
	} else {
		for(int i=0; i < pins_length; i++) {
			turn_off_PXnum(output_pins[i], 'B');
		}
	}
}




