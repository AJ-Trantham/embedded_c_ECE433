#include "stm32f4xx.h"
#include <stdio.h>

/**
 * Takes user input over UART from keyboard to change LED between Red,Green,Blue
 * Author: AJ Trantham
 */


/**
 * Configure the USART2
 * Sets the baud rate to 16,000,000
 */
void UART2_init(void) {

	// We need to setup the baud rate to 115,200bps, 8 bit, 1 stop bit, no parity, and no hw flow control
	USART2->BRR = 0x8B;
	USART2->CR1 |= (1<<3); // enable transmitter
	USART2->CR1 |= (1<<2); // enables the receiver
	USART2->CR2 = 0x0000; // sets one stop bit and all other CR2 features are left as default
	USART2->CR3 = 0x0000; // no control flow and all other features in CR3 as default

	// enable the entire USART2 module
	USART2->CR1 |= (1<<13);      // enable USART2


}

/**
 * enable and initialize the rx UART2
 */
void UART2_rx(void) {
	// set PA3 to UART2 RX
	GPIOA->MODER &= ~(0x3<<(2*3)); // clear PA3's 2 bits
	GPIOA->MODER |= (0x02<<(2*3)); // set PA3's 2 bits to AF

	//set the AF to be AF 7
	GPIOA->AFR[0] &= ~(0xF<<(4*3)); // clear PA3's 4 bits in AFR
	GPIOA->AFR[0] |= (0x7<<(4*3)); // set the 4 bits to AF 7
}

/**
 * enable and initialize the tx UART2
 */
void UART2_tx(void) {
	// Connect PA2 to UART2 TX
	//PA2 must be set to AF 7
	GPIOA->MODER &= ~(0x3<<(2*2)); // clear bits 4 and 5
	GPIOA->MODER |= (0x2<<(2*2)); // set the mode to AF ( 0x2 = 0b10 10 is the code for AF) Why is in position 4

	GPIOA->AFR[0] &= ~(0xF00); // clear the bits where we want alt 7
	GPIOA->AFR[0] = (0x700); // this sets alt 7
}

/**
 * Configures PCx (where x = pc_num) to be
 * General Purpose output mode 01
 */
void PCx_OUT_MODER_config(int pc_num) {
	GPIOC->MODER &= ~(0x3<<(2*pc_num)); // clears 2 in/out mode for PCpc_num
	GPIOC->MODER |= (1<<(2*pc_num)); // this sets bits 2 mode bits for PCpc_num General purpose output mode 01
}


void B1_config(void) {
	GPIOC->MODER &= ~(0x3<<(2*13)); // clears 2 in/out mode for PC13
	GPIOC->MODER |= (0<<(2*13)); // this sets bits 2 mode bits for PC13 to input 00
}

/**
 * Turns of PC(pc_num)
 */
void turn_off_PCx(int pc_num) {
	GPIOC->ODR = (0x0<<pc_num);
}

/**
 * // turns PC(pc_num)
 */
void turn_on_PCx(int pc_num) {
	GPIOC->ODR |= (0x1<<pc_num);
}

void delayMs(int n) {
	int i;
	for (; n > 0; n--)
		for (i = 0; i < 3195; i++);
}


int main(void) {

	// enable GPIOA CLK
	RCC->AHB1ENR |= 1;

	// need to enable GPIOC clk
	RCC->AHB1ENR |= (1<<2); // this does not override GPIOA CLK


	// enable the USART 2 clock - this is labeled as the UART2RST
	RCC->APB1ENR |= (1<<17);

	UART2_tx();
	UART2_rx();
	UART2_init();

	int txCntr = 0;
	int name_len = 12;
	char input;
	int red = 8; // red LED is connected to PC8
	int green = 6; // green LED is connected to PC6
	int blue = 5; // blue LED is connected to PC5
	char txData[12] = "AJ Trantham "; // data to send when blue C13 button is pressed

	// configure PC8, PC6, PC5 to be general purpose output
	PCx_OUT_MODER_config(red);
	PCx_OUT_MODER_config(green);
	PCx_OUT_MODER_config(blue);
	B1_config();

	while (1) {

		// write input when button is pressed
		if (!((GPIOC->IDR)&(1<<13))) {
			//if the TXE bit is 0 then Transmit Data register is NOT empty (so we need to stall the CPU)
			//if the TXE bit is 1 then Transmit Data register is empty (so we can write next packet)
			while (!(USART2->SR & 0x0080)); // wait until Tx buffer empty
			// We got out of the while loop, the Transmit Data Register is empty
			USART2->DR = txData[(txCntr++)%name_len]; // Write the packet to be sentr
			delayMs(100);
		}

		// receive input
		// Some delay
		for (int i=0; i<10000;i++) {
		// Check if there is data in the Receive Data Register
			if ((USART2->SR)&(1<<5))  { // Check if there is a packet received

				input = (USART2->DR)&0xff;    // Reading the packet will automatically clear the RXNE

				// turn off all LEDs as we have a new input
				turn_off_PCx(red);
				turn_off_PCx(green);
				turn_off_PCx(blue);

				// parse input make color selection
				if (input == 'g' || input == 'G') {
					turn_on_PCx(green);
				} else if (input == 'r' || input =='R') {
					turn_on_PCx(red);
				} else if (input == 'b' || input == 'B') {
					turn_on_PCx(blue);
				} else {
					input = '\0';
				}
			}
		}
	}
	return 0;
}

