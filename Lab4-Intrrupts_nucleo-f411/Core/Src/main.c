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

char txData[16] = "AJ Trantham\0"; // data to send when blue C13 button is pressed
int txCntr = 0;


int main(void) {

	delay_ms(300); // was only working in debug mode??? what?

	// enable clks for GPIOA, GPIOB and GPIOC
	RCC->AHB1ENR |= 7;

	RCC->APB2ENR |= 1<<14; // enable the syscfg

	LED_init();
	B1_config();

	__disable_irq(); // disables the global interrupt request
	set_sysTick_interrupt();

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

//	// enable the push button interrupt on rising edge
	SYSCFG->EXTICR[3] &= ~0x00F0;       /* clear port selection for EXTI13 */
	SYSCFG->EXTICR[3] |= 0x0020;        /* select port C for EXTI13 */

	EXTI->IMR |= 0x2000;                /* unmask EXTI13 */
	EXTI->FTSR |= 0x2000;               /* select falling edge trigger- this was supposed to be falling edge but it wouldn't work on 0*/

	//    NVIC->ISER[1] = 0x00000100;         /* enable IRQ40 (bit 8 of ISER[1]) */
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	__enable_irq();                     /* global enable IRQs */


	while(1) {

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

void EXTI15_10_IRQHandler(void) {
	// set the TX enable interrupt to so that when we have TX data the interrupt will fire
	USART2->CR1 |= 1<<7;	//TXEIE TXE interrupt enable

	// if true we just sent our last char
	if (txData[txCntr] == '\0') {
		EXTI->PR = 0x2000;
		USART2->CR1 |= 0x0020; 			// enables the rx interrupt
		USART2->CR1 &= 0<<7;			// disable the TXEIE
		NVIC_EnableIRQ(USART2_IRQn);	// enable interrupt in NVIC
		UART2_tx_init();
		UART2_rx_init();
		UART2_init();
		set_sysTick_interrupt(); // why do I need to reset this
		// TODO: the led bar it not able to reset after using button
	}
}

// Note I had to comment out the stm32f4xx_it.c
void SysTick_Handler(void) {
	toggle_pin_5();
	set_sysTick_interrupt();
}

// default interrupt handler - should trigger when rx packet is received
void USART2_IRQHandler(void) {

	if (EXTI->PR == 0x2000) {
	//if the TXE bit is 0 then Transmit Data register is NOT empty (so we need to stall the CPU)
	//if the TXE bit is 1 then Transmit Data register is empty (so we can write next packet)
		while (!(USART2->SR & 0x0080)); // wait until Tx buffer empty
		// We got out of the while loop, the Transmit Data Register is empty
		int name_len = sizeof(txData)/sizeof(txData[0]);
		USART2->DR = txData[(txCntr++)%name_len]; // Write the packet to be sentr
		delay_ms(100);	// generally not a good idea to delay in interrupt
		if (txData[txCntr] == '\0') {
			USART2->CR1 &= 0<<7; // clear the TX enable so we don't worry about it until the user presses the button
		}

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




