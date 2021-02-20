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

	// enable clks for GPIOA, GPIOB and GPIOC
	RCC->AHB1ENR |= 7;

	RCC->APB2ENR |= 1<<14; // enable the syscfg
	//RCC->APB2ENR |= 0x4000;             /* enable SYSCFG clock */

	// Req 1: •	The on-board LED blinks every second based on SysTick timer running in interrupt mode.
	LED_init();
	B1_config();

	__disable_irq(); // disables the global interrupt request
	set_sysTick_interrupt();
	// END Req 1

	/* Req 2: The USART2 triggers an interrupt whenever a character is received from the PC. The acceptable
	 * characters are ‘0’ to ‘9’ where each lights a single LED of the ‘0’ to ‘9’ in the 10-segment bargraph.
	 * Any other character should turn off all LEDs.
	 */
	            /* enable GPIOA clock */

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
////	SYSCFG->EXTICR[3] &= ~0x00F0;		// clears the port selection for EXTICR
////	SYSCFG->EXTICR[3] |= 0x0020;		// select port C as the calling peripheral - I think these are incorrect
////
////	// these EXTI registers are backwards, shifting 9 sets from 22-9 = 13 - diverging from book sample
////	EXTI->IMR |= 1<<13;						// EXTI->IMR |= 0x2000; This enables the External Interrupt 13
////	EXTI->FTSR &= 0<<13;						// select rising edge triggered
////
////	//NVIC->ISER[1] = 0x00000100;
////	NVIC_EnableIRQ(EXTI15_10_IRQn);			//enable the external Interrupt handler
//
//
//	// books
//    SYSCFG->EXTICR[3] &= ~0x00F0;       /* clear port selection for EXTI13 */
//    SYSCFG->EXTICR[3] |= 0x0020;        /* select port C for EXTI13 */
//
//    EXTI->IMR |= 0x2000;                /* unmask EXTI13 */
//    EXTI->FTSR |= 0x2000;               /* select falling edge trigger */
//
////    NVIC->ISER[1] = 0x00000100;         /* enable IRQ40 (bit 8 of ISER[1]) */
//    NVIC_EnableIRQ(EXTI15_10_IRQn);
//
//	__enable_irq();

	//__disable_irq();                    /* global disable IRQs */

//	    RCC->AHB1ENR |= 4;	                /* enable GPIOC clock */
//	    RCC->AHB1ENR |= 1;                  /* enable GPIOA clock */
//	    RCC->APB2ENR |= 0x4000;             /* enable SYSCFG clock */

	    /* configure PA5 for LED */
//	    GPIOA->MODER &= ~0x00000C00;        /* clear pin mode */
//	    GPIOA->MODER |=  0x00000400;        /* set pin to output mode */

	    /* configure PC13 for push button interrupt */
	    //GPIOC->MODER &= ~0x0C000000;        /* clear pin mode to input mode */

	    SYSCFG->EXTICR[3] &= ~0x00F0;       /* clear port selection for EXTI13 */
	    SYSCFG->EXTICR[3] |= 0x0020;        /* select port C for EXTI13 */

	    EXTI->IMR |= 0x2000;                /* unmask EXTI13 */
	    EXTI->FTSR |= 0x2000;               /* select falling edge trigger */

	//    NVIC->ISER[1] = 0x00000100;         /* enable IRQ40 (bit 8 of ISER[1]) */
	    NVIC_EnableIRQ(EXTI15_10_IRQn);

	    __enable_irq();                     /* global enable IRQs */




	while(1) {

		// write input when button is pressed
//		if (!((GPIOC->IDR)&(1<<13))) {
//		//if the TXE bit is 0 then Transmit Data register is NOT empty (so we need to stall the CPU)
//			//if the TXE bit is 1 then Transmit Data register is empty (so we can write next packet)
//			while (!(USART2->SR & 0x0080)); // wait until Tx buffer empty
//			// We got out of the while loop, the Transmit Data Register is empty
//			int name_len = sizeof(txData)/sizeof(txData[0]);
//			USART2->DR = txData[(txCntr++)%name_len]; // Write the packet to be sentr
//			delay_ms(100);
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

//void EXTI15_10__IRQHandler(void) {
//	// starts the processs
//
//	toggle_pin_5();
//
////	if (!((GPIOC->IDR)&(1<<13))) {
////			//if the TXE bit is 0 then Transmit Data register is NOT empty (so we need to stall the CPU)
////				//if the TXE bit is 1 then Transmit Data register is empty (so we can write next packet)
////			while (!(USART2->SR & 0x0080)); // wait until Tx buffer empty
////			// We got out of the while loop, the Transmit Data Register is empty
////			int name_len = sizeof(txData)/sizeof(txData[0]);
////			USART2->DR = txData[(txCntr++)%name_len]; // Write the packet to be sentr
////			delay_ms(100);
////		}
//
//	EXTI->PR = 1<<13; // clears the interrupt pending flag for PR13
//}

void EXTI15_10_IRQHandler(void) {

//		toggle_pin_5();
	// write input when button is pressed




   EXTI->PR = 0x2000;          /* clear interrupt pending flag */
}

// Note I had to comment out the stm32f4xx_it.c
void SysTick_Handler(void) {
	toggle_pin_5();
	set_sysTick_interrupt();
}

// default interrupt handler - should trigger when rx packet is received
void USART2_IRQHandler(void) {

	if (!((GPIOC->IDR)&(1<<13))) {
				//if the TXE bit is 0 then Transmit Data register is NOT empty (so we need to stall the CPU)
	//if the TXE bit is 1 then Transmit Data register is empty (so we can write next packet)
		while (!(USART2->SR & 0x0080)); // wait until Tx buffer empty
		// We got out of the while loop, the Transmit Data Register is empty
		int name_len = sizeof(txData)/sizeof(txData[0]);
		USART2->DR = txData[(txCntr++)%name_len]; // Write the packet to be sentr
		delay_ms(100);	// generally not a good idea to delay in interrupt
	}

	if ((USART2->SR)&(1<<5))  { // Check if there is a packet received
		char input = (USART2->DR)&0xff;    // Reading the packet will automatically clear the RXNE
		handle_input(input);
	}

	// if my buffer is empty i need to stop it here


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




