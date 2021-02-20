/**
 * This file
 */
#include "common_functions.h"


GPIO_TypeDef * select_GPIO(char GPIO);

void toggle_pin_5(void) {
	GPIOA->ODR ^= (0x1<<5);
}

void LED_init(){
    // configure PA5 as output to drive the LED
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */
    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode */
    GPIOA->MODER |=  0x00000400;    /* set pin to output mode */
    GPIOA->BSRR   = (1<<21);        /* Turn LED off           */
}

void B1_config(void) {
	GPIOC->MODER &= ~(0x3<<(2*13)); // clears 2 in/out mode for PC13
	// This should be unnecessary: GPIOC->MODER |= (0<<(2*13)); // this sets bits 2 mode bits for PC13 to input 00
}



void delay_ms(uint32_t val) {
    // Using SysTick Timer:
    //        A delay function that can stall CPU 1msec to 100 sec, depending on val.
    //
    // useful link: https://www.youtube.com/watch?v=aLCUDv_fgoU
    //
    // The concept here is to make a delay block using SysTick timer for a delay of 1 msec.
    // The 1 msec delay will be inside a for loop that will loop for val times which will
    // result in a delay of as short as 1msec (for val=1) and as long as 1msec*0xffff_ffff (4,294,967.295 sec)

    // Here are the steps to set the SysTick to delay 1 msec
    //   1- Set the load register to achieve 1msec. Note that you have two options to source your
    //      timer clock. One is to use the HSI clock of 16MHz while the other to use 16MHz/8.
    //   2- Set the counter current value to 0 so that the counter start
    //   3- Enable the counter and the bit to select which clock you want to use
    // Now the counter is counting down once it reaches 0, a flag in the control register
    // will be set -- use that flag.

	uint32_t sysClk = 16000000; //Hz
	float time = 0.001; //sec
    SysTick->LOAD =  (sysClk * time) - 1;    /* reload with number of clocks per millisecond (use N-1)*/
    SysTick->VAL = 0;          /* clear current value register */
    SysTick->CTRL = 0x05;      /* Enable the timer */

    for (uint32_t i=0; i<val; i++){
            while((SysTick->CTRL & 0x10000) == 0); /* wait until the COUNTFLAG is set */
        }

    SysTick->CTRL = 0; 	/* Stop the timer (Enable = 0) */

}

int PXx_OUT_MODER_config(int PX_num, char GPIO) {
	if (GPIO == 'B' || GPIO == 'B') {
		GPIOB->MODER &= ~(0x3<<(2*PX_num)); // clears 2 in/out mode for PBpc_num
		GPIOB->MODER |= (1<<(2*PX_num)); // this sets bits 2 mode bits for PBc_num General purpose output mode 01
	} else if (GPIO == 'C' || GPIO == 'c') {
		GPIOC->MODER &= ~(0x3<<(2*PX_num)); // clears 2 in/out mode for PCpc_num
		GPIOC->MODER |= (1<<(2*PX_num)); // this sets bits 2 mode bits for PCpc_num General purpose output mode 01
	} else {
		// unsupported GIPO choice
		return -1;
	}
	return 0;
}

/**
 * Turns of PC(pc_num)
 */
void turn_off_PXnum(int px_num,char GPIO) {
	GPIO_TypeDef * selected_GPIO = select_GPIO(GPIO);
	selected_GPIO->ODR = (0x0<<px_num);
}

/**
 * // turns PC(pc_num)
 */
void turn_on_PXnum(int px_num, char GPIO) {
	GPIO_TypeDef * selected_GPIO = select_GPIO(GPIO);
	selected_GPIO->ODR |= (0x1<<px_num);
}

// could have a function that just sets a pointer to the proper GPIOX
GPIO_TypeDef * select_GPIO(char GPIO) {

	if(GPIO == 'B') {
		return GPIOB;
	} else if (GPIO =='C') {
		return GPIOC;
	}
}



/**
 * Configure the USART2
 * Sets the baud rate to 16,000,000
 */
void UART2_init(void) {
	RCC->APB1ENR |= (1<<17); // enable the USART 2 clock - this is labeled as the UART2RST
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
void UART2_rx_init(void) {
	// set PA3 to UART2 RX
	GPIOA->MODER &= ~(0X3<<(2*3)); // clear PA3's 2 bits
	GPIOA->MODER |= (0x02<<(2*3)); // set PA3's 2 bits to AF

	//set the AF to be AF 7
	GPIOA->AFR[0] &= ~(0xF<<(4*3)); // clear PA3's 4 bits in AFR
	GPIOA->AFR[0] |= (0x7<<(4*3)); // set the 4 bits to AF 7
}

/**
 * enable and initialize the tx UART2
 */
void UART2_tx_init(void) {
	// Connect PA2 to UART2 TX
	//PA2 must be set to AF 7
	GPIOA->MODER &= ~(0x3<<(2*2)); // clear bits 4 and 5
	GPIOA->MODER |= (0x2<<(2*2)); // set the mode to AF ( 0x2 = 0b10 10 is the code for AF) Why is in position 4

	GPIOA->AFR[0] &= ~(0xF00); // clear the bits where we want alt 7
	GPIOA->AFR[0] = (0x700); // this sets alt 7
}
