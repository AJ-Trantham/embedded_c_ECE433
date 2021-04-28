
#include "stm32f4xx.h"
#include "stdio.h"

// signal priority will always be #1
#define CONTROL_SAMPLE_PRIO 3

// util functions
void PCx_OUT_MODER_config(int pc_num);
void LED_init(void);
void LED_toggle(void);
void delay_ms(uint32_t val);
void myprint(char msg[]);
int USART2_read(void);
void USART2_write(int ch);
void USART2_init(void);
// adc functions
int read_ADC_step(void);
float read_read_pot_percent(void);
void ADC_init(void);
// control functions
void update_rate(void);
void update_depth(void);
void control_sample_timer_config(void);



// constants
const float V_REF = 3.3; //volts
const int DAC_RES = 1024; // 10 bits - output to 10 bit DAC MPC4911
const int ADC_RES = 4096; // 12 bits - resolution of ADC reads
const int MINRATE = 333; // 333 ms or 3 Hz
const int MAXRATE = 100; // 100 ms or 10 Hz
const int MAXDEPTH = 0; // signal is attenuated 100%, or voltage is 0, step of 0
const int MINDEPTH = 1024; // when depth is minimized, signal is not attenuated, which means voltage is max, stored as DAC_Step
// TODO: compiler won't let me assign a const here? why??
const int control_sample_time = 100000; // micro seconds, so reads controls 10 times a second

// globals
/**
 * depth represents the level of signal attenuation stored as a digital value,
 * the more depth, the more attenuation, aka more resistance (and less voltage to octocupler)
 * this is stored as a step value between MINDEPTH and MAXDEPTH
 *
 */
int depth = 0;

/**
 * represents the value of the rate pot
 * controls the period of the trem wave
 * value stored in ms
 * should always be between MINRATE and MAXRATE */
int rate = 0;

// -------------------------------------------------ISRs--------------------------------------------------------------------

void TIM2_IRQHandler(void) {
	TIM2->SR = 0;	// clears the interrupt flag UIF

	update_rate();
	update_depth();

	// reset interrupt
	control_sample_timer_config();
}


int main(void) {

	ADC_init();
	LED_init();
	USART2_init();

	control_sample_timer_config();

	 // set up interrupts
	 __disable_irq(); // disables the global interrupt request
	 NVIC_EnableIRQ(TIM2_IRQn);				// enables the tim2 interrupt
	 NVIC_SetPriority(TIM2_IRQn, CONTROL_SAMPLE_PRIO);	// sets the timer priority
	 __enable_irq();

	while(1) {

		char send[10];
		sprintf(send, "$%d;", depth);
		myprint(send);

		// need to paramatarize between acceptable time threasholds
		LED_toggle();
		delay_ms(rate/2);
	}
}


// --------------------------------ADC Functions---------------------------------------------------------------------
/**
 * Initializes ADC to read both the Depth (PA0) and Rate (PA1)
 * Set up ADC to read both values on a read conversation
 * TODO: could update this to read PA0 and then PA1 in one read...
 */
void ADC_init(void) {
	RCC->AHB1ENR |= 1;	            /* enable GPIOA clock */
	GPIOA->MODER |= 0xC;           /* PA1 analog mode Rate */
	GPIOA->MODER |= 0x3;		  /* PA0 to analog mode Depth Control*/

    /* setup ADC1 to read starting at PA0 */
    RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
    ADC1->CR2 = 0;                  /* SW trigger */
    ADC1->SQR3 = 0;                 /* conversion sequence starts at ch 0 */
    ADC1->SQR1 = 0;                 /* conversion sequence length 1 */
    ADC1->CR2 |= 1;                 /* enable ADC1 */
}

/** returns the float percentage of the ADC - based off resolution of the ADC */
float read_pot_percent(void) {
	return (float)read_ADC_step() / (ADC_RES);
}

/** returns the discrete value between 0 and RES */
int read_ADC_step(void) {
	RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
	ADC1->CR2 |= 0x40000000;        /* start a conversion */
	while(!(ADC1->SR & 2)) {}       /* wait for conv complete */

	return (ADC1->DR);
}

/**
 * Using TIM2 sets a triggers an interrupt every control_sample_time micro seconds
 */
void control_sample_timer_config(void) {
	// set up timer to to interrupt when we should sample
	RCC->APB1ENR |= 1;              /* enable TIM2 clock */
	TIM2->PSC = 16-1;               /* divided by 16  (use N-1) - dividing by 16 gets in in u sec*/
	TIM2->ARR = control_sample_time-1;              /* sampling frequency is the number of micro seconds to count to*/
	TIM2->CNT = 0;                  /* clear timer counter */
	TIM2->CR1 = 1;                  /* enable TIM2 */

	TIM2->DIER |= 1;				// enable the Update Interrupt Enable
}

// ------------------------------------------------- Control Functions ---------------------------------------------------
/** Reads Rate Pot PA1 and updates rate global variable */
void update_rate(void) {
	ADC1->SQR3 = 1;                 /* conversion sequence starts at ch 1 which maps to PA1 */
	ADC1->CR2 |= 1;                 /* enable ADC1 */
	float pot_perc = read_pot_percent();
	rate = (pot_perc * (MAXRATE - MINRATE)) + MINRATE;
}

/** Reads Depth Pot PA0 and updates depth global variable */
void update_depth(void) {
	ADC1->SQR3 = 0;                 /* conversion sequence starts at ch 0 which maps to PA0 */
	ADC1->CR2 |= 1;                 /* enable ADC1 */
	float pot_perc = read_pot_percent();
	depth = (pot_perc * (MAXDEPTH - MINDEPTH)) + MINDEPTH;
}

// --------------------------------------------------Utility Functions-----------------------------------------------------

void LED_init(void) {
    // configure PA5 as output to drive the LED
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */
    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode */
    GPIOA->MODER |=  0x00000400;    /* set pin to output mode */
    GPIOA->BSRR   = (1<<21);        /* Turn LED off           */
}


void LED_toggle(void) {
    GPIOA->ODR ^=0x20;              /* Toggle LED            */
}

void delay_ms(uint32_t val) {
    // Using SysTick Timer:
    //        A delay function that can stall CPU 1msec to 100 sec, depending on val.

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

void myprint(char msg[]){
	uint8_t idx=0;
	while(msg[idx]!='\0' ){
		USART2_write(msg[idx++]);
	}
}

/* initialize USART2 to transmit at 115200 Baud */
void USART2_init(void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */

    /* Configure PA2, PA3 for USART2 TX, RX */
    GPIOA->AFR[0] &= ~0xFF00;
    GPIOA->AFR[0] |=  0x7700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x00F0;
    GPIOA->MODER  |=  0x00A0;   /* enable alt. function for PA2, PA3 */

    USART2->BRR = 0x008B;       /* 115200 baud @ 16 MHz */
    USART2->CR1 = 0x000C;       /* enable Tx, Rx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
}

/* Write a character to USART2 */
void USART2_write (int ch) {
    while (!(USART2->SR & 0x0080)) {}   // wait until Tx buffer empty
    USART2->DR = (ch & 0xFF);
}

/* Read a character from USART2 */
int USART2_read(void) {
    while (!(USART2->SR & 0x0020)) {}   // wait until char arrives
    return USART2->DR;
}
