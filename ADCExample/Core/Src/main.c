// Analog Monitoring Programe

#include "stm32f4xx.h"
#include "stdio.h"
#include "common_functions.h"

#define PRIO_BUTTON 1
#define PRIO_SYSTICK 2

void delayMs(int);
void USART2_init(void);
void USART2_write(int c);
int USART2_read(void);
float get_temp_value(float thermistor_reading);
int get_sample_rate_from_voltage(float voltage_perc);
void myprint(char msg[]);
void reset_SysTick_interrupt(void);
float read_ADC(void);

// globals for interrupts
const float TEMP_MAX_F = 257.00; // 125 C
const float TEMP_MIN_F = -40.00; // -40 C
const float V_REF = 3.3; //volts
const int RES = 4096; // 12 bits
const int MAX_SAMPLE_RATE = 2000000; //us or 2 sec
const int MIN_SAMPLE_RATE = 1; //us
const int SAMPLES_IN_ONE_SEC = 1000000;
const int CLK_SPEED = 16000000;

int sampling_frequency;
int sampling_range;

// -----------------------------------------------HANDLERS-------------------------------
// Note I had to comment out the stm32f4xx_it.c
void SysTick_Handler(void) {

	// take sample
	char txt[256];
	float voltage = read_ADC();
	float temp = get_temp_value(voltage);
	sprintf(txt, "$%.02f;", temp);
	myprint(txt);

	delayMs(10);

	// reset interrupt
	reset_SysTick_interrupt();
}

/** handler for button PC8 **/
void EXTI15_10_IRQHandler(void) {
	// start saving data when this interrupt will take longer to execute than the sample rate

	// change ADC to sample the pot on ch1
	ADC1->SQR3 = 1;                 /* conversion sequence starts at ch 1 which maps to A1 */
	ADC1->CR2 |= 1;                 /* enable ADC1 */

	// update sample_frequency
	float voltage = read_ADC();
	sampling_frequency = get_sample_rate_from_voltage(voltage);
	reset_SysTick_interrupt();

	// set ADC back to channel 0 to read thermistor
	ADC1->SQR3 = 0;                 /* conversion sequence starts at ch 1 which maps to A1 */
	ADC1->CR2 |= 1;                 /* enable ADC1 */

	EXTI->PR = 0x2000; 		// clear the pending interrupt flag
}

// --------------------------------------------END HANDLERS---------------------------------------

int main (void) {

    //uint16_t result;
	sampling_frequency = (MAX_SAMPLE_RATE + MIN_SAMPLE_RATE) / 2; // set default sample frequency
	sampling_range = MAX_SAMPLE_RATE - MIN_SAMPLE_RATE;

	delay_ms(300); // was only working in debug mode??? what?

	// enable clks for GPIOA, GPIOB and GPIOC
	RCC->AHB1ENR |= 7;

	RCC->APB2ENR |= 1<<14; // enable the syscfg


    USART2_init();
    RCC->AHB1ENR |= 1<<2; // enable GPIOC for push button
    B1_config(); // set up push button

    RCC->APB2ENR |= 0x4000; // enable the SysCFG clk


    RCC->AHB1ENR |= 1;	            /* enable GPIOA clock */
    GPIOA->MODER |= 0xC;           /* PA1 analog */
    GPIOA->MODER |= 0x3;		  /* PA0 to analog mode*/

    /* setup ADC1 to read PA0 */
    RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
    ADC1->CR2 = 0;                  /* SW trigger */
    ADC1->SQR3 = 0;                 /* conversion sequence starts at ch 0 */
    ADC1->SQR1 = 0;                 /* conversion sequence length 1 */
    ADC1->CR2 |= 1;                 /* enable ADC1 */


    // set up interrupts
    __disable_irq(); // disables the global interrupt request

    reset_SysTick_interrupt();

    // set the push button interrupt
    SYSCFG->EXTICR[3] &= ~0x00F0;       /* clear port selection for EXTI13 */
    SYSCFG->EXTICR[3] |= 0x0020;        /* select port C for EXTI13 */

    EXTI->IMR |= 0x2000;                /* unmask EXTI13 */
    EXTI->FTSR |= 0x2000;               /* select falling edge trigger- this was supposed to be falling edge but it wouldn't work on 0*/

    NVIC_EnableIRQ(EXTI15_10_IRQn);

    NVIC_SetPriority(EXTI15_10_IRQn, PRIO_BUTTON);

    __enable_irq();

    while (1) {

    }
}

// returns the float percentage of the ADC
float read_ADC(void) {
	RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
	ADC1->CR2 |= 0x40000000;        /* start a conversion */
	while(!(ADC1->SR & 2)) {}       /* wait for conv complete */

	return (ADC1->DR)*(V_REF/RES);
}


void reset_SysTick_interrupt(void) {
	int clk_div = ((float) sampling_frequency / SAMPLES_IN_ONE_SEC) * CLK_SPEED;
	set_sysTick_interrupt(clk_div);
}

/**
 * thermistor_reading is the voltage percentage as calculated by ADC_VAL*V_ref/Res
 */
float get_temp_value(float thermistor_reading) {
	float temp_celsius = (thermistor_reading * 100) - 50;
	float temp_far = (temp_celsius * (1.8)) + 32;
	return temp_far;
}

/**
 * voltage_perc is the voltage percentage as calculated by ADC_VAL*V_ref/Res
 */
int get_sample_rate_from_voltage(float voltage) {
	return ((voltage/V_REF) * sampling_range);
}

void myprint(char msg[]){
	uint8_t idx=0;
	while(msg[idx]!='\0' ){
		USART2_write(msg[idx++]);
	}
}
/* initialize USART2 to transmit at 9600 Baud */
void USART2_init (void) {
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


void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}
