// Analog Monitoring Program

#include "stm32f4xx.h"
#include "stdio.h"
#include "common_functions.h"

#define PRIO_BUTTON 2 	// it is important that the button priority is higher (less than) the TIM2 priority
#define PRIO_TIM2 3		// when the sampling_frequency is set too fast, the tim2 interrupt will constantly fire not leaving time for the button
#define FAST_SAMPLE_SIZE 1000
#define FAST_THREASHOLD 500

void delayMs(int);
void USART2_init(void);
void USART2_write(int c);
int USART2_read(void);
float get_temp_value(float thermistor_reading);
int get_sample_rate_from_ADC(int adc_val);
void myprint(char msg[]);
void set_sample_interrupt(void);
float read_ADC_voltage(void);
int read_ADC_step(void);

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
int adc_samples[FAST_SAMPLE_SIZE];
int fast_sample_index = 0;

// -----------------------------------------------HANDLERS-------------------------------


void TIM2_IRQHandler(void) {
	TIM2->SR = 0;			// clears the interrupt flag UIF

	// when sampling faster than 50 u sec we want to just take samples as fast as possible
	if (sampling_frequency < FAST_THREASHOLD && fast_sample_index < 1000) {
		// read adc value and store in adc_samples
		adc_samples[fast_sample_index++] = read_ADC_step();

	} else if (sampling_frequency < FAST_THREASHOLD) {
		// when we are in fast mode but are on the 1000th sample
		fast_sample_index = 0;
		char txt[256];
		for (int i=0; i < FAST_SAMPLE_SIZE; i++ ) {
			float temp = get_temp_value(adc_samples[i]*(V_REF/RES));
			sprintf(txt, "$%.02f;", temp);
			myprint(txt);
		}

	} else {
		// take sample and send it
		char txt[256];
		float voltage = read_ADC_voltage();
		float temp = get_temp_value(voltage);
		sprintf(txt, "$%.02f;", temp);
		myprint(txt);
	}

	// reset interrupt
	set_sample_interrupt();
}

/** handler for button PC8 **/
void EXTI15_10_IRQHandler(void) {
	// start saving data when this interrupt will take longer to execute than the sample rate

	// change ADC to sample the pot on ch1
	ADC1->SQR3 = 1;                 /* conversion sequence starts at ch 1 which maps to A1 */
	ADC1->CR2 |= 1;                 /* enable ADC1 */

	// update sample_frequency
	int adc_val = read_ADC_step();
	sampling_frequency = get_sample_rate_from_ADC(adc_val);

	// set ADC back to channel 0 to read thermistor
	ADC1->SQR3 = 0;                 /* conversion sequence starts at ch 1 which maps to A1 */
	ADC1->CR2 |= 1;                 /* enable ADC1 */

	EXTI->PR = 0x2000; 		// clear the pending interrupt flag
	set_sample_interrupt();
}

// --------------------------------------------END HANDLERS---------------------------------------

int main (void) {

    // initialize variables
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

    set_sample_interrupt();

    NVIC_EnableIRQ(TIM2_IRQn);				// enables the tim2 interrupt
    NVIC_SetPriority(TIM2_IRQn, PRIO_TIM2);	// sets the timer priority

    // set the push button interrupt
    SYSCFG->EXTICR[3] &= ~0x00F0;       /* clear port selection for EXTI13 */
    SYSCFG->EXTICR[3] |= 0x0020;        /* select port C for EXTI13 */

    EXTI->IMR |= 0x2000;                /* unmask EXTI13 */
    EXTI->FTSR |= 0x2000;               /* select falling edge trigger- this was supposed to be falling edge but it wouldn't work on 0*/

    NVIC_EnableIRQ(EXTI15_10_IRQn);						// enables the Button interrupt
    NVIC_SetPriority(EXTI15_10_IRQn, PRIO_BUTTON);		// sets the button priority

    __enable_irq();

    while (1) {

    }
}

// returns the float percentage of the ADC
float read_ADC_voltage(void) {
	return read_ADC_step()*(V_REF/RES);
}

// returns the discrete value between 0 and RES
int read_ADC_step() {
	RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
	ADC1->CR2 |= 0x40000000;        /* start a conversion */
	while(!(ADC1->SR & 2)) {}       /* wait for conv complete */

	return (ADC1->DR);
}


void set_sample_interrupt(void) {
	// set up timer to to interrupt when we should sample
	RCC->APB1ENR |= 1;              /* enable TIM2 clock */
	TIM2->PSC = 16-1;               /* divided by 16  (use N-1) - dividing by 16 gets in in u sec*/
	TIM2->ARR = sampling_frequency-1;              /* sampling frequency is the number of micro seconds to count to*/
	TIM2->CNT = 0;                  /* clear timer counter */
	TIM2->CR1 = 1;                  /* enable TIM2 */

	TIM2->DIER |= 1;				// enable the Update Interrupt Enable
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
 * get the sample rate from the ADC discrete value between 0 and RES
 */
int get_sample_rate_from_ADC(int adc_val) {

	// adjust for 0 voltage - pot all the way off
	if (adc_val == 0) {
		return 2; // smallest value possible as we subtract 1 when feeding to ARR
	}
	int new_rate = ((float)adc_val/RES) * sampling_range;
	return new_rate;
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
