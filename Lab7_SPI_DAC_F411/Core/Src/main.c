// Lab 7: usingn DAC SPI device to create waves
// Remember NSS pin must be tied high alt function is pa4 for this

#include "stm32f411xe.h"
#include "stdio.h"

#define PRIO_BUTTON 2 	// it is important that the button priority is higher (less than) the TIM2 priority
#define PRIO_TIM2 3		// when the sampling_frequency is set too fast, the tim2 interrupt will constantly fire not leaving time for the button
#define PRIO_SYSTICK 4
#define FAST_SAMPLE_SIZE 1000
#define FAST_THREASHOLD 500
#define SS_PIN 11
#define SS_GPIO 'C'
#define LDAC_PIN 2
#define LDAC_GPIO 'D'
#define NUM_WAVES 4
#define NUM_SAMPLES 50

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
void TIM1_init(void);
void set_sysTick_interrupt(int clk_div);
void delay_ms(uint32_t val);
void B1_config(void);
void ADC_init(void);
int PXx_OUT_MODER_config(int PX_num, char GPIO);
void turn_on_PXnum(int px_num, char GPIO);
GPIO_TypeDef * select_GPIO(char GPIO);
void turn_off_PXnum(int px_num,char GPIO);
void SPI3_init(void);
void DAC_write(short data);

// globals for interrupts

const float V_REF = 3.3; //volts
const int RES = 4096; // 12 bits
const int MAX_SAMPLE_RATE = 2000000; //us or 2 sec
const int MIN_SAMPLE_RATE = 1; //us
const int SAMPLES_IN_ONE_SEC = 1000000;
const int CLK_SPEED = 16000000;

//const int NUM_SAMPLES = 10; // this is for lab 6 and the number of samples in a wave
//const int NUM_SAMPLES = 100;
//const int SAMPLE_SPACING = 1600; // this is the clock divider to divide the systick for 10 samples at a period of a ms - triggers interrupt every 1 us
//const int SAMPLE_SPACING = 16; // triggers systick every 1/100 of a ms fro 100 samples
const int SAMPLE_SPACING = 160; // for 50 samples

int sampling_frequency;
int sampling_range;
int adc_samples[FAST_SAMPLE_SIZE];
int fast_sample_index = 0;

int sample_index = 0;
int wave_index = 0;
// create arrays to hold duty cycle values representing a wave for 1 period
int current_wave[NUM_WAVES][NUM_SAMPLES] = {
		{512,576.17,639.33,700.48,758.66,812.95,862.49,906.5,944.3,975.27,999,1015,1023,1023,1014.9,998.94,975.27,944.3,906.5,862.49,812.95,758.66,700.48,639.33,576.17,512,447.83,384.67,323.52,265.34,211.05,161.51,117.5,79.704,48.729,25.059,9.0689,1.0103,1.0103,9.0689,25.059,48.729,79.704,117.5,161.51,211.05,265.34,323.52,384.67,447.83,
		}, //sine wave
		{1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
		}, // square wave
		{0,40.96,81.92,122.88,163.84,204.8,245.76,286.72,327.68,368.64,409.6,450.56,491.52,532.48,573.44,614.4,655.36,696.32,737.28,778.24,819.2,860.16,901.12,942.08,983.04,1023,983.04,942.08,901.12,860.16,819.2,778.24,737.28,696.32,655.36,614.4,573.44,532.48,491.52,450.56,409.6,368.64,327.68,286.72,245.76,204.8,163.84,122.88,81.92,40.96
		}, // triangle wave
		{0,20.48,40.96,61.44,81.92,102.4,122.88,143.36,163.84,184.32,204.8,225.28,245.76,266.24,286.72,307.2,327.68,348.16,368.64,389.12,409.6,430.08,450.56,471.04,491.52,512,532.48,552.96,573.44,593.92,614.4,634.88,655.36,675.84,696.32,716.8,737.28,757.76,778.24,798.72,819.2,839.68,860.16,880.64,901.12,921.6,942.08,962.56,983.04,1003.5
		} // sawtooth
};



// -----------------------------------------------HANDLERS-------------------------------

//  collect samples from PA0, read with ADC and send via UART
void TIM2_IRQHandler(void) {
	TIM2->SR = 0;			// clears the interrupt flag UIF

	// take sample and send it
	char txt[256];
	float voltage = read_ADC_voltage();
	sprintf(txt, "$%.02f;", voltage);
	myprint(txt);

	// reset interrupt
	set_sample_interrupt();
}

/** handler for button PC8
 * 1) Read the Pot and update the sampling rate
 * 2) switch which wave is transmitting to the ADC
 **/
void EXTI15_10_IRQHandler(void) {
	// start saving data when this interrupt will take longer to execute than the sample rate

	wave_index++;
	if (wave_index % NUM_WAVES == 0 ) {
		wave_index = 0;
	}

	EXTI->PR = 0x2000; 		// clear the pending interrupt flag
	set_sample_interrupt();
}

// Note I had to comment out the stm32f4xx_it.c
void SysTick_Handler(void) {

	// update the TIM1 duty cycle for each channel to continue to generate the wave form
	short wave_val = current_wave[wave_index][sample_index++];
	DAC_write(wave_val);

	if (sample_index % NUM_SAMPLES == 0) {
		sample_index = 0;
	}

	set_sysTick_interrupt(SAMPLE_SPACING);
}

// --------------------------------------------END HANDLERS---------------------------------------

int main (void) {

    // initialize variables
	sampling_frequency = 500000; // set default sample frequency

	delay_ms(300); // this fixed something once

    USART2_init();


    B1_config(); // set up push button

    RCC->APB2ENR |= 0x4000; // enable the SysCFG clk, used for sysTick

    RCC->AHB1ENR |= 1;	            /* enable GPIOA clock */
    //GPIOA->MODER |= 0xC;           /* PA1 analog */
    GPIOA->MODER |= 0x3;		  /* PA0 to analog mode*/

    ADC_init();

    // initialize •	Use PC10, PC12, PC11, and PD2 for SCK, MOSI, SS, and LDAC_
    GPIOC->AFR[1] |= 0x6<<(4*2); 		/*set PC10 to SPI3 CLK*/
    GPIOC->MODER &= ~(0x3<<(2*10));		/* clear MODER for PC10*/
    GPIOC->MODER |= (0x2<<((2*10)));	/* set PC10 to AF 10*/

    GPIOC->AFR[1] |= 0x6<<(4*4);			/*set PC12 to MOSI */
    GPIOC->MODER &= ~(0x3<<(2*12));		/* clear MODER for PC12*/
    GPIOC->MODER |= (0x2<<((2*12)));	/* set PC10 to AF 12*/

    PXx_OUT_MODER_config(SS_PIN,SS_GPIO);		/*set PC11 to general output mode for SS */

    RCC->AHB1ENR |= 1<<3; 				//enable GPIOD
    PXx_OUT_MODER_config(LDAC_PIN,LDAC_GPIO);		/*set PD2 to general output mode for LDAC*/

    turn_on_PXnum(LDAC_PIN, LDAC_GPIO);				/* set LDAC HIGH to start*/

    // enable SPI3
    SPI3_init();

    // set up interrupts
    __disable_irq(); // disables the global interrupt request

    set_sample_interrupt(); // sets up TIM2

    set_sysTick_interrupt(SAMPLE_SPACING); // sets  systick interrupt to change the duty cycle every 1/NUM_SAMPLES * Period
    NVIC_SetPriority(SysTick_IRQn,PRIO_SYSTICK);

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

void DAC_write(short data) {

	while(!(SPI3->SR & 2)) {}

	turn_off_PXnum(SS_PIN,SS_GPIO); 	// bring SS low

	// write data and fill config bits 0011 with MSB first
	SPI3->DR = 0x3000 | (data << 2);

	while(SPI3->SR & 0x80) {}					// wait for the transmission to be done. while busy wait
	//bring SS high - we are done writing
	turn_on_PXnum(SS_PIN, SS_GPIO);

	turn_off_PXnum(LDAC_PIN, LDAC_GPIO);			// set LDAC low - this tells the DAC to process the value we sent it
	for(int i=0; i<10; i++) {}						// some delay
	turn_on_PXnum(LDAC_PIN, LDAC_GPIO);			// bring LDAC high again
}

void SPI3_init(void) {
	RCC->APB1ENR |= (1<<15);			/* Enable SPI3 CLK */
	RCC->AHB1ENR |= 1;					/* enable GPIOA clock*/
	GPIOA->AFR[0] |= 0x6<<(4*4);		/*set PA4 to alternate function 6 */
	GPIOA->MODER &= ~(0x3<<(2*4));		/* clear MODER for PA4*/
	GPIOA->MODER |= (0x2<<((2*4)));		/* set PA4 to AF */
	SPI3->CR1 |= (1<<11);				/* sets SPI to send 16 bits */
	SPI3->CR1 |= (1<<2);				/* Master selection */
	SPI3->CR1 |= (1<<3);				/*Set the baud rate to 1*/
	SPI3->CR2 = 0;
	SPI3->CR1 |= 0x40;			/* Enable the SPI*/
}


void ADC_init(void) {
    /* setup ADC1 to read PA0 */
    RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
    ADC1->CR2 = 0;                  /* SW trigger */
    ADC1->SQR3 = 0;                 /* conversion sequence starts at ch 0 */
    ADC1->SQR1 = 0;                 /* conversion sequence length 1 */
    ADC1->CR2 |= 1;                 /* enable ADC1 */
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

void TIM1_init(void) {
	/* setup TIM1 */
	    RCC->APB2ENR |= 1;              /* enable TIM1 clock */
	    TIM1->PSC = 16;             /* divided by 0xffff -- the slowest it can be */
	    TIM1->ARR = 100;                /* divided by 100 -- so the duty cycle is based on 100% */
	    TIM1->CNT = 0;					// set count to 0
	    TIM1->CCMR1 = 0x6060;           /* PWM mode channel 1 and 2 */
	    TIM1->CCMR2 = 0x6060;			/*PWM for channel 3 and 4*/
	    TIM1->CCER |= 0x1111; 			/*enables all four channels */
	    TIM1->BDTR |=0x8000;            /* Enable timer */
	    TIM1->CCR1 = 50;                /* pulse width 50/100 of the period, 50% Duty Cycle */
	    TIM1->CCR2 = 50;				/* pulse width 50/100 of the period, 50% Duty Cycle */
	    TIM1->CCR3 = 50;				/* pulse width 50/100 of the period, 50% Duty Cycle */
	    TIM1->CCR4 = 50;				/* pulse width 50/100 of the period, 50% Duty Cycle */
	    TIM1->CR1 |= 0x021;             /* enable timer, Center-aligned */
}


/* initialize USART2 to transmit at 115200 Baud */
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

/**
 * Sets up the SysTick interrupt to fire
 * clk_div divdes the clock so clk_div = 16000000 gets a
 * 1 sec interrupt
 */
void set_sysTick_interrupt(int clk_div) {
	// config SysTick to be in interrupt mode
	SysTick->LOAD = clk_div-1; // set reload the number to 16 meg. This will run out once every second
	SysTick->VAL = 0;
	SysTick->CTRL = 0x7;			//enables the SysTick interrupt
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

void B1_config(void) {
	RCC->AHB1ENR |= 1<<2; // enable GPIOC for push button
	GPIOC->MODER &= ~(0x3<<(2*13)); // clears 2 in/out mode for PC13
	// This should be unnecessary: GPIOC->MODER |= (0<<(2*13)); // this sets bits 2 mode bits for PC13 to input 00
}




int PXx_OUT_MODER_config(int PX_num, char GPIO) {
	if (GPIO == 'B' || GPIO == 'b') {
		GPIOB->MODER &= ~(0x3<<(2*PX_num)); // clears 2 in/out mode for PBpc_num
		GPIOB->MODER |= (1<<(2*PX_num)); // this sets bits 2 mode bits for PBc_num General purpose output mode 01
	} else if (GPIO == 'C' || GPIO == 'c') {
		GPIOC->MODER &= ~(0x3<<(2*PX_num)); // clears 2 in/out mode for PCpc_num
		GPIOC->MODER |= (1<<(2*PX_num)); // this sets bits 2 mode bits for PCpc_num General purpose output mode 01
	} else if (GPIO == 'D' || GPIO == 'd') {
		GPIOD->MODER &= ~(0x3<<(2*PX_num)); // clears 2 in/out mode for PCpc_num
		GPIOD->MODER |= (1<<(2*PX_num)); // this sets bits 2 mode bits for PCpc_num General purpose output mode 01
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

GPIO_TypeDef * select_GPIO(char GPIO) {

	if(GPIO == 'B') {
		return GPIOB;
	} else if (GPIO =='C') {
		return GPIOC;
	} else if (GPIO == 'D') {
		return GPIOD;
	}
}
