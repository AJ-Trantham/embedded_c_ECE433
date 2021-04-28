
#include "stm32f4xx.h"
#include "stdio.h"

// Goal with priorities is to read controls periodically but not lose signal quality
#define PRIO_BUTTON 2 	// want to switch imediately to new wave, its ok if this disrupts are signal as it already is by the nature of the request
#define CONTROL_SAMPLE_PRIO 3 // controls have higher priority since they happen less often temporally
#define PRIO_SYSTICK 4
// wave macros
#define NUM_WAVEPOINTS_PER_CYCLE 50
#define NUM_WAVES 4
// pin macros
#define SS_PIN 11
#define SS_GPIO 'C'
#define LDAC_PIN 2
#define LDAC_GPIO 'D'
#define NUM_WAVES 4

// util functions
void PCx_OUT_MODER_config(int pc_num);
void LED_init(void);
void LED_toggle(void);
void delay_ms(uint32_t val);
void myprint(char msg[]);
int USART2_read(void);
void USART2_write(int ch);
void USART2_init(void);
void B1_config(void);
int PXx_OUT_MODER_config(int PX_num, char GPIO);
void turn_on_PXnum(int px_num, char GPIO);
GPIO_TypeDef * select_GPIO(char GPIO);
void turn_off_PXnum(int px_num,char GPIO);
// adc functions
int read_ADC_step(void);
float read_read_pot_percent(void);
void ADC_init(void);
// timing
void control_sample_timer_config(void);
void set_sysTick_interrupt(int clk_div);
void SysTick_Handler(void);
// voltage output
void config_SPI_pins(void);
void SPI3_init(void);
void DAC_write(short data);
// control functions
void update_rate(void);
void update_depth(void);

// constants
const float V_REF = 3.3; //volts
const int DAC_RES = 1024; // 10 bits - output to 10 bit DAC MPC4911
const int ADC_RES = 4096; // 12 bits - resolution of ADC reads
const int MINRATE = 333; // 333 ms or 3 Hz
const int MAXRATE = 100; // 100 ms or 10 Hz
const int MAXDEPTH = 0; // signal is attenuated 100%, or voltage is 0, step of 0
const int MINDEPTH = 1024; // when depth is minimized, signal is not attenuated, which means voltage is max, stored as DAC_Step
// TODO: compiler won't let me assign a const here? why??
const int control_sample_time = 100000; // micro seconds, so reads controls 10 times a second at 100000 or every 100 ms
const int num_wavepoints_per_cycle = NUM_WAVEPOINTS_PER_CYCLE; // wave sample


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

/** The time in ms between each wave sample data. Detirmined by rate/num_wavepoints_per_cycle */
int wavepoint_time_space;

/** index for the way type */
int wave_index = 0;

/** index for the wavepoint sample */
int sample_index = 0;

float current_wave[NUM_WAVES][NUM_WAVEPOINTS_PER_CYCLE] = {
		{0.5,0.56267,0.62434,0.68406,0.74088,0.79389,0.84227,0.88526,0.92216,0.95241,0.97553,0.99114,0.99901,0.99901,0.99114,0.97553,0.95241,0.92216,0.88526,0.84227,0.79389,0.74088,0.68406,0.62434,0.56267,0.5,0.43733,0.37566,0.31594,0.25912,0.20611,0.15773,0.11474,0.077836,0.047586,0.024472,0.0088564,0.00098664,0.00098664,0.0088564,0.024472,0.047586,0.077836,0.11474,0.15773,0.20611,0.25912,0.31594,0.37566,0.43733
		}, //sine wave
		{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // square wave
		{0,0.04,0.08,0.12,0.16,0.2,0.24,0.28,0.32,0.36,0.4,0.44,0.48,0.52,0.56,0.6,0.64,0.68,0.72,0.76,0.8,0.84,0.88,0.92,0.96,1,0.96,0.92,0.88,0.84,0.8,0.76,0.72,0.68,0.64,0.6,0.56,0.52,0.48,0.44,0.4,0.36,0.32,0.28,0.24,0.2,0.16,0.12,0.08,0.04
		}, // triangle wave
		{0,0.020408,0.040816,0.061224,0.081633,0.10204,0.12245,0.14286,0.16327,0.18367,0.20408,0.22449,0.2449,0.26531,0.28571,0.30612,0.32653,0.34694,0.36735,0.38776,0.40816,0.42857,0.44898,0.46939,0.4898,0.5102,0.53061,0.55102,0.57143,0.59184,0.61224,0.63265,0.65306,0.67347,0.69388,0.71429,0.73469,0.7551,0.77551,0.79592,0.81633,0.83673,0.85714,0.87755,0.89796,0.91837,0.93878,0.95918,0.97959,1
		} // sawtooth
};

// -------------------------------------------------ISRs--------------------------------------------------------------------

void TIM2_IRQHandler(void) {
	TIM2->SR = 0;	// clears the interrupt flag UIF

	update_rate();
	update_depth();

	// reset interrupt
	control_sample_timer_config();
}

// Note I had to comment out the stm32f4xx_it.c
void SysTick_Handler(void) {

	// update the TIM1 duty cycle for each channel to continue to generate the wave form
	float wave_val = current_wave[wave_index][sample_index++];

	// apply depth control to base wave value - TODO: could this be a critical section if a call to update depth occurs here
	int digital_attenuation_range = MINDEPTH - depth;
	short scaled_wave_point = (digital_attenuation_range * wave_val) + depth;
	DAC_write(scaled_wave_point);

	char send[16];
	sprintf(send, "$%d;", scaled_wave_point);
	myprint(send);

	if (sample_index % NUM_WAVEPOINTS_PER_CYCLE == 0) {
		sample_index = 0;
	}

	set_sysTick_interrupt(wavepoint_time_space);
}


int main(void) {

	ADC_init();
	LED_init();
	USART2_init();
	SPI3_init();

	update_rate();
	update_depth();
	control_sample_timer_config(); // sets up TIM2 to read ADCs periodically

	 // set up interrupts
	 __disable_irq(); // disables the global interrupt request
	 NVIC_EnableIRQ(TIM2_IRQn);				// enables the tim2 interrupt
	 NVIC_SetPriority(TIM2_IRQn, CONTROL_SAMPLE_PRIO);	// sets the timer priority

	 set_sysTick_interrupt(wavepoint_time_space);
	 NVIC_SetPriority(SysTick_IRQn,PRIO_SYSTICK);

	 // set the push button interrupt
	 SYSCFG->EXTICR[3] &= ~0x00F0;       /* clear port selection for EXTI13 */
	 SYSCFG->EXTICR[3] |= 0x0020;        /* select port C for EXTI13 */

	 EXTI->IMR |= 0x2000;                /* unmask EXTI13 */
	 EXTI->FTSR |= 0x2000;               /* select falling edge trigger- this was supposed to be falling edge but it wouldn't work on 0*/

	 NVIC_EnableIRQ(EXTI15_10_IRQn);						// enables the Button interrupt
	 NVIC_SetPriority(EXTI15_10_IRQn, PRIO_BUTTON);		// sets the button priority
	 __enable_irq();

	while(1) {

//		char send[10];
//		sprintf(send, "$%d;", depth);
//		myprint(send);

		// need to paramatarize between acceptable time threasholds
		//LED_toggle();
		//delay_ms(rate/2);
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

// --------------------------------------------Timing --------------------------------------------------------------------------

/**
 * Using TIM2 sets a triggers an interrupt every control_sample_time micro seconds
 * Used to control the read time of the ADC, every time this timer fires we read the ADC
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

/**
 * Sets up the SysTick interrupt to fire
 * clk_div divdes the clock so clk_div = 16000000 gets a
 * 1 sec interrupt
 * here clk_div param is assumed to be milli seconds
 */
void set_sysTick_interrupt(int clk_div) {
	RCC->APB2ENR |= 0x4000; // enable the SysCFG clk, used for sysTick
	// config SysTick to be in interrupt mode
	uint32_t sysClk = 16000000; //Hz
	float time = 0.001; //sec
	SysTick->LOAD = (int)(time * sysClk * clk_div)-1; // set reload to 1 ms times clk_div ms to get clk_div ms
	SysTick->VAL = 0;
	SysTick->CTRL = 0x7;			//enables the SysTick interrupt
}

// -------------------------------------------------- Voltage output -----------------------------------------------------
/** sets up pins to use for SPI lines to communicate with the MCP4911 DAC
 * on this DAC, when LDAC pin goes LOW DAC does conversion and writes to VOUT
 */
void config_SPI_pins(void) {

	RCC->AHB1ENR |= 1<<3; 				//enable GPIOD
	RCC->AHB1ENR |= 1<<2;				// enable GPIOC
	// initialize •	Use PC10, PC12, PC11, and PD2 for SCK, MOSI, SS, and LDAC_
	GPIOC->AFR[1] |= 0x6<<(4*2); 		/*set PC10 to SPI3 CLK*/
	GPIOC->MODER &= ~(0x3<<(2*10));		/* clear MODER for PC10*/
	GPIOC->MODER |= (0x2<<((2*10)));	/* set PC10 to AF 10*/

	GPIOC->AFR[1] |= 0x6<<(4*4);		/*set PC12 to MOSI */
	GPIOC->MODER &= ~(0x3<<(2*12));		/* clear MODER for PC12*/
	GPIOC->MODER |= (0x2<<((2*12)));	/* set PC10 to AF 12*/

	PXx_OUT_MODER_config(SS_PIN,SS_GPIO);		/*set PC11 to general output mode for SS */

	PXx_OUT_MODER_config(LDAC_PIN,LDAC_GPIO);		/*set PD2 to general output mode for LDAC*/

	turn_on_PXnum(LDAC_PIN, LDAC_GPIO);			    /* set LDAC HIGH to start - latch closed aka don't write to VOUT*/
}

/**Configures and enables the SPI3 module */
void SPI3_init(void) {
	config_SPI_pins();
	RCC->APB1ENR |= (1<<15);			/* Enable SPI3 CLK */
	RCC->AHB1ENR |= 1;					/* enable GPIOA clock*/
	GPIOA->AFR[0] |= 0x6<<(4*4);		/*set PA4 to alternate function 6 */
	GPIOA->MODER &= ~(0x3<<(2*4));		/* clear MODER for PA4*/
	GPIOA->MODER |= (0x2<<((2*4)));		/* set PA4 to AF */
	//TODO: why is PA4 important - NSS pin needs to be tied high
	SPI3->CR1 |= (1<<11);				/* sets SPI to send 16 bits */
	SPI3->CR1 |= (1<<2);				/* Master selection */
	SPI3->CR1 |= (1<<3);				/*Set the baud rate to 1*/
	SPI3->CR2 = 0;
	SPI3->CR1 |= 0x40;			/* Enable the SPI*/
}

/** Write digital data value to the DAC */
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

// ------------------------------------------------- Control Functions ---------------------------------------------------
/** Reads Rate Pot PA1 and updates rate global variable */
void update_rate(void) {
	ADC1->SQR3 = 1;                 /* conversion sequence starts at ch 1 which maps to PA1 */
	ADC1->CR2 |= 1;                 /* enable ADC1 */
	float pot_perc = read_pot_percent();
	rate = (pot_perc * (MAXRATE - MINRATE)) + MINRATE;
	// reset the period of our wave
	wavepoint_time_space = rate / num_wavepoints_per_cycle; // in ms
	set_sysTick_interrupt(wavepoint_time_space); // reset the wave period when rate is updated
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

void B1_config(void) {
	RCC->AHB1ENR |= 1<<2; // enable GPIOC for push button
	GPIOC->MODER &= ~(0x3<<(2*13)); // clears 2 in/out mode for PC13
	// This should be unnecessary: GPIOC->MODER |= (0<<(2*13)); // this sets bits 2 mode bits for PC13 to input 00
}

/** set up Specific GPIO pin to be general output */
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

/** Turns off PC(pc_num) */
void turn_off_PXnum(int px_num,char GPIO) {
	GPIO_TypeDef * selected_GPIO = select_GPIO(GPIO);
	selected_GPIO->ODR = (0x0<<px_num);
}

/** turns PC(pc_num) to high  */
void turn_on_PXnum(int px_num, char GPIO) {
	GPIO_TypeDef * selected_GPIO = select_GPIO(GPIO);
	selected_GPIO->ODR |= (0x1<<px_num);
}

/** returns the GPIO pointer based on the GPIO char */
GPIO_TypeDef * select_GPIO(char GPIO) {

	if (GPIO == 'A') {
		return GPIOA;
	} else if(GPIO == 'B') {
		return GPIOB;
	} else if (GPIO =='C') {
		return GPIOC;
	} else if (GPIO == 'D') {
		return GPIOD;
	} else {
		return GPIOE;
	}
}
