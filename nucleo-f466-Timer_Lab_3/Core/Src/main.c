// LAB3:  Counters/Timers
// In this lab you will learn how to setup and use STM32F4 timers.
//
// General-purpose timer cookbook for STM32 microcontrollers:
//   https://www.st.com/resource/en/application_note/dm00236305-generalpurpose-timer-cookbook-for-stm32-microcontrollers-stmicroelectronics.pdf
///////////////////////////////////////////////////////////////////
#include "stm32f446xx.h"


/////////////////////////////////////
/*      Functions Declaration      */
/////////////////////////////////////
void delay_ms(uint32_t val);
void delay_us(uint32_t val);
void freq_gen(uint16_t val);
void edge_counter();
void compare_timer(uint32_t val);



// Helping Functions
void USART2_init (void);
void LED_init();
void LED_toggle();


//////////////////////////////////
/*         Main Function        */
//////////////////////////////////
int main(void) {
    LED_init();    // Initialize LED at PA5 as an output


    // TEST CASES:
    // Uncomment one case at a time

    // will blink 5 times
     for (int blink_num=0; blink_num<10; blink_num++){
         // TEST CASE 1:
//          LED_toggle();
//          delay_ms(1000);  // 1sec delay using SysTick

          //// TEST CASE 2:
//          LED_toggle();
//          delay_us(1000000);  // 1sec delay using Timer
     }


    //// TEST CASE 3:
//    freq_gen(1000);


    //// TEST CASE 4:
    //edge_counter();



    //// TEST CASE 5:
    compare_timer(1000);    // Autonomous toggling of PA5 using TIM2 Output Compare Feature

    while (1) {}
}


//////////////////////////////////
/*   User Defined Function      */
//////////////////////////////////
void delay_ms(uint32_t val){
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

	// how do I know which clk I am using??
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




void delay_us(uint32_t val){
    // Using TIM2: A delay function that can delay 1usec to 10 sec
    //
    // In this example will will use the 32bit timer 2 to generate the delay without a for loop.
    // If we prescale the clock to be 1MHz meaning 1usec, then we just need to delay by the val.
    // The range of the delays will be {1usec,  0xFFFF_FFFF*1usec= 4,297.967295sec}

    // Here are the steps to set the counter as a timer to delay x usec
    // 1- Enable the timer clock
    // 2- Set the prescaler to prescale the 16MHz to 1MHz (Note that you need to set it to N-1)
    // 3- Set the auto reload register to val (Note that you need to set it to N-1)
    // 4- Set the counter current value to 0
    // 5- Enable the timer
    // 6- Stall the CPU until the "Update Interrupt Flag" is raised.
    RCC->APB1ENR |= 1;              /* enable TIM2 clock */
    TIM2->PSC = 16-1;               /* divided by 16  (use N-1)*/
    TIM2->ARR = val-1;              /* divided by val */
    TIM2->CNT = 0;                  /* clear timer counter */
    TIM2->CR1 = 1;                  /* enable TIM2 */
    while(!(TIM2->SR & 1));         /* wait until UIF (Update Interrupt Flag) set */

    TIM2->CR1 = 0;                  // Disable the timer
    TIM2->SR &= ~0x1;               // Clear UIF
}



void freq_gen(uint16_t val){
    // Using TIM8: A frequency generator function that generate a 50% duty cycle with a period in millisecond
    // The timer should be used to blink the LED on PA5
    //
    // _____       _____       _____       _____       _____
    //      |_____|     |_____|     |_____|     |_____|     |_____|
    //      |<-- VAL -->|
    //
    // Configure PA5 as output to drive the LED
    // Steps to setup Timer 8 As A frequency generator:
    //   1- Enable Timer Clock
    //   2- Set prescaler (choose a prescale value that could make your live easier?!)
    //   3- Set auto reload register
    //   4- Reset the counter current value
    //   5- Enable the timer
    //   6- In a while loop toggle the LED based UIF
    LED_init();                     // Initialize LED
    //TIM8 would not turn on for some reason so I used TIM1
    RCC->APB2ENR |= 0x1;            /* enable TIM8 clock */
    TIM1->PSC = 16000 - 1;          /* divided by 16000 (Counter input clock is now 1KHz, 1msec) */
    TIM1->ARR = (val/2) -1;         /* We divide val by 2 (=shift right by 1) because duty cycle is 50% */
    TIM1->CNT = 0;                  /* clear timer counter */
    TIM1->CR1 = 1;                  /* enable TIM8 */

    while (1) {                     // Loop forever
        while(!(TIM1->SR & 1)) {}   /* wait until UIF set */
        TIM1->SR = 0;                /* clear UIF */
        LED_toggle();               /* toggle green LED */
    }                               // End while loop
}


void edge_counter(){
    // Use external input PB8 as the TIM2 clock source. Should be connected to the PIN TX of on the STLINK.
    // This way any character sent from the terminal will generate a waveform.
    // Each edge of the input signal
    // increments the TIM2 counter by 1.
    // Timer need to count from positive edge to positive edge
    // Use external input Pin TX on the board (USART2 TX) as the TIM2 clock source.


    USART2_init(); // USART2 is enabled so that you can can generated input for counters
                   // For example if you send the letter 'U' (0x55=0101_0101)from realterm you
                   // a waveform like this one:    (5 neg edges, 5 pos edges)
                   //  _____   _   _   _   _   ______
                   //       \_/ \_/ \_/ \_/ \_/
                   // IDLE   S 0 1 2 3 4 5 6 7   STOP
                   // The receiver channel of USART2 can be accessible from the STLINK TX pin

    // *** YOU MUST CONNECT STLINK TX PIN TO PB8 ***

    RCC->AHB1ENR |= 1;
    GPIOA->MODER &= ~0X00000c00;
    GPIOA->MODER |=  0X00000400;

   // Configure PB8 as input of TIM2 ETR
   RCC->AHB1ENR  |=  2;             /* enable GPIOB clock */
   GPIOB->MODER  &= ~0x00030000;    /* clear pin mode */
   GPIOB->MODER  |=  0x00020000;    /* set pin to alternate function */
   GPIOB->AFR[1] &= ~0x0000000F;    /* clear pin AF bits */
   GPIOB->AFR[1] |=  0x00000001;           /* set pin to AF1 for TIM2 ETR */ // I think this sets thAFRR to 0001

   // Configure TIM2 to use external input as counter clock source
   RCC->APB1ENR |= 1;              /* enable TIM2 clock */
   TIM2->SMCR = 0x4377;            // Set Slave Mode Control Register with the following configuration:
                                   //  - External Clock Mode 1- Rising edges of the selected trigger (TRGI) clock the counter
                                   //  - External Trigger input (ETRF)
                                   //  - External trigger filter = fSAMPLING=fCK_INT, N=8
                                   //  - External trigger prescaler OFF
                                   //  - External clock mode 2 enabled.
   TIM2->CNT = 0;                  /* clear counter */
   TIM2->CR1 = 1;                   /* enable TIM2 to counts up - counts up by default so setting equal to one sets the DIR (bit 4) to 0 */

   while (1){
       // Monitor: TIM2->CNT while you are sending the letter 'U' from terminal
       // you are going to see 0 before you press, then 5 after the 1st 'U',
       // then 10 and so on.
	   if ((USART2->SR)&(1<<5))  { // Check if there is a packet received
	   		char input = (USART2->DR)&0xff;
	   }
	   if (TIM2->CNT & 1) {
		   LED_toggle();
	   }
   }
}




void compare_timer(uint32_t val){
    // Use Timer 2:
    // This is a function to show you how to use the compare functionality of the timers.
    // The function will setup the timer to blink the LED on a val msec periods.
    //
    // Useful linkL https://www.youtube.com/watch?v=BtAi6-7Lnlw
    //
    // Steps to setup the timer in an output compare mode:
    //  1- Enable clock
    //  2- Set prescaler
    //  3- Set auto reload register (using the passed val)
    //  4- Set the Capture/Compare Mode Register to set output to toggle on match
    //  5- Set the match value for the Capture/Compare Reg 1
    //  6- Enable CH1 compare mode which is connected to the PA5 (AF1)
    //  7- Reset the counter current value
    //  8- Enable the timer
    //  No need to do anything else! The toggling of the PA5 is done automatically by the TIM2, the CPU
    //  can do anything else.


    // configure PA5 as output to drive the LED
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */
    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode */
    /*######*/                      /* set pin to alternate function */
    GPIOA->AFR[0] &= 0x00F00000;    /* clear pin AF bits */
    GPIOA->AFR[0] |= 0x00100000;    /* set pin to AF1 for TIM2 CH1 */

    // configure TIM2 to wrap around at 1 Hz
// and toggle CH1 output when the counter value is 0
    RCC->APB1ENR |= 1;              /* enable TIM2 clock */
    /*######*/                      /* divided by 16000 */
    /*######*/                      /* divided by val */
    /*######*/                      /* set output to toggle on match */
    /*######*/                      /* set match value */
    TIM2->CCER |= 1;                /* enable CH1 compare mode */
    TIM2->CNT = 0;                  /* clear counter */
    /*######*/                      /* enable TIM2 */

}


/////////////////////
// Helping Functions
void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */

    /* Configure PA3 for USART2 RX */
    GPIOA->AFR[0] &= ~0xF000;
    GPIOA->AFR[0] |=  0x7000;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x00C0;
    GPIOA->MODER  |=  0x0080;   /* enable alternate function for PA3 */

    USART2->BRR = 0x0683;       /* 9600 baud @ 16 MHz */
    USART2->CR1 = 0x0004;       /* enable Rx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
}


void LED_init(){
    // configure PA5 as output to drive the LED
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */
    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode */
    GPIOA->MODER |=  0x00000400;    /* set pin to output mode */
    GPIOA->BSRR   = (1<<21);        /* Turn LED off           */
}


void LED_toggle(){
    GPIOA->ODR ^=0x20;              /* Toggle LED            */
}
