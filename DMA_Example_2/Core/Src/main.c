/* p13_2.c Acquire data from ADC by DMA (in double buffer mode) and then send data to USART via DMA
 *
 * The program sets up the timer TIM2 channel 2 to trigger ADC1 to
 * convert the analog input channel 0. The output of the ADC1 is transferred
 * to the buffer in memory by DMA. Once the buffer if full, the DMA switches
 * to the second buffer and begins filling it.
 * That data are converted to decimal ASCII numbers and sent
 * to USART2 via DMA2.
 *
 * A global variable, done_streaming_in and done_processing_input, are used by the DMA transfer complete interrupt
 * handler to signal the other part of the program that is buffer full and that
 * data conversion is done.
 *
 * This program was tested with STM32F411RE board using STMCube
 */

#include "stm32f4xx.h"
#include "stdio.h"

#define ADCBUFSIZE 64  	// size of DMA memory buffers
#define DATA_SIZE 6		// size of data to be transmitted

void USART2_init (void);    /* Initialize UART pins, Baudrate */
void DMA2_init(void);       /* Initialize DMA2 controller */
void DMA2_Stream0_setup(unsigned int src1, unsigned int src2, unsigned int dst, int len); /* set up a DMA transfer for ADC1 */
void TIM2_init(void);       /* initialize TIM2 */
void ADC1_init(void);       /* setup ADC */
void DMA1_init(void);
void DMA1_Stream6_setup(unsigned int src, unsigned int dst, int len);

const float V_REF = 3.3; //volts
const int RES = 255; // 8 bits
int done = 1;
int done_streaming_in = 1;
int done_processing_input = 1;
char adc_buf0[ADCBUFSIZE];    /* buffer to receive DMA data transfers from ADC conversion results */
char adc_buf1[ADCBUFSIZE];
char uartbuf[ADCBUFSIZE * DATA_SIZE];   /* buffer to hold ASCII numbers for display */

//char transmission_message[ADCBUFSIZE * ];	/*The string that will be sent using uart*/

int main(void) {
    int i;
    char* p;

    USART2_init();
    DMA2_init();
    DMA1_init();
    TIM2_init();
    ADC1_init();

    while(1) {
        done_streaming_in = 0;               /* clear done flag */
        char *available_buf;


        /* start a DMA stream of ADC */
        DMA2_Stream0_setup((uint32_t)adc_buf0,  (uint32_t)adc_buf1, (uint32_t)&(ADC1->DR), ADCBUFSIZE);
        while (done_streaming_in == 0) {}     /* wait for ADC DMA transfer complete - based on interrupt */

        // check which buffer to pull from
        if (DMA2_Stream0->CR & (1<<19)) { // compare with the CT bit as indicator for which DMA buffer is currently being filled
        	available_buf = adc_buf1;
        } else {
        	available_buf = adc_buf0;
        }

        /* convert the ADC data into decimal ASCII numbers for display */
        p = uartbuf;
        for (i = 0; i < ADCBUFSIZE-1; i++) {
        	float data  = available_buf[i]*(V_REF/RES); // convert to voltage
            sprintf(p, "$%.02f;", data);
            p += DATA_SIZE;								// increment pointer DATA_SIZE bytes as
        }

        // then stream from available buffer to usart2
        /* send the message out by USART2 using DMA */
        int size = sizeof(uartbuf);
        while (done_processing_input == 0) {}    /* wait until DMA data transfer is done */
        done_processing_input = 0;               /* clear done flag */
        DMA1_Stream6_setup((unsigned int)uartbuf, (unsigned int)&USART2->DR, size);
    }
}

/*  Initialize ADC
    ADC1 is configured to do 8-bit data conversion and triggered by
    the rising edge of timer TIM2 channel 2 output.
 */
void ADC1_init(void) {
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */
    GPIOA->MODER |=  3;             /* PA0 analog */
    RCC->APB2ENR |= 0x0100;         /* enable ADC1 clock */
    ADC1->CR1 = 0x2000000;          /* 8-bit conversion */
    ADC1->CR2 = 0x13000000;         /* exten rising edge, extsel 3 = tim2.2 */
    ADC1->CR2 |= 0x400;             /* enable setting EOC bit after each conversion */
    ADC1->CR2 |= 1;                 /* enable ADC1 */
}

/*  Initialize TIM2
    Timer TIM2 channel 2 is configured to generate PWM at 1 kHz. The output of
    the timer signal is used to trigger ADC conversion.
 */
void TIM2_init(void) {
    RCC->AHB1ENR |=  2;             /* enable GPIOB clock */
    GPIOB->MODER |=  0x80;          /* PB3 timer2.2 out */
    GPIOB->AFR[0] |= 0x1000;        /* set pin for timer output mode */
    RCC->APB1ENR |= 1;              /* enable TIM2 clock */
    TIM2->PSC = 160 - 1;            /* divided by 160 */
    TIM2->ARR = 100 - 1;            /* divided by 100, sample at 1 kHz */
    TIM2->CNT = 0;
    TIM2->CCMR1 = 0x6800;           /* pwm1 mode,  preload enable */
    TIM2->CCER = 0x10;              /* ch2 enable */
    TIM2->CCR2 = 50 - 1;
}

/*  Initialize DMA2 controller
 *  DMA2 controller's clock is enabled and also the DMA interrupt is
 *  enabled in NVIC.
 */
void DMA2_init(void) {
    RCC->AHB1ENR |= 0x00400000;     /* DMA2 controller clock enable */
    DMA2->HIFCR = 0x003F;           /* clear all interrupt flags of Stream 0 */
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);  /* DMA interrupt enable at NVIC */
}

/*  Set up a DMA transfer for ADC
 *  The ADC1 is connected to DMA2 Stream 0. This function sets up the
 *  peripheral register address, memory address, number of transfers,
 *  data size, transfer direction, and DMA interrupts are enabled.
 *  At the end, the DMA controller is enabled, the ADC conversion
 *  complete is used to trigger DMA data transfer, and the timer
 *  used to trigger ADC is enabled.
 */
void DMA2_Stream0_setup(unsigned int src1, unsigned int src2, unsigned int dst, int len) {
    DMA2_Stream0->CR &= ~1;         /* disable DMA2 Stream 0 */
    while (DMA2_Stream0->CR & 1) {} /* wait until DMA2 Stream 0 is disabled */
    DMA2->HIFCR = 0x003F;           /* clear all interrupt flags of Stream 0 */
    DMA2_Stream0->PAR = dst;
    DMA2_Stream0->M0AR = src1;
    DMA2_Stream0->M1AR = src2;
    DMA2_Stream0->NDTR = len;
    DMA2_Stream0->CR &= (1<<19);	/* Set DMA2 Stream0 Channel 0 for ADC1_0, make sure to preserve the CT bit. //DMA2_Stream0->CR = 0x00000000; would overwrite the CT bit*/
    DMA2_Stream0->CR |= 0x00000400; /* data size byte, mem incr, peripheral-to-mem */
    DMA2_Stream0->CR |= 0x16;       /* enable interrupts DMA_IT_TC | DMA_IT_TE | DMA_IT_DME */
    DMA2_Stream0->FCR = 0;          /* direct mode, no FIFO */
    DMA2_Stream0->CR |= (1<<18);	// this puts the DMA2 into double buffer mode
    DMA2_Stream0->CR |= 1;          /* enable DMA2 Stream 0 */

    ADC1->CR2 |= 0x0100;            /* enable ADC conversion complete DMA data transfer */
    TIM2->CR1 = 1;                  /* enable timer2 */
}


/*  DMA2 Stream0 interrupt handler
    This function handles the interrupts from DMA2 controller Stream0. The error interrupts
    have a placeholder for error handling code. If the interrupt is from DMA data
    transfer complete, the DMA controller is disabled, the interrupt flags are
    cleared, the ADC conversion complete DMA trigger is turned off and the timer
    that triggers ADC conversion is turned off too.
 */
void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->HISR & 0x000C)        /* if an error occurred */
        while(1) {}                 /* substitute this by error handling */

    DMA2_Stream0->CR = 0;           /* disable DMA2 Stream 0 */
    DMA2->LIFCR = 0x003F;           /* clear DMA2 interrupt flags */
    ADC1->CR2 &= ~0x0100;           /* disable ADC conversion complete DMA */
    TIM2->CR1 &= ~1;                /* disable timer2 */

    done_streaming_in = 1;
}

/*  Initialize UART pins, Baudrate
    The USART2 is configured to send output to pin PA2 at 9600 Baud.
 */
void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* enable USART2 clock */

    /* Configure PA2 for USART2_TX */
    GPIOA->AFR[0] &= ~0x0F00;
    GPIOA->AFR[0] |=  0x0700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x0030;
    GPIOA->MODER  |=  0x0020;   /* enable alternate function for PA2 */

    USART2->BRR = 0x8B;       /* 9600 baud @ 16 MHz */
    USART2->CR1 = 0x0008;       /* enable Tx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */

    USART2->SR = ~0x40;         /* clear TC flag */
    USART2->CR1 |= 0x0040;      /* enable transmit complete interrupt */

    NVIC_EnableIRQ(USART2_IRQn);    /* USART2 interrupt enable at NVIC */
}

/*  USART2 interrupt handler
 *  USART2 transmit complete interrupt is used to set the done flag to signal
 *  the other part of the program that the data transfer is done.
 */
void USART2_IRQHandler(void)
{
    USART2->SR &= ~0x0040;          /* clear transmit complete interrupt flag */
    done = 1;                       /* set the done flag */
}



/*  Initialize DMA1 controller
 *  DMA1 controller's clock is enabled and also the DMA interrupt is
 *  enabled in NVIC.
 */
void DMA1_init(void) {
    RCC->AHB1ENR |= 0x00200000;     /* DMA controller clock enable */
    DMA1->HIFCR = 0x003F0000;       /* clear all interrupt flags of Stream 6 */
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);  /* DMA interrupt enable at NVIC */
}

/*  Set up a DMA transfer for USART2
 *  The USART2 is connected to DMA1 Stream 6. This function sets up the
 *  peripheral register address, memory address, number of transfers,
 *  data size, transfer direction, and DMA interrupts are enabled.
 *  At the end, the DMA controller is enabled and the USART2 transmit
 *  DMA is enabled.
 */
void DMA1_Stream6_setup(unsigned int src, unsigned int dst, int len) {
    DMA1_Stream6->CR &= ~1;         /* disable DMA1 Stream 6 */
    while (DMA1_Stream6->CR & 1) {} /* wait until DMA1 Stream 6 is disabled */
    DMA1->HIFCR = 0x003F0000;       /* clear all interrupt flags of Stream 6 */
    DMA1_Stream6->PAR = dst;
    DMA1_Stream6->M0AR = src;
    DMA1_Stream6->NDTR = len;
    DMA1_Stream6->CR = 0x08000000;  /* USART2_TX on DMA1 Stream6 Channel 4 */
    DMA1_Stream6->CR |= 0x00000440; /* data size byte, mem incr, mem-to-peripheral */
    DMA1_Stream6->CR |= 0x16;       /* enable interrupts DMA_IT_TC | DMA_IT_TE | DMA_IT_DME */
    DMA1_Stream6->FCR  = 0;         /* direct mode, no FIFO */
    DMA1_Stream6->CR |= 1;          /* enable DMA1 Stream 6 */

    USART2->SR &= ~0x0040;          /* clear UART transmit complete interrupt flag */
    USART2->CR3 |= 0x80;            /* enable USART2 transmitter DMA */
}

/*  DMA1 Stream6 interrupt handler
    This function handles the interrupts from DMA1 controller Stream6. The error interrupts
    have a placeholder for error handling code. If the interrupt is from DMA data
    transfer complete, the DMA controller is disabled, the interrupt flags are
    cleared.
 */
void DMA1_Stream6_IRQHandler(void)
{
    if (DMA1->HISR & 0x000C0000)    /* if an error occurred */
        while(1) {}                 /* substitute this by error handling */
    DMA1->HIFCR = 0x003F0000;       /* clear all interrupt flags of Stream 6 */
    DMA1_Stream6->CR &= ~0x10;      /* disable DMA1 Stream 6 TCIE */
    done_processing_input = 1;
}
