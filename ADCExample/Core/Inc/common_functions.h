// common_functions.h
// This file holds all the functions are general enough to use in many projects
#include "stm32f4xx.h"

void toggle_pin_5(void);
void LED_init();

/**
 * Configures PC13 (push button) to be a set to input mode 00
 */
void B1_config(void);

/**
 * delay for milli seconds
 * Note: This function the sysTick timer to delay
 */
void delay_ms(uint32_t val);

/**
 * Configure the USART2
 * Sets the baud rate to 16,000,000
 */
void UART2_init(void);

/**
 * enable and initialize the rx UART2
 */
void UART2_rx_init(void);

/**
 * enable and initialize the tx UART2
 */
void UART2_tx_init(void);

/**
 * Configures PXy (where X is the pin bus (A,B,C,D,E) and y = py_num (1-15)) to be
 * General Purpose output mode 01
 *
 * @returns -1 when input is not valid and 0 otherwise
 */
int PXx_OUT_MODER_config(int PX_num, char GPIO);

void turn_on_PXnum(int px_num, char GPIO);

void turn_off_PXnum(int px_num, char GPIO);

void set_sysTick_interrupt(int clk_div);
