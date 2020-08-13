/**
  ******************************************************************************
  * @file    LTEmodule.c
  * @author  Ankit
  * @version V1.0
  * @date    07-August-2020
  * @brief   This file contains all the functions defination for the LTE module interface with STM32F4
  ******************************************************************************/

#include "LTEmodule.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*
extern volatile int line_valid;
extern volatile char Rx_String[MAX_RESLEN + 1] ;
extern void USART_SendString(USART_TypeDef *USARTx ,volatile char *str);
// Function prototypes
static void prvSetupLTEmodule(void);


void prvSetupLTEmodule(void) {

	USART_SendString(USART2, "AT\r\n");
	USART_SendString(UART4, "AT\r\n");
	if (line_valid) {
		if (strstr(Rx_String, "OK")) {
			USART_SendString(UART4, "OK received from LTE module\r\n");
			memset(Rx_String, 0, sizeof(Rx_String)); // reset string
			line_valid = 0;
		} else {
			USART_SendString(UART4, Rx_String);
			memset(Rx_String, 0, sizeof(Rx_String)); // reset string
			line_valid = 0;
		}
	} else {
		USART_SendString(UART4, "\nLTE module idle\r\n");
	}

}
*/
