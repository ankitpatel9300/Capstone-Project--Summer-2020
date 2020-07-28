/*
===============================================================================
 Name        : Smart_IoT_Scale.c
 Author      : $(Ankit)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "stdint.h"
#include "chip.h"
#include "board.h"
#include "string.h"

int main(void) {


    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
        i++ ;
        // "Dummy" NOP to allow source level single
        // stepping of tight while() loop

    }
    return 0 ;
}
