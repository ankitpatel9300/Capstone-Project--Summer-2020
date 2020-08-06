#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define TRUE 1
#define FALSE 0

// function prototype
void vHX711_Task(void *pvParameters);
void vSIM_7600_Task(void *pvParameters);
void vMQ2_Sensor_Task(void *pvParameters);
static void prvSetupHardware(void) ;
static void prvSetupUART2(void) ;
void prvSetupGPIO(void);
void button_handler( void *pvParamaters) ;



			

int main(void)
{
	DWT->CTRL |= (1 << 0); //Enable the CYCCNT in DWT_CTRL

	//1. Reset the RCC clock configuration to he default reset state.
	//HSI ON, OLL OFF,HSE OFF, System Clock=16Mhz, CPU Clock=16MHz
	RCC_DeInit();

	//2. Update the SystemCoreClock variable
	SystemCoreClockUpdate();

	prvSetupHardware();

	// Start Recording
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	//3. Create Tasks
	xTaskCreate(vSIM_7600_Task, "SIM7600_Task", configMINIMAL_STACK_SIZE, NULL,1, NULL);
	xTaskCreate(vMQ2_Sensor_Task, "MQ2_Sensor_Task", configMINIMAL_STACK_SIZE,NULL, 3, NULL);
	xTaskCreate(vHX711_Task, "HX711_Task", configMINIMAL_STACK_SIZE, NULL,2, NULL);

	//4. Start the scheduler
	vTaskStartScheduler();

	return 1 ;

}

void vSIM_7600_Task(void *pvParameters) {
	while(1)
	{

	}
}

void vHX711_Task(void *pvParameters)
{
	while(1)
	{

	}
}
static void prvSetupHardware(void)
{
	// Setup LED and Button GPIO
	prvSetupGPIO();

	// Setup UART2
	prvSetupUART2();
}

static void prvSetupUART2(void)
{
	    GPIO_InitTypeDef gpio_uart_pins;
		USART_InitTypeDef uart2_init ;

	   //1.Enable the UART2 and GPIOA peripheral clock
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE) ;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE) ;

	   // Setting PA2-USART Tx and PA3-USART Rx

	   //2. Alternate function of MCU pins to behave as USART2
		// zeroing each and every member element of the structure
		memset(&gpio_uart_pins,0,sizeof(gpio_uart_pins) ) ;
		gpio_uart_pins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 ;
		gpio_uart_pins.GPIO_Mode =GPIO_Mode_AF;
		gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init( GPIOA, &gpio_uart_pins) ;

		//3.AF Mode settings for pins
	    GPIO_PinAFConfig(GPIOA ,GPIO_PinSource2 ,GPIO_AF_USART2)  ; //PA2
	    GPIO_PinAFConfig(GPIOA ,GPIO_PinSource3 ,GPIO_AF_USART2)  ; //PA3

	    //4. UART parameter initializations
	    // zeroing each and every member element of the structure
	    memset(&uart2_init,0,sizeof(uart2_init) ) ;
	    uart2_init.USART_BaudRate =115200 ;
	    uart2_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None ;
	    uart2_init.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx ;
	    uart2_init.USART_Parity =USART_Parity_No ;
	    uart2_init.USART_StopBits =USART_StopBits_1 ;
	    uart2_init.USART_WordLength= USART_WordLength_8b ;
	    USART_Init(USART2 ,&uart2_init) ;

	    //Enable the USART2 peripheral
	    USART_Cmd(USART2 ,ENABLE) ;

}

void prvSetupGPIO(void)
{
    //Board Specific function- STM32F407-Disc1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE) ;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE) ;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE) ;

	GPIO_InitTypeDef led_init, button_init ;

	led_init.GPIO_Mode = GPIO_Mode_OUT;
	led_init.GPIO_OType =GPIO_OType_PP ;
	led_init.GPIO_Pin = GPIO_Pin_12;
	led_init.GPIO_Speed = GPIO_Low_Speed ;
    led_init.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOD, &led_init);

    button_init.GPIO_Mode =GPIO_Mode_IN ;
    button_init.GPIO_OType =GPIO_OType_PP ;
    button_init.GPIO_Pin= GPIO_Pin_0 ;
    button_init.GPIO_Speed= GPIO_Low_Speed ;
    button_init.GPIO_PuPd =GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &button_init);

    //interrupt configuration for button  (PA0)
    // 1. System configuration for EXTI line (SYSCFG settings)
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA ,EXTI_PinSource0) ;

    // 2. EXTI line configuration 0 , falling EDGE , interrupt mode
    EXTI_InitTypeDef exti_init ;
    exti_init.EXTI_Line =EXTI_Line0 ;
    exti_init.EXTI_Mode=EXTI_Mode_Interrupt ;
    exti_init.EXTI_Trigger= EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd =ENABLE;
    EXTI_Init(&exti_init) ;

    // 3. NVIC settings (IRQ settings for selected EXTI line 0)
    NVIC_SetPriority(EXTI0_IRQn ,5) ;
    NVIC_EnableIRQ(EXTI0_IRQn) ;
}

void EXTI0_IRQHandler (void)
{
	traceISR_ENTER();
  // 1.Clear the interrupt pending bit of EXTI line 0
	EXTI_ClearITPendingBit(EXTI_Line0);
	button_handler(NULL);
	traceISR_EXIT() ;

}
