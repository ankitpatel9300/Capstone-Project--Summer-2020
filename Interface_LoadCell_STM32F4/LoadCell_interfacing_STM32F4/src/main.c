#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "LoadCell.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define TRUE 1
#define FALSE 0
#define MAX_RESLEN 100   // Set Max LTE module response string length
volatile uint8_t connection_flag=0 ; // lte module connection flag
volatile uint8_t mq2sensor_flag=0 // mq-2 sensor flag
volatile char Rx_String[MAX_RESLEN + 1] ; // Buffer to save LTE module response
volatile int line_valid=0;
volatile int weight=0;


// Function prototype
void vHX711_Task(void *pvParameters);
void vSIM_7600_Task(void *pvParameters);
void vMQ2_Sensor_Task(void *pvParameters);
static void prvSetupHardware(void) ;
static void prvSetupUART2(void);
static void prvSetupUART4(void);
static void prvSetupLTEmodule(void);
void USART_SendString(USART_TypeDef *USARTx ,volatile char *str);
static void prvSetupGPIO(void);
//void button_handler( void *pvParamaters) ;

const TickType_t xTickstoDelay= 2000/portTICK_PERIOD_MS ;
const TickType_t xTickstoDelay400ms= 400 ;

			

int main(void)
{
	DWT->CTRL |= (1 << 0); //Enable the CYCCNT in DWT_CTRL

	//1. Reset the RCC clock configuration to he default reset state.
	//HSI ON, PLL OFF,HSE OFF, System Clock=16Mhz, CPU Clock=16MHz
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


	prvSetupLTEmodule();


	while(1)
	{


		if (line_valid) {
				USART_SendString(UART4, Rx_String);
				memset(Rx_String, 0, sizeof(Rx_String)); // reset string
				line_valid = 0;
				vTaskDelay(xTickstoDelay) ;
			}


     vTaskDelay(xTickstoDelay);

	}
}

void vMQ2_Sensor_Task(void *pvParametrs){

	while(1){

		vTaskDelay(xTickstoDelay) ;
	}
}
void vHX711_Task(void *pvParameters) {
	// force other task to wait until setup finish
	taskENTER_CRITICAL();
	HX711 myLoadCell;
	myLoadCell.PD_SCK_PinType = GPIOA;
	myLoadCell.PD_SCK_PinNumber = GPIO_Pin_6;
	myLoadCell.DOUT_PinType = GPIOA;
	myLoadCell.DOUT_PinNumber = GPIO_Pin_7;
	myLoadCell.gain = 0;
	taskEXIT_CRITICAL();

	while (1) {
		weight = HX711_Read(&myLoadCell);
		vTaskDelay(xTickstoDelay) ;
		USART_SendString(UART4,(uint16_t)weight);
	}
}

static void prvSetupHardware(void)
{

	// Setup LED and Button GPIO
	prvSetupGPIO();

	// Setup UART2 and UART4
	prvSetupUART2();
	prvSetupUART4();


}

//USART2 Driver:- Work as UART, TODO:- shift this function to specific file
static void prvSetupUART2(void)
{
	    GPIO_InitTypeDef gpio_uart_pins;
		USART_InitTypeDef uart2_init ;
		NVIC_InitTypeDef nvic_init ;

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
	    uart2_init.USART_BaudRate = 9600 ;
	    uart2_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None ;
	    uart2_init.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx ;
	    uart2_init.USART_Parity =USART_Parity_No ;
	    uart2_init.USART_StopBits =USART_StopBits_1 ;
	    uart2_init.USART_WordLength= USART_WordLength_8b ;
	    USART_Init(USART2 ,&uart2_init) ;

	    // Enable NVIC USART_IRQ
	    USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	    nvic_init.NVIC_IRQChannel =USART2_IRQn ;
	    nvic_init.NVIC_IRQChannelPreemptionPriority =0;
	    nvic_init.NVIC_IRQChannelPreemptionPriority =0;
	    nvic_init.NVIC_IRQChannelCmd =ENABLE ;
	    NVIC_Init(&nvic_init);

	    //Enable the USART2 peripheral
	    USART_Cmd(USART2 ,ENABLE) ;

}

//UART4  Driver:- Work as UART, TODO:- shift this function to specific file
static void prvSetupUART4(void)
{
	    GPIO_InitTypeDef gpio_uart4_pins;
		USART_InitTypeDef uart4_init ;
		//NVIC_InitTypeDef nvic_init ;

	   //1.Enable the UART4 and GPIOC peripheral clock
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE) ;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE) ;

	   // Setting PC10-UART4 Tx and PC11-UART4 Rx

	   //2. Alternate function of MCU pins to behave as USART4
		// zeroing each and every member element of the structure
		memset(&gpio_uart4_pins,0,sizeof(gpio_uart4_pins) ) ;
		gpio_uart4_pins.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 ;
		gpio_uart4_pins.GPIO_Mode =GPIO_Mode_AF;
		gpio_uart4_pins.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init( GPIOC, &gpio_uart4_pins) ;

		//3.AF Mode settings for pins
	    GPIO_PinAFConfig(GPIOC ,GPIO_PinSource10 ,GPIO_AF_UART4)  ; //PC10
	    GPIO_PinAFConfig(GPIOC ,GPIO_PinSource11 ,GPIO_AF_UART4)  ; //PC11

	    //4. UART parameter initializations
	    // zeroing each and every member element of the structure
	    memset(&uart4_init,0,sizeof(uart4_init) ) ;
	    uart4_init.USART_BaudRate = 9600 ;
	    uart4_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None ;
	    uart4_init.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx ;
	    uart4_init.USART_Parity =USART_Parity_No ;
	    uart4_init.USART_StopBits =USART_StopBits_1 ;
	    uart4_init.USART_WordLength= USART_WordLength_8b ;
	    USART_Init(UART4 ,&uart4_init) ;


	    //Enable the USART4 peripheral
	    USART_Cmd(UART4 ,ENABLE) ;

}

static void prvSetupGPIO(void)
{
    //Board Specific function- STM32F407-Disc1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE) ;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE) ;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE) ;

	GPIO_InitTypeDef led_init, button_init, sck_init,data_init ;

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

    sck_init.GPIO_Mode= GPIO_Mode_OUT;
    sck_init.GPIO_OType= GPIO_OType_PP;
    sck_init.GPIO_Pin= GPIO_Pin_6 ;
    sck_init.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    sck_init.GPIO_Speed=GPIO_Low_Speed;
    GPIO_Init(GPIOA,&sck_init);

    data_init.GPIO_Mode=GPIO_Mode_IN ;
    data_init.GPIO_Pin= GPIO_Pin_7;
    data_init.GPIO_PuPd=GPIO_PuPd_NOPULL;
    data_init.GPIO_Speed=GPIO_Low_Speed;
    GPIO_Init(GPIOA,&data_init);

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

//Function to send string on USART
void USART_SendString(USART_TypeDef *USARTx ,volatile char *str){
	while(*str){
		while( USART_GetFlagStatus(USARTx,USART_FLAG_TC) == RESET);
	    USART_SendData(USARTx, *str);
	    str++ ;
	}
}

void prvSetupLTEmodule(void) {

	USART_SendString(USART2,"AT\r\n");
	USART_SendString(UART4,"AT\r\n");
	vTaskDelay(xTickstoDelay);

	if (line_valid) {
		if (strstr(Rx_String, "OK"))
		{
			USART_SendString(UART4, "OK\r\n");
			USART_SendString(UART4, "LTE module connected\r\n");
			memset(Rx_String, 0, sizeof(Rx_String)); // reset string
			line_valid = 0;
		}
		}

	USART_SendString(USART2,"ATE0\r\n");
		USART_SendString(UART4,"ATE0\r\n");
		vTaskDelay(xTickstoDelay);

		if (line_valid) {
			if (strstr(Rx_String, "OK"))
			{
				USART_SendString(UART4, "OK\r\n");
				USART_SendString(UART4, "ECHO OFF\r\n");
				memset(Rx_String, 0, sizeof(Rx_String)); // reset string
				line_valid = 0;
			}
			}
	USART_SendString(USART2, "AT+CMQTTSTART\r\n");
	USART_SendString(UART4, "AT+CMQTTSTART\r\n");
	vTaskDelay(xTickstoDelay);

	if (line_valid) {
		if (strstr(Rx_String, "OK")) {
			USART_SendString(UART4, "MQTT service start\r\n");
			memset(Rx_String, 0, sizeof(Rx_String)); // reset string
			line_valid = 0;
		}
		else {
			USART_SendString(UART4,Rx_String);
			USART_SendString(UART4,Rx_String);
			memset(Rx_String, 0, sizeof(Rx_String)); // reset string
			line_valid = 0;
		}
	}
	/* ***********  */
	USART_SendString(USART2, "AT+CMQTTACCQ=0,\"ankit_321\"\r\n");
	USART_SendString(UART4, "AT+CMQTTACCQ=0,\"ankit_321\"\r\n");
	vTaskDelay(xTickstoDelay);
	if (line_valid) {
		USART_SendString(UART4, Rx_String);
		memset(Rx_String, 0, sizeof(Rx_String)); // reset string
		line_valid = 0;
	}

	/* ***********  */
	USART_SendString(USART2,
			"AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",90,1\r\n");
	USART_SendString(UART4,
			"AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",90,1\r\n");
	vTaskDelay(xTickstoDelay);
	if (line_valid) {
		USART_SendString(UART4, Rx_String);
		memset(Rx_String, 0, sizeof(Rx_String)); // reset string
		line_valid = 0;
	}

	/* ***********  */
	USART_SendString(USART2, "AT+CMQTTSUBTOPIC=0,24,1\r\n");
	USART_SendString(UART4, "AT+CMQTTSUBTOPIC=0,24,1\r\n");
	vTaskDelay(xTickstoDelay);
	if (line_valid) {
		USART_SendString(UART4, Rx_String);
		memset(Rx_String, 0, sizeof(Rx_String)); // reset string
		line_valid = 0;
	}

	/* ***********  */
	USART_SendString(USART2, "Smart_IoT_Scale/LoadCell\r\n");
	USART_SendString(UART4, "Smart_IoT_Scale/LoadCell\r\n");
	vTaskDelay(xTickstoDelay);
	if (line_valid) {
		USART_SendString(UART4, Rx_String);
		memset(Rx_String, 0, sizeof(Rx_String)); // reset string
		line_valid = 0;
	}

	/* ***********  */
	USART_SendString(USART2, "AT+CMQTTSUB=0,4,1,1\r\n");
	USART_SendString(UART4, "AT+CMQTTSUB=0,4,1,1\r\n");
	vTaskDelay(xTickstoDelay);
	if (line_valid) {
		USART_SendString(UART4, Rx_String);
		memset(Rx_String, 0, sizeof(Rx_String)); // reset string
		line_valid = 0;
	}

		/* ***********  */
	USART_SendString(USART2, "HAII\r\n");
	USART_SendString(UART4, "HAII\r\n");
	vTaskDelay(xTickstoDelay);
	if (line_valid) {
		if (strstr(Rx_String, "OK")) {
			USART_SendString(UART4, "LTE module Setup Done\r\n");
			memset(Rx_String, 0, sizeof(Rx_String)); // reset string
			line_valid = 0;
		}

	}

    connection_flag= TRUE;

}  //End of LTE Module Configuration


/*
static void prvSetupLoadCell(void)
{
	HX711 myLoadCell ;
	myLoadCell.PD_SCK_PinType= GPIOA;
	myLoadCell.PD_SCK_PinNumber= GPIO_Pin_6;
	myLoadCell.DOUT_PinType= GPIOA ;
	myLoadCell.DOUT_PinNumber=GPIO_Pin_7;
	myLoadCell.gain=0;

}  */


//GPIO Pin Interrupt handler
void EXTI0_IRQHandler (void)
{
	traceISR_ENTER();
  // 1.Clear the interrupt pending bit of EXTI line 0
	EXTI_ClearITPendingBit(EXTI_Line0);
	//MQ-2_Sensor_handler(NULL);
	traceISR_EXIT() ;

}

// Interrupt handler for USART2
void USART2_IRQHandler(void) {
	static int count = 0;
	static char Rx_Buff[MAX_RESLEN];
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {

		char ch = USART_ReceiveData(USART2); // move data from DR to buffer variable
		//USART_ReceiveData(USART2);
		if ((ch == '\n') || (ch == '\r')) {
			if (count != 0) {
				Rx_Buff[count++]='\n';
				Rx_Buff[count++]='\r';
				memcpy((void*) Rx_String, Rx_Buff, count);
				Rx_Buff[count] = 0; //terminate the string
				line_valid = 1; // indicate new line is received
				count = 0;  //Reset counter
			}
		}
		else {
			if (count == MAX_RESLEN)
				count = 0; // if cross max length reset counter
			Rx_Buff[count++] = ch;  //save received char in buffer
		}
	}
}
