#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "LoadCell.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define TRUE 1
#define FALSE 0
#define MAX_RESLEN 100   // Set Max LTE module response string length
volatile uint8_t connection_flag=0 ; // LTE module connection flag
volatile uint8_t mq2sensor_flag=0 ; // MQ-2 sensor flag
volatile char Rx_String[MAX_RESLEN + 1] ; // Buffer to save LTE module response
volatile int line_valid=0;  // LTE module response flag
volatile uint32_t weight=0;
volatile uint32_t Max_Weight=0;
volatile uint32_t tankstate=0;

// Function prototype
void vHX711_Task(void *pvParameters);
void vSIM_7600_Task(void *pvParameters);
void vMQ2_Sensor_Task(void *pvParameters);
static void prvSetupHardware(void) ;
static void prvSetupUART2(void);
static void prvSetupLTEmodule(void);
void USART_SendString(USART_TypeDef *USARTx ,volatile char *str);
static void prvSetupGPIO(void);

const TickType_t xTickstoDelay= 500/portTICK_PERIOD_MS ;
const TickType_t xTickstoDelay5s= 5000/portTICK_PERIOD_MS ;

			

int main(void)
{
	//1. Reset the RCC clock configuration to he default reset state.
	//HSI ON, PLL OFF,HSE OFF, System Clock=16Mhz, CPU Clock=16MHz
	RCC_DeInit();

	//2. Update the SystemCoreClock variable
	SystemCoreClockUpdate();

	//3. Setup system peripherals and IO pins
	prvSetupHardware();


	//4. Create Tasks
	xTaskCreate(vSIM_7600_Task, "SIM7600_Task", configMINIMAL_STACK_SIZE, NULL,1, NULL);
	xTaskCreate(vMQ2_Sensor_Task, "MQ2_Sensor_Task", configMINIMAL_STACK_SIZE,NULL, 2, NULL);
	xTaskCreate(vHX711_Task, "HX711_Task", configMINIMAL_STACK_SIZE, NULL,2, NULL);

	//5. Start the scheduler
	vTaskStartScheduler();

	return 1 ;

}

void vSIM_7600_Task(void *pvParameters) {

	// force other task to wait until setup finish
	taskENTER_CRITICAL();
	prvSetupLTEmodule();
	taskEXIT_CRITICAL();
	while (1) {

		if (line_valid) {
			USART_SendString(UART4, Rx_String);
			memset(Rx_String, 0, sizeof(Rx_String)); // reset string value
			line_valid = 0;
			vTaskDelay(xTickstoDelay);
		}

		vTaskDelay(xTickstoDelay);

	}
}

void vMQ2_Sensor_Task(void *pvParametrs) {

	GPIO_InitTypeDef mq2pins;
	mq2pins.GPIO_Mode = GPIO_Mode_IN;  // Config GPIO A pin 0 as MQ2 Sensor data pin
	mq2pins.GPIO_Pin = GPIO_Pin_0;
	mq2pins.GPIO_Speed = GPIO_Low_Speed;
	mq2pins.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &mq2pins);  // Set PA0 pins as MQ2 Sensor Input pin

	while (1) {

		mq2sensor_flag = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0); // Read MQ-2 Sensor Pin status flag

		if (mq2sensor_flag) {                                      // if flag is set send notification to LTE module
			USART_SendString(USART2,"AT+CMQTTTOPIC=0,25");
			vTaskDelay(xTickstoDelay);
			USART_SendString(USART2, "Smart_IoT_Scale/GasSensor\r\n"); // Set MQ2 Sensor publish topic
			USART_SendString(USART2,"AT+CMQTTPAYLOAD=0,2");  // Set payload length
			USART_SendString(USART2,"ON");
			USART_SendString(USART2,"AT+CMQTTPUB=0,1,60");  // confirm MQTT message publish
			vTaskDelay(xTickstoDelay);
		}
		else                                               // if flag is not set send notification to LTE module
		{
			USART_SendString(USART2, "AT+CMQTTTOPIC=0,25");
			vTaskDelay(xTickstoDelay);
			USART_SendString(USART2, "Smart_IoT_Scale/GasSensor\r\n"); // Set MQ2 Sensor publish topic
			USART_SendString(USART2, "AT+CMQTTPAYLOAD=0,3"); // Set payload length
			USART_SendString(USART2, "OFF");
			USART_SendString(USART2, "AT+CMQTTPUB=0,1,60"); // confirm MQTT message publish
			vTaskDelay(xTickstoDelay5s);
		}
		}
}
void vHX711_Task(void *pvParameters) {

	taskENTER_CRITICAL();     // force other task to wait until setup finish
	HX711 myLoadCell;
	myLoadCell.PD_SCK_PinType = GPIOC;
	myLoadCell.PD_SCK_PinNumber = GPIO_Pin_4;  // Set GPIOC Pin 4 as SCK
	myLoadCell.DOUT_PinType = GPIOC;
	myLoadCell.DOUT_PinNumber = GPIO_Pin_5;    // Set GPIOC Pin 5 as DOUT
	myLoadCell.gain = 0;
	HX711_Tare(&myLoadCell, 10);  // set off set value
	vTaskDelay(xTickstoDelay);
	taskEXIT_CRITICAL();

	while (1) {
        while(strstr(Rx_String,"Smart_IoT_Scale/Max_Weight"))
        {
        	Max_Weight=(uint32_t)Rx_String ; // set Max_Weight variable as received Max_Weight
        }
		weight = (uint32_t) HX711_GetValue(&myLoadCell);   // read weight value
		tankstate = (uint32_t)((weight *100 )/Max_Weight) ;
		USART_SendString(USART2, "AT+CMQTTTOPIC=0,24");
		vTaskDelay(xTickstoDelay);
		USART_SendString(USART2, "Smart_IoT_Scale/LoadCell\r\n"); // Set LoadCell publish topic
		USART_SendString(USART2, "AT+CMQTTPAYLOAD=0,4");  // Set pay-load length
		USART_SendString(USART2, weight);
		USART_SendString(USART2, "AT+CMQTTPUB=0,1,60");  // confirm MQTT message publish

		vTaskDelay(xTickstoDelay);
		if (strstr(Rx_String, "Smart_IoT_Scale/Reset")) {
			if (strstr(Rx_String, "Reset_Scale")) {
				HX711_Tare(&myLoadCell, 10);              // Reset Scale to zero
			}
		}
	}
}

static void prvSetupHardware(void) {

	prvSetupGPIO();  // Setup GPIO Pins to behave as HX711 DOUT,PD_SCK

	prvSetupUART2();  // Setup UART2 as LTE module connection port

}

//USART2 Driver:- Work as UART
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


static void prvSetupGPIO(void)
{
    //Board Specific function- STM32F407-Disc1
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE) ;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE) ;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE) ;

	GPIO_InitTypeDef  sck_init,data_init ;

    sck_init.GPIO_Mode= GPIO_Mode_OUT;
    sck_init.GPIO_OType= GPIO_OType_PP;
    sck_init.GPIO_Pin= GPIO_Pin_6 ;
    sck_init.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    sck_init.GPIO_Speed=GPIO_Low_Speed;
    GPIO_Init(GPIOA,&sck_init);

    data_init.GPIO_Mode=GPIO_Mode_IN ;
    data_init.GPIO_Pin= GPIO_Pin_7;
    data_init.GPIO_PuPd=GPIO_PuPd_UP;
    data_init.GPIO_Speed=GPIO_Low_Speed;
    GPIO_Init(GPIOA,&data_init);

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

	USART_SendString(USART2, "ATE0\r\n");           // Turn off local echo
	vTaskDelay(xTickstoDelay);

	USART_SendString(USART2, "AT+CMQTTSTART\r\n");  // MQTT service start
	vTaskDelay(xTickstoDelay);

	USART_SendString(USART2, "AT+CMQTTACCQ=0,\"ankit_321\"\r\n");             // set client ID
	vTaskDelay(xTickstoDelay);

	USART_SendString(USART2,
			"AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",90,1\r\n");  // set broker address and port

	vTaskDelay(xTickstoDelay);

	USART_SendString(USART2, "AT+CMQTTSUBTOPIC=0,24,1\r\n");
	vTaskDelay(xTickstoDelay);

	USART_SendString(USART2, "Smart_IoT_Scale/Reset\r\n");       // subscribe to Reset scale topic
	vTaskDelay(xTickstoDelay);

	USART_SendString(USART2, "AT+CMQTTSUBTOPIC=0,26,1\r\n");
	vTaskDelay(xTickstoDelay);

	USART_SendString(USART2, "Smart_IoT_Scale/Max_Weight\r\n");  // subscribe to Max_Weight topic
	vTaskDelay(xTickstoDelay);

	connection_flag = TRUE;                                      // Set connection success flag

}  //End of LTE Module Configuration





// Interrupt handler for USART2
void USART2_IRQHandler(void) {
	traceISR_ENTER();
	static int count = 0;
	static char Rx_Buff[MAX_RESLEN];
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {

		char ch = USART_ReceiveData(USART2); // move data from DR to buffer variable
		//USART_ReceiveData(USART2);
		if ((ch == 0x0A) || (ch == 0x0D)) {
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
	traceISR_EXIT() ;
}
