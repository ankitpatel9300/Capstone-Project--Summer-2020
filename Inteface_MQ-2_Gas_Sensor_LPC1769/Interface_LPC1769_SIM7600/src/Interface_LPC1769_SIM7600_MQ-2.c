#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "stdint.h"
#include "chip.h"
#include "board.h"
#include "string.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#if defined(BOARD_NXP_LPCXPRESSO_1769)
#define UART_SELECTION 	LPC_UART0
#define IRQ_SELECTION 	UART0_IRQn
#define HANDLER_NAME 	UART0_IRQHandler
#endif

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
volatile uint8_t key;
volatile int bytes;
volatile uint8_t sensorStatus;

#define MQ2_GPIO_PORT_NUM 2
#define MQ2_GPIO_BIT_NUM  7
#define RED_LED    0
#define GREEN_LED  1
#define BLUE_LED   2

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 1024	// Send Ring-buffer size
#define UART_RRB_SIZE 1024	// Receive Ring-Buffer size

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

char rxDataBuff[UART_RRB_SIZE];
const portTickType xDelay500ms = portTICK_RATE_MS * 500;

size_t length_rxDatabuff = sizeof(rxDataBuff) / sizeof(rxDataBuff[0]);

const char inst1[] = "AT+CMQTTSTART\r\n";  //Establishing MQTT Connection
const char inst2[] = "AT+CMQTTACCQ=0,\"ankit_321\"\r\n"; //Client ID - change this for each client as this need to be unique
const char inst3[] = "AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",90,1\r\n"; //MQTT Server Name for connecting the client
const char inst4[] = "AT+CMQTTSUBTOPIC=0,9,1\r\n";
const char inst5[] = "mq-2_sensor\r\n";  // MQ-2 Sensor publish Topic name
const char inst6[] = "ATE0\r\n"; //Turn-off local Echo
const char inst7[] = "AT\r\n"; // check AT response
const char inst8[] = "AT+CMQTTTOPIC=0,8\r\n"; // MQTT topic
const char inst9[] = "led/publ\r\n"; // Publish Topic
const char inst10[] = "AT+CMQTTPAYLOAD=0,6\r\n"; //Payload Length
const char inst11[] = "Gas Detected\r\n"; //Payload Message
const char inst12[] = "No Gas Detected\r\n";
const char inst13[] = "AT+CMQTTPUB=0,1,60\r\n"; // Acknowledgment
const char inst14[] = "AT+CMQTTSUB=0,4,1,1\r\n"; //Sub
const char inst15[] = "HAII\r\n";

/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void prvSetupHardware(void);
static void prvSetupUART0(void);
void HANDLER_NAME(void);
/*****************************************************************************
 * Public functions
 ****************************************************************************/

/*****************************************************************************
 * Task Prototypes
 ****************************************************************************/
void vSIM_7600_Task(void *pvParameters);
void vMQ2_Sensor_Task(void *pvParameters) ;
//TODO :- Load cell task prototype

int main(void) {

	SystemCoreClockUpdate(); // Update system core clock rate, should be called
	// if the system has a clock rate change
	Board_Init();
	Board_SystemInit();      // Setup Pin Muxing and system clock
	prvSetupHardware();     // Application specific HW initialization

	// Create Tasks
	xTaskCreate(vSIM_7600_Task, "SIM7600_Task", configMINIMAL_STACK_SIZE, NULL,1, NULL);
	xTaskCreate(vMQ2_Sensor_Task, "MQ2_Sensor_Task", configMINIMAL_STACK_SIZE, NULL,2, NULL);
	//TODO:- Create Load-cell Tasks here

	//4. Start the scheduler
	vTaskStartScheduler();

	/**** Should never arrive here ******/

	/* DeInitialize UART0 peripheral */
	NVIC_DisableIRQ(IRQ_SELECTION);
	Chip_UART_DeInit(UART_SELECTION);

	return 1;
}

// SIM7600_LTE_Module Thread
void vSIM_7600_Task(void *pvParameters) {

	vTaskDelay(xDelay500ms * 4);
	//portTickType xLastWakeTime = xTaskGetTickCount();
	//const portTickType xDelay500ms = portTICK_RATE_MS * 500;

	Chip_UART_SendRB(UART_SELECTION, &txring, inst6, sizeof(inst6) - 1); // ATE0
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);
	vTaskDelay(xDelay500ms);

	Chip_UART_SendRB(UART_SELECTION, &txring, inst1, sizeof(inst1) - 1); // AT+CMQTTSTART
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);
	vTaskDelay(xDelay500ms);

	Chip_UART_SendRB(UART_SELECTION, &txring, inst2, sizeof(inst2) - 1); // AT+CMQTTACCQ   Client ID
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);
	vTaskDelay(xDelay500ms);

	Chip_UART_SendRB(UART_SELECTION, &txring, inst3, sizeof(inst3) - 1); // Set Broker
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);
	vTaskDelay(xDelay500ms);

	Chip_UART_SendRB(UART_SELECTION, &txring, inst4, sizeof(inst4) - 1); // Set Sub topic length
	Board_LED_Set(RED_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(RED_LED, true);
	vTaskDelay(xDelay500ms);

	Chip_UART_SendRB(UART_SELECTION, &txring, inst5, sizeof(inst5) - 1); // Sub Topic
	Board_LED_Set(RED_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(RED_LED, true);
	vTaskDelay(xDelay500ms);

	Chip_UART_SendRB(UART_SELECTION, &txring, inst14, sizeof(inst14) - 1); //
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);

	Chip_UART_SendRB(UART_SELECTION, &txring, inst15, sizeof(inst15) - 1); //
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);

	Chip_UART_SendRB(UART_SELECTION, &txring, inst8, sizeof(inst8) - 1); // MQTT TOPIC
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);
	vTaskDelay(xDelay500ms);
	Chip_UART_SendRB(UART_SELECTION, &txring, inst9, sizeof(inst9) - 1); // PUBLISH TOPIC NAME
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);
	vTaskDelay(xDelay500ms);
	Chip_UART_SendRB(UART_SELECTION, &txring, inst10, sizeof(inst10) - 1); // MQTT PAYLOAD LENGTH
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);
	vTaskDelay(xDelay500ms);
	Chip_UART_SendRB(UART_SELECTION, &txring, inst11, sizeof(inst11) - 1); // MQTT MSG
	Board_LED_Set(BLUE_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(BLUE_LED, true);
	vTaskDelay(xDelay500ms);
	Chip_UART_SendRB(UART_SELECTION, &txring, inst13, sizeof(inst13) - 1); // Acknowledgment of Msg
	Board_LED_Set(GREEN_LED, false);
	vTaskDelay(xDelay500ms * 4);
	Board_LED_Set(GREEN_LED, true);

	//Chip_UART_ReadRB(UART_SELECTION, &rxring, rxDataBuff, sizeof(rxDataBuff));
	memset(rxDataBuff, 0, length_rxDatabuff);  //Flush the rxData  */

	while (1) {

		if(sensorStatus==1)
		{
		    Chip_UART_SendRB(UART_SELECTION, &txring, inst9, sizeof(inst9) - 1); // PUBLISH TOPIC NAME
			Board_LED_Set(GREEN_LED, false);
			vTaskDelay(xDelay500ms * 4);
			Board_LED_Set(GREEN_LED, true);
			vTaskDelay(xDelay500ms);
			Chip_UART_SendRB(UART_SELECTION, &txring, inst10,sizeof(inst10) - 1); // MQTT PAYLOAD LENGTH
			Board_LED_Set(GREEN_LED, false);
			vTaskDelay(xDelay500ms * 4);
			Board_LED_Set(GREEN_LED, true);
			vTaskDelay(xDelay500ms);
			Chip_UART_SendRB(UART_SELECTION, &txring, inst11,sizeof(inst11) - 1); // MQTT MSG Gas Detected
			Board_LED_Set(BLUE_LED, false);
			vTaskDelay(xDelay500ms * 4);
			Board_LED_Set(BLUE_LED, true);
		}
		else
		{
		    Chip_UART_SendRB(UART_SELECTION, &txring, inst9, sizeof(inst9) - 1); // PUBLISH TOPIC NAME
			Board_LED_Set(GREEN_LED, false);
			vTaskDelay(xDelay500ms * 4);
			Board_LED_Set(GREEN_LED, true);
			vTaskDelay(xDelay500ms);
			Chip_UART_SendRB(UART_SELECTION, &txring, inst10,sizeof(inst10) - 1); // MQTT PAYLOAD LENGTH
			Board_LED_Set(GREEN_LED, false);
			vTaskDelay(xDelay500ms * 4);
			Board_LED_Set(GREEN_LED, true);
			vTaskDelay(xDelay500ms);
			Chip_UART_SendRB(UART_SELECTION, &txring, inst11,sizeof(inst11) - 1); // MQTT MSG  No Gas detected
			Board_LED_Set(BLUE_LED, false);
			vTaskDelay(xDelay500ms * 4);
			Board_LED_Set(BLUE_LED, true);
		}

	}
}
// MQ-2 Gas Sensor Thread
void vMQ2_Sensor_Task(void *pvParameters)
 {

	vTaskDelay(xDelay500ms*2);  // 1 Sec Delay to Allow gas sensor to heat up coil.
	while (1) {
		if (Chip_GPIO_GetPinState(LPC_GPIO2, MQ2_GPIO_PORT_NUM,MQ2_GPIO_BIT_NUM)) {
			sensorStatus = 1; // If MQ2 pin is high set status as 1
			vTaskDelay(xDelay500ms);
		} else {
			sensorStatus = 0; // else MQ2 sensor status is 0
			vTaskDelay(xDelay500ms);
		}
	}
 }

// Do all Project specific HW setup here
static void prvSetupHardware(void) {
	/* Sets up DEBUG UART */
	DEBUGINIT();

	// Setup LED and Button GPIO
	Board_LED_Init();     // Initialize on-board LED
	Board_LED_Set(RED_LED, true);
	Board_LED_Set(GREEN_LED, true);
	Board_LED_Set(BLUE_LED, true);
	Board_Buttons_Init(); //Initialize On-Board button

	//Chip_GPIO_Init(LPC_GPIO2) ; // Enable peripheral clock for GPIO2
	//Chip_GPIO_SetPinDIRInput(LPC_GPIO2,MQ2_GPIO_PORT_NUM, MQ2_GPIO_BIT_NUM); // Set GPIO2.17 pin as input

	/* Setup UART - UART 0 to connect SIM7600
	 UART0 (TXD 0) --- SIM7600 (RXD)
	 UART0 (RXD 0) --- SIM7600 (TXD)    */
	prvSetupUART0();
}
// Initialize Required UART
static void prvSetupUART0(void) {
	/* Setup UART0 for 115.2K8N1 */
	Chip_UART_Init(UART_SELECTION);
	Chip_UART_SetBaud(UART_SELECTION, 9600);
	Chip_UART_ConfigData(UART_SELECTION, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(UART_SELECTION,(UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(UART_SELECTION);

	/* Before using the ring buffers, initialize them using the ring buffer init function */
	RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

	/* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
	Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |
	UART_FCR_TX_RS | UART_FCR_TRG_LEV3));

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(UART_SELECTION, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(IRQ_SELECTION, 1);
	NVIC_EnableIRQ(IRQ_SELECTION);
}
/**
 * @brief	UART 0 interrupt handler using ring buffers
 * @return	Nothing
 */
void HANDLER_NAME(void) {
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
	 code if you need more capability. */
	Chip_UART_IRQRBHandler(UART_SELECTION, &rxring, &txring);
}

//TODO: Implement this function to optimize SIM7600 Task.
/*void atCommandTx(char *atCommand)
 {
 const portTickType xDelay500ms = portTICK_RATE_MS * 500;
 char *inCommand= atCommand ;
 Chip_UART_SendRB(UART_SELECTION, &txring, inCommand, sizeof(inCommand) - 1); // ATE0
 Board_LED_Set(GREEN_LED,false) ;
 vTaskDelay(xDelay500ms*4);
 Board_LED_Set(GREEN_LED,true) ;
 vTaskDelay(xDelay500ms);
 }
 */
