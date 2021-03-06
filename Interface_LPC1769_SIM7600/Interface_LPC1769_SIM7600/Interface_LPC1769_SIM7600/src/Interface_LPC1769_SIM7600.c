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
#else
#error No UART selected for undefined board
#endif

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 128	// Send Ring-buffer size
#define UART_RRB_SIZE 128	// Receive Ring-Buffer size

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

const char inst1[] = "AT+CMQTTSTART\r\n";  //Establishing MQTT Connection
const char inst2[] = "AT+CMQTTACCQ=0,\"ankit_321\"\r\n";  //Client ID - change this for each client as this need to be unique
const char inst3[] = "AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",90,1\\r\n"; //MQTT Server Name for connecting this client
const char inst4[] = "AT+CMQTTSUBTOPIC=0,9,1" ;
const char inst5[] = "led/subsc" ;
const char inst6[] = "AT+CMQTTSUB=0,4,1,1" ;
const char inst7[] = "ATE0" ;


/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void prvSetupHardware(void);
static void prvSetupUART0(void);
void HANDLER_NAME(void);
/*****************************************************************************
 * Public functions
 ****************************************************************************/




int main(void)
{
	uint8_t key;
	int bytes;

	SystemCoreClockUpdate(); // Update system core clock rate, should be called
	                         // if the system has a clock rate change
	Board_Init();
	Board_SystemInit(); //
	prvSetupHardware() ; // Application specific HW initialization
	Board_LED_Set(0, false);

	/* Send initial messages */
	Chip_UART_SendRB(UART_SELECTION, &txring, inst1, sizeof(inst1) - 1);
	Chip_UART_SendRB(UART_SELECTION, &txring, inst2, sizeof(inst2) - 1);

	/* Poll the receive ring buffer for the ESC (ASCII 27) key */
	key = 0;
	while (key != 27) {
		bytes = Chip_UART_ReadRB(UART_SELECTION, &rxring, &key, 1);
		if (bytes > 0) {
			/* Wrap value back around */
			if (Chip_UART_SendRB(UART_SELECTION, &txring, (const uint8_t *) &key, 1) != 1) {
				Board_LED_Toggle(0);/* Toggle LED if the TX FIFO is full */
			}
		}
	}

	/* DeInitialize UART0 peripheral */
	NVIC_DisableIRQ(IRQ_SELECTION);
	Chip_UART_DeInit(UART_SELECTION);

	return 1;
}
// Do all Project specific HW setup here
static void prvSetupHardware(void)
{
	/* Sets up DEBUG UART */
		DEBUGINIT();

		// Setup LED and Button GPIO
		Board_LED_Init();     // Initialize on-board LED
		Board_Buttons_Init(); //Initialize On-Board button


		/* Setup UART - UART 0 to connect SIM7600
		   UART0 (TXD 0) --- SIM7600 (RXD)
		   UART0 (RXD 0) --- SIM7600 (TXD)    */
		   prvSetupUART0();
}
// Initialize Required UART
static void prvSetupUART0(void)
{
		/* Setup UART0 for 115.2K8N1 */
		   Chip_UART_Init(UART_SELECTION);
		   Chip_UART_SetBaud(UART_SELECTION, 9600);
		   Chip_UART_ConfigData(UART_SELECTION, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
		   Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
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
void HANDLER_NAME(void)
{
	/* Want to handle any errors? Do it here. */

	/* Use default ring buffer handler. Override this with your own
	   code if you need more capability. */
	Chip_UART_IRQRBHandler(UART_SELECTION, &rxring, &txring);
}

/********
 ********
 ********/
