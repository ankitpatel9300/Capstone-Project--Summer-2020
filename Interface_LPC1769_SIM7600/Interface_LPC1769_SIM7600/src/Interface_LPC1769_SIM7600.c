
#include "chip.h"
#include "board.h"
#include "string.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#if defined(BOARD_EA_DEVKIT_1788) || defined(BOARD_EA_DEVKIT_4088)
#define UART_SELECTION 	LPC_UART0
#define IRQ_SELECTION 	UART0_IRQn
#define HANDLER_NAME 	UART0_IRQHandler
#elif defined(BOARD_NXP_LPCXPRESSO_1769)
#define UART_SELECTION 	LPC_UART0
#define IRQ_SELECTION 	UART0_IRQn
#define HANDLER_NAME 	UART0_IRQHandler
#else
#error No UART selected for undefined board
#endif

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* Transmit and receive ring buffers */
STATIC RINGBUFF_T txring, rxring;

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 128	// Send Ring-buffer size
#define UART_RRB_SIZE 128	// Receive Ring-Buffer size

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

const char inst1[] = "LPC17xx/40xx UART example using ring buffers\r\n";
const char inst2[] = "Press a key to echo it back or ESC to quit\r\n";



/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void prvSetupHardware(void);
static void prvSetupUART0(void);

/*****************************************************************************
 * Public functions
 ****************************************************************************/

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

/**
 * @brief	Main UART program body
 * @return	Always returns 1
 */
int main(void)
{
	uint8_t key;
	int bytes;

	SystemCoreClockUpdate();
	Board_Init();
	Board_UART_Init(UART_SELECTION);
	Board_LED_Set(0, false);





	/* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
	Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |
							UART_FCR_TX_RS | UART_FCR_TRG_LEV3));

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(UART_SELECTION, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(IRQ_SELECTION, 1);
	NVIC_EnableIRQ(IRQ_SELECTION);

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

static void prvSetupHardware(void)
{
		// Setup LED and Button GPIO
		   prvSetupGPIO();
		/* Setup UART - UART 0 to connect SIM7600
		   UART0 (TXD 0) --- SIM7600 (RXD)
		   UART0 (RXD 0) --- SIM7600 (TXD)
		*/
		   prvSetupUART0();
}
static void prvSetupUART0(void)
{
		/* Setup UART0 for 115.2K8N1 */
		   Chip_UART_Init(UART_SELECTION);
		   Chip_UART_SetBaud(UART_SELECTION, 115200);
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
static void prvSetupGPIO(void);
/**
 * @}
 */
