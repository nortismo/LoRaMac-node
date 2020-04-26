/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
 *
 * \author    Diego Bienz
 */

#include "board.h"
#include "fsl_common.h"
#include "uart-board.h"
#include "board-config.h"
#include "fsl_uart_edma.h"
#include "fsl_dmamux.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "uart.h"

#define UART_DEFAULT_CLK_FREQ 						CLOCK_GetFreq(SYS_CLK)
#define TX_BUFFER_RETRY_COUNT                       10
/* Buffer length of 1 is on purpose, since the transmition only is for one single char */
#define RXTX_DEFAULT_BUFFER_LENGTH 					1

/*!
 * k22UartHandle_t is the struct enabling the user to use three different UARTs of the K22.
 * The handle holds all the information to the UART, the DMA and the buffers.
 * The configuratoin of the handles is static, see the configuration definitions below.
 */
typedef struct {
	UART_Type *type;
	uart_config_t config;
} k22UartHandle_t;

/*!
 * Configurations for the different Uart of the board
 */
/* Configuration for Uart0. NOT USED YET */
static k22UartHandle_t UartHandle0;

/* Configuration for Uart1 */
static k22UartHandle_t UartHandle1 = { .type = DEBUG_UART };

/* Configuration for Uart2. NOT USED YET */
static k22UartHandle_t UartHandle2;

void MapUartIdToHandle(UartId_t uartId, k22UartHandle_t **handle);

/*!
 * CAUTION: Configurations like Pin-Muxing and Clocks need to be preconfigured in BOARD_InitPins.
 * This init function doesn't care about the defined pins
 */
void UartMcuInit(Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx) {
	uart_config_t config;
	k22UartHandle_t *handle;

	obj->UartId = uartId;
	MapUartIdToHandle(uartId, &handle);

	UART_GetDefaultConfig(&config);
	handle->config.baudRate_Bps = DEBUG_UART_BAUDRATE;
	handle->config.enableTx = true;
	handle->config.enableRx = true;

	UART_Init(handle->type, &(handle->config), UART_DEFAULT_CLK_FREQ);
}

void UartMcuConfig(Uart_t *obj, UartMode_t mode, uint32_t baudrate,
		WordLength_t wordLength, StopBits_t stopBits, Parity_t parity,
		FlowCtrl_t flowCtrl) {

	k22UartHandle_t *handle;

	MapUartIdToHandle(obj->UartId, &handle);

	UART_GetDefaultConfig(&(handle->config));

	handle->config.baudRate_Bps = baudrate;

	if (mode == TX_ONLY) {
		handle->config.enableTx = true;
		handle->config.enableRx = false;
	} else if (mode == RX_ONLY) {
		handle->config.enableTx = false;
		handle->config.enableRx = true;
	} else if (mode == RX_TX) {
		handle->config.enableTx = true;
		handle->config.enableRx = true;
	} else {
		for (;;) {
			/*!
			 * You should not reach this state.
			 */
		}
	}

#if defined(FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT) && FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
    switch( stopBits )
    {
    case UART_2_STOP_BIT:
    	handle->config.stopBitCount = kUART_OneStopBit;
        break;
    case UART_1_STOP_BIT:
    default:
    	handle->config.stopBitCount = kUART_TwoStopBit;
        break;
    }
#endif

	if (parity == NO_PARITY) {
		handle->config.parityMode = kUART_ParityDisabled;
	} else if (parity == EVEN_PARITY) {
		handle->config.parityMode = kUART_ParityEven;
	} else {
		handle->config.parityMode = kUART_ParityOdd;
	}

	UART_Init(handle->type, &(handle->config), UART_DEFAULT_CLK_FREQ);
}

void UartMcuDeInit(Uart_t *obj) {
	k22UartHandle_t *handle;
	MapUartIdToHandle(obj->UartId, &handle);

	UART_Deinit(handle->type);
}

uint8_t UartMcuPutChar(Uart_t *obj, uint8_t data) {
	k22UartHandle_t *handle;
	MapUartIdToHandle(obj->UartId, &handle);

	UART_WriteBlocking(handle->type, &data, sizeof(uint8_t));
	return 0;
}

uint8_t UartMcuGetChar(Uart_t *obj, uint8_t *data) {
	k22UartHandle_t *handle;
	MapUartIdToHandle(obj->UartId, &handle);

	return UART_ReadBlocking(handle->type, data, sizeof(uint8_t));
	return 0;
}

uint8_t UartMcuPutBuffer(Uart_t *obj, uint8_t *buffer, uint16_t size) {
	k22UartHandle_t *handle;
	MapUartIdToHandle(obj->UartId, &handle);

	UART_WriteBlocking(handle->type, buffer, size);
	return 0;
}

uint8_t UartMcuGetBuffer(Uart_t *obj, uint8_t *buffer, uint16_t size,
		uint16_t *nbReadBytes) {
	k22UartHandle_t *handle;
	MapUartIdToHandle(obj->UartId, &handle);

	*nbReadBytes = size;

	return UART_ReadBlocking(handle->type, buffer, size);
}

/*!
 * Since the LoRaMac-node stack's Uart enum (UartId_t) goes from Uart1 to Uart3,
 * but the K22 SDK goes from Uart0 to Uart2, a short mapping is required. This
 * method set the pointer "handle" to the corresponding handle of the given
 * uartId.
 */
void MapUartIdToHandle(UartId_t uartId, k22UartHandle_t **handle) {

	switch (uartId) {
	case UART_1:
		*handle = &UartHandle0;
		break;
	case UART_2:
		*handle = &UartHandle1;
		break;
	case UART_3:
		*handle = &UartHandle2;
		break;
	default:
		break;
	}
}

