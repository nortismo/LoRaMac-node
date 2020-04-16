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
#define UART_TX_DMA_DEFAULT_CHANNEL 				0U
#define UART_RX_DMA_DEFAULT_CHANNEL 				1U
/* Buffer length of 1 is on purpose, since the transmition only is for one single char */
#define RXTX_DEFAULT_BUFFER_LENGTH 					1

/*!
 * k22UartHandle_t is the struct enabling the user to use three different UARTs of the K22.
 * The handle holds all the information to the UART, the DMA and the buffers.
 * The configuratoin of the handles is static, see the configuration definitions below.
 */
typedef struct {
	UART_Type *type;
	uint32_t baudRate;
	DMAMUX_Type *dmamuxType;
	DMA_Type *dmaType;
	uint32_t txDmaRequestSource;
	uint32_t txDmaRequestChannel;
	uint32_t rxDmaRequestSource;
	uint32_t rxDmaRequestChannel;
	uart_edma_handle_t uartEdmaHandle;
	edma_handle_t uartTxEdmaHandle;
	edma_handle_t uartRxEdmaHandle;
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;
	uint8_t txBuffer[RXTX_DEFAULT_BUFFER_LENGTH];
	uint8_t rxBuffer[RXTX_DEFAULT_BUFFER_LENGTH];
	volatile bool rxBufferEmpty;
	volatile bool txOnGoing;
	volatile bool rxOnGoing;
} k22UartHandle_t;

/*!
 * Configurations for the different Uart of the board
 */
/* Configuration for Uart0. NOT USED YET */
const k22UartHandle_t UartHandle0;

/* Configuration for Uart1 */
k22UartHandle_t UartHandle1 = { .type = DEBUG_UART, .baudRate =
DEBUG_UART_BAUDRATE, .dmamuxType = DEBUG_UART_DMAMUX_BASEADDR, .dmaType =
DEBUG_UART_DMA_BASEADDR, .txDmaRequestSource = DEBUG_UART_TX_DMA_REQUEST,
		.txDmaRequestChannel =
		UART_TX_DMA_DEFAULT_CHANNEL, .rxDmaRequestSource =
		DEBUG_UART_RX_DMA_REQUEST, .rxDmaRequestChannel =
		UART_RX_DMA_DEFAULT_CHANNEL, .rxBufferEmpty = true, .txOnGoing = false,
		.rxOnGoing = false };

/* Configuration for Uart2. NOT USED YET */
const k22UartHandle_t UartHandle2;

void UartUserCallback(UART_Type *base, uart_edma_handle_t *handle,
		status_t status, void *userData);
void UartConfigureEdma(k22UartHandle_t *handle);

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
	config.baudRate_Bps = handle->baudRate;
	config.enableTx = true;
	config.enableRx = true;

	UART_Init(handle->type, &config, UART_DEFAULT_CLK_FREQ);
	UartConfigureEdma(handle);
	handle->sendXfer.data = handle->txBuffer;
	handle->sendXfer.dataSize = RXTX_DEFAULT_BUFFER_LENGTH;
	handle->receiveXfer.data = handle->rxBuffer;
	handle->receiveXfer.dataSize = RXTX_DEFAULT_BUFFER_LENGTH;
}

void UartMcuConfig(Uart_t *obj, UartMode_t mode, uint32_t baudrate,
		WordLength_t wordLength, StopBits_t stopBits, Parity_t parity,
		FlowCtrl_t flowCtrl) {

	uart_config_t config;
	k22UartHandle_t *handle;

	MapUartIdToHandle(obj->UartId, &handle);

	UART_GetDefaultConfig(&config);

	config.baudRate_Bps = baudrate;
	handle->baudRate = baudrate;

	if (mode == TX_ONLY) {
		config.enableTx = true;
		config.enableRx = false;
	} else if (mode == RX_ONLY) {
		config.enableTx = false;
		config.enableRx = true;
	} else if (mode == RX_TX) {
		config.enableTx = true;
		config.enableRx = true;
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
        config.stopBitCount = kUART_OneStopBit;
        break;
    case UART_1_STOP_BIT:
    default:
        config.stopBitCount = kUART_TwoStopBit;
        break;
    }
#endif

	if (parity == NO_PARITY) {
		config.parityMode = kUART_ParityDisabled;
	} else if (parity == EVEN_PARITY) {
		config.parityMode = kUART_ParityEven;
	} else {
		config.parityMode = kUART_ParityOdd;
	}

	UART_Init(handle->type, &config, UART_DEFAULT_CLK_FREQ);
	UartConfigureEdma(handle);
}

void UartMcuDeInit(Uart_t *obj) {
	/* Nothing to do */
}

uint8_t UartMcuPutChar(Uart_t *obj, uint8_t data) {
	CRITICAL_SECTION_BEGIN();

	k22UartHandle_t *handle;
	MapUartIdToHandle(obj->UartId, &handle);

	/* If TX is idle and g_txBuffer is full, start to send data. */
	if (!handle->txOnGoing) {
		memcpy(handle->txBuffer, &data, sizeof(data));
		handle->txOnGoing = true;
		UART_SendEDMA(handle->type, &(handle->uartEdmaHandle),
				&(handle->sendXfer));
		CRITICAL_SECTION_END();
		return 0; // OK
	} else {
		CRITICAL_SECTION_END();
		return 1; // Busy
	}
}

uint8_t UartMcuGetChar(Uart_t *obj, uint8_t *data) {
	CRITICAL_SECTION_BEGIN();

	k22UartHandle_t *handle;
	MapUartIdToHandle(obj->UartId, &handle);

	if ((!handle->rxOnGoing) && !handle->rxBufferEmpty) {
		memcpy(data, handle->rxBuffer, sizeof(data));
		handle->rxBufferEmpty = true;
	} else {
		CRITICAL_SECTION_END();
		return 1; // NOT OK
	}

	/* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
	if ((!handle->rxOnGoing) && handle->rxBufferEmpty) {
		handle->rxOnGoing = true;
		UART_ReceiveEDMA(handle->type, &(handle->uartEdmaHandle),
				&(handle->receiveXfer));
	}
	CRITICAL_SECTION_END();
	return 0; //OK
}

uint8_t UartMcuPutBuffer(Uart_t *obj, uint8_t *buffer, uint16_t size) {
	uint8_t retryCount;
	uint16_t i;

	for (i = 0; i < size; i++) {
		retryCount = 0;
		while (UartPutChar(obj, buffer[i]) != 0) {
			retryCount++;

			// Exit if something goes terribly wrong
			if (retryCount > TX_BUFFER_RETRY_COUNT) {
				return 1; // Error
			}
		}
	}
	return 0; // OK
}

uint8_t UartMcuGetBuffer(Uart_t *obj, uint8_t *buffer, uint16_t size,
		uint16_t *nbReadBytes) {
	uint16_t localSize = 0;

	while (localSize < size) {
		if (UartGetChar(obj, buffer + localSize) == 0) {
			localSize++;
		} else {
			break;
		}
	}

	*nbReadBytes = localSize;

	if (localSize == 0) {
		return 1; // Empty
	}
	return 0; // OK
}

/*!
 * Configuration for the EDMA
 */
void UartConfigureEdma(k22UartHandle_t *handle) {
	edma_config_t config;

	/* Init DMAMUX */
	DMAMUX_Init(handle->dmamuxType);
	/* Set channel for UART */
	DMAMUX_SetSource(handle->dmamuxType, handle->txDmaRequestChannel,
			handle->txDmaRequestSource);
	DMAMUX_SetSource(handle->dmamuxType, handle->rxDmaRequestChannel,
			handle->rxDmaRequestSource);
	DMAMUX_EnableChannel(handle->dmamuxType, handle->txDmaRequestChannel);
	DMAMUX_EnableChannel(handle->dmamuxType, handle->rxDmaRequestChannel);
	/* Init the EDMA module */
	EDMA_GetDefaultConfig(&config);
	EDMA_Init(handle->dmaType, &config);
	EDMA_CreateHandle(&(handle->uartTxEdmaHandle), handle->dmaType,
			handle->txDmaRequestChannel);
	EDMA_CreateHandle(&(handle->uartRxEdmaHandle), handle->dmaType,
			handle->rxDmaRequestChannel);

	/* Create UART DMA handle. */
	UART_TransferCreateHandleEDMA(handle->type, &(handle->uartEdmaHandle),
			UartUserCallback, handle, &(handle->uartTxEdmaHandle),
			&(handle->uartRxEdmaHandle));
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

/*!
 * UART user callback
 */
void UartUserCallback(UART_Type *base, uart_edma_handle_t *handle,
		status_t status, void *userData) {
	k22UartHandle_t *uartHandle = userData;

	/* When the interrupt happened because RX is in idle now,
	 * means that no TX is ongoing anymore.
	 */
	if (kStatus_UART_TxIdle == status) {
		uartHandle->txOnGoing = false;
	}

	/* When the interrupt happened because RX is in idle now,
	 * means that rxBuffer is now filled with data and not empty anymore.
	 */
	if (kStatus_UART_RxIdle == status) {
		uartHandle->rxBufferEmpty = false;
		uartHandle->rxOnGoing = false;
	}
}

