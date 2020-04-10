/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
 *
 * \author    Diego Bienz
 */

#include "board-config.h"
#include "fsl_uart_edma.h"
#include "fsl_dmamux.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "uart.h"

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
#define TX_BUFFER_RETRY_COUNT                       10
#define UART_SRC_CLK CLOCK_GetFreq(SYS_CLK)
#define UART UART1
#define UART_DMAMUX_BASEADDR DMAMUX0
#define UART_DMA_BASEADDR DMA0
#define UART_TX_DMA_CHANNEL 0U
#define UART_RX_DMA_CHANNEL 1U
#define UART1_TX_DMA_REQUEST kDmaRequestMux0UART1Tx
#define UART1_RX_DMA_REQUEST kDmaRequestMux0UART1Rx
#define RXTX_BUFFER_LENGTH 1

uart_edma_handle_t g_uartEdmaHandle;
edma_handle_t g_uartTxEdmaHandle;
edma_handle_t g_uartRxEdmaHandle;
uart_transfer_t sendXfer;
uart_transfer_t receiveXfer;
uint8_t g_txBuffer[RXTX_BUFFER_LENGTH] =
  { 0 };
uint8_t g_rxBuffer[RXTX_BUFFER_LENGTH] =
  { 0 };

volatile bool rxBufferEmpty = true;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;

/* The MK22 has 3 Uart and the index is 0 based. This function makes the MK22 compatible to the stack */
UART_Type *UartBaseAddresses[3] =
  { UART0, UART1, UART2 };

//TODO: Make it dynamic. Currently it is only possible to use UART1

void
UartUserCallback (UART_Type *base, uart_edma_handle_t *handle, status_t status,
		  void *userData);
void
UartConfigureEdma ();

/*!
 * CAUTION: Configurations like Pin-Muxing and Clocks need to be preconfigured in BOARD_InitPins
 */
void
UartMcuInit (Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx)
{
  uart_config_t config;

  obj->UartId = uartId;

  UART_GetDefaultConfig (&config);
  config.baudRate_Bps = BOARD_DEFAULT_UART_BAUDRATE;
  config.enableTx = true;
  config.enableRx = true;

  UART_Init (UartBaseAddresses[uartId], &config, UART_SRC_CLK);
  void
  UartConfigureEdma ();
  sendXfer.data = g_txBuffer;
  sendXfer.dataSize = RXTX_BUFFER_LENGTH;
  receiveXfer.data = g_rxBuffer;
  receiveXfer.dataSize = RXTX_BUFFER_LENGTH;
}

void
UartMcuConfig (Uart_t *obj, UartMode_t mode, uint32_t baudrate,
	       WordLength_t wordLength, StopBits_t stopBits, Parity_t parity,
	       FlowCtrl_t flowCtrl)
{
  uart_config_t config;
  UART_GetDefaultConfig (&config);

  config.baudRate_Bps = baudrate;

  if (mode == TX_ONLY)
    {
      config.enableTx = true;
      config.enableRx = false;
    }
  else if (mode == RX_ONLY)
    {
      config.enableTx = false;
      config.enableRx = true;
    }
  else if (mode == RX_TX)
    {
      config.enableTx = true;
      config.enableRx = true;
    }
  else
    {
      for (;;)
	{
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

  if (parity == NO_PARITY)
    {
      config.parityMode = kUART_ParityDisabled;
    }
  else if (parity == EVEN_PARITY)
    {
      config.parityMode = kUART_ParityEven;
    }
  else
    {
      config.parityMode = kUART_ParityOdd;
    }

  UART_Init (UartBaseAddresses[obj->UartId], &config, UART_SRC_CLK);
  void
  UartConfigureEdma ();
}

void
UartMcuDeInit (Uart_t *obj)
{
  /* Nothing to do */
}

uint8_t
UartMcuPutChar (Uart_t *obj, uint8_t data)
{
  CRITICAL_SECTION_BEGIN ();

  /* If TX is idle and g_txBuffer is full, start to send data. */
  if (!txOnGoing)
    {
      memcpy (g_txBuffer, &data, sizeof(data));
      txOnGoing = true;
      UART_SendEDMA (UartBaseAddresses[obj->UartId], &g_uartEdmaHandle,
		     &sendXfer);
      CRITICAL_SECTION_END ();
      return 0; // OK
    }
  else
    {
      CRITICAL_SECTION_END ();
      return 1; // Busy
    }
}

uint8_t
UartMcuGetChar (Uart_t *obj, uint8_t *data)
{
  CRITICAL_SECTION_BEGIN ();
  if ((!rxOnGoing) && !rxBufferEmpty)
    {
      memcpy (data, g_rxBuffer, sizeof(data));
      rxBufferEmpty = true;
    }
  else
    {
      CRITICAL_SECTION_END ();
      return 1; // NOT OK
    }
  CRITICAL_SECTION_END ();

  CRITICAL_SECTION_BEGIN ();
  /* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
  if ((!rxOnGoing) && rxBufferEmpty)
    {
      rxOnGoing = true;
      UART_ReceiveEDMA (UartBaseAddresses[obj->UartId], &g_uartEdmaHandle,
			&receiveXfer);
    }
  CRITICAL_SECTION_END ();
  return 0; //OK
}

uint8_t
UartMcuPutBuffer (Uart_t *obj, uint8_t *buffer, uint16_t size)
{
  uint8_t retryCount;
  uint16_t i;

  for (i = 0; i < size; i++)
    {
      retryCount = 0;
      while (UartPutChar (obj, buffer[i]) != 0)
	{
	  retryCount++;

	  // Exit if something goes terribly wrong
	  if (retryCount > TX_BUFFER_RETRY_COUNT)
	    {
	      return 1; // Error
	    }
	}
    }
  return 0; // OK
}

uint8_t
UartMcuGetBuffer (Uart_t *obj, uint8_t *buffer, uint16_t size,
		  uint16_t *nbReadBytes)
{
  uint16_t localSize = 0;

  while (localSize < size)
    {
      if (UartGetChar (obj, buffer + localSize) == 0)
	{
	  localSize++;
	}
      else
	{
	  break;
	}
    }

  *nbReadBytes = localSize;

  if (localSize == 0)
    {
      return 1; // Empty
    }
  return 0; // OK
}

/* Configuration for the EDMA */
void
UartConfigureEdma ()
{
  edma_config_t config;

  /* Init DMAMUX */
  DMAMUX_Init (UART_DMAMUX_BASEADDR);
  /* Set channel for UART */
  DMAMUX_SetSource (UART_DMAMUX_BASEADDR, UART_TX_DMA_CHANNEL,
		    UART1_TX_DMA_REQUEST);
  DMAMUX_SetSource (UART_DMAMUX_BASEADDR, UART_RX_DMA_CHANNEL,
		    UART1_RX_DMA_REQUEST);
  DMAMUX_EnableChannel (UART_DMAMUX_BASEADDR, UART_TX_DMA_CHANNEL);
  DMAMUX_EnableChannel (UART_DMAMUX_BASEADDR, UART_RX_DMA_CHANNEL);
  /* Init the EDMA module */
  EDMA_GetDefaultConfig (&config);
  EDMA_Init (UART_DMA_BASEADDR, &config);
  EDMA_CreateHandle (&g_uartTxEdmaHandle, UART_DMA_BASEADDR,
		     UART_TX_DMA_CHANNEL);
  EDMA_CreateHandle (&g_uartRxEdmaHandle, UART_DMA_BASEADDR,
		     UART_RX_DMA_CHANNEL);

  /* Create UART DMA handle. */
  UART_TransferCreateHandleEDMA (UART, &g_uartEdmaHandle, UartUserCallback,
				 NULL, &g_uartTxEdmaHandle,
				 &g_uartRxEdmaHandle);
}

/* UART user callback */
void
UartUserCallback (UART_Type *base, uart_edma_handle_t *handle, status_t status,
		  void *userData)
{
  userData = userData;

  if (kStatus_UART_TxIdle == status)
    {
      txOnGoing = false;
    }

  if (kStatus_UART_RxIdle == status)
    {
      rxBufferEmpty = false;
      rxOnGoing = false;
    }
}

