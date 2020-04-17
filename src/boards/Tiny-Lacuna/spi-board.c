/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
 *
 * \author    Diego Bienz
 */

#include "board.h"
#include "board-config.h"
#include "spi-board.h"
#include "fsl_dspi.h"
#include "fsl_dspi_edma.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"

#define SPI_DEFAULT_MASTER_CLK_FREQ 			CLOCK_GetFreq(SYS_CLK)
#define TRANSFER_SIZE 							2

/*!
 * k22SpiHandle_t is the struct enabling the user to use two different SPIs of the K22.
 * The handle holds all the information to the SPI, the DMA and the buffers.
 * The configuratoin of the handles is static, see the configuration definitions below.
 */
typedef struct {
	SPI_Type *type;
	DMAMUX_Type *dmamuxType;
	DMA_Type *dmaType;
	uint32_t txDmaRequestSource;
	uint32_t txDmaRequestChannel;
	uint32_t rxDmaRequestSource;
	uint32_t rxDmaRequestChannel;
	uint32_t intermediaryDmaRequestChannel;
	dspi_master_config_t masterConfig;
	dspi_transfer_t masterXfer;
	dspi_master_edma_handle_t spiEdmaHandle;
	edma_handle_t spiEdmaMasterRxToRxDataHandle;
	edma_handle_t spiEdmaMasterTxToIntermediaryHandle;
	edma_handle_t spiEdmaMasterIntermediaryToTxHandle;
	volatile bool isTransferCompleted;
	uint8_t masterRxData[TRANSFER_SIZE];
	uint8_t masterTxData[TRANSFER_SIZE];
	uint32_t transferConfigFlags;
} k22SpiHandle_t;

/*!
 * Configurations for the different SPIs of the board
 */
/* Configuration for Spi0 */
static k22SpiHandle_t SpiHandle0 = { .type = RADIO_SPI_MASTER_BASEADDR,
		.dmamuxType =
		RADIO_SPI_MASTER_DMAMUX_BASEADDR, .dmaType =
		RADIO_SPI_MASTER_DMA_BASEADDR, .txDmaRequestSource =
		RADIO_SPI_MASTER_DMA_TX_REQUEST_SOURCE, .txDmaRequestChannel =
		RADIO_SPI_MASTER_DMA_TX_CHANNEL, .rxDmaRequestSource =
		RADIO_SPI_MASTER_DMA_RX_REQUEST_SOURCE, .rxDmaRequestChannel =
		RADIO_SPI_MASTER_DMA_RX_CHANNEL, .intermediaryDmaRequestChannel =
		RADIO_SPI_MASTER_DMA_INTERMEDIARY_CHANNEL, .isTransferCompleted = true,
		.masterConfig.whichCtar = RADIO_SPI_CONF_CTAR_SELECTION,
		.masterConfig.ctarConfig.baudRate = RADIO_SPI_CONF_TRANSFER_BAUDRATE,
		.masterConfig.ctarConfig.bitsPerFrame =
		RADIO_SPI_CONF_CTAR_BIT_PER_FRAME, .masterConfig.ctarConfig.cpol =
		RADIO_SPI_CONF_CTAR_CPOL, .masterConfig.ctarConfig.cpha =
		RADIO_SPI_CONF_CTAR_CPHA, .masterConfig.ctarConfig.direction =
		RADIO_SPI_CONF_CTAR_DIR,
		.masterConfig.ctarConfig.pcsToSckDelayInNanoSec = (1000000000U
				/ RADIO_SPI_CONF_TRANSFER_BAUDRATE),
		.masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = (1000000000U
				/ RADIO_SPI_CONF_TRANSFER_BAUDRATE),
		.masterConfig.ctarConfig.betweenTransferDelayInNanoSec = (1000000000U
				/ RADIO_SPI_CONF_TRANSFER_BAUDRATE), .masterConfig.whichPcs =
		RADIO_SPI_CONF_CS, .masterConfig.pcsActiveHighOrLow =
		RADIO_SPI_CONF_CS_LOW_HIGH_ACTIVE, .masterConfig.enableContinuousSCK =
		RADIO_SPI_CONF_CONT_SCK, .masterConfig.enableRxFifoOverWrite =
		RADIO_SPI_CONF_RX_FIFO_OVERWRITE,
		.masterConfig.enableModifiedTimingFormat =
		RADIO_SPI_CONF_MOD_TIMING_FORMAT, .masterConfig.samplePoint =
		RADIO_SPI_CONF_SAMPLE_POINT, .transferConfigFlags =
		RADIO_SPI_TRANSFER_CONFIG_FLAGS };

/* Configuration for Spi1. NOT USED YET  */
static k22SpiHandle_t SpiHandle1;

void MapSpiIdToHandle(SpiId_t spiId, k22SpiHandle_t **handle);

void SPI_MasterUserCallback(SPI_Type *base, dspi_master_edma_handle_t *handle,
		status_t status, void *userData);

/*!
 * CAUTION: Configurations like Pin-Muxing and Clocks need to be preconfigured in BOARD_InitPins.
 * This init function doesn't care about the defined pins
 */
void SpiInit(Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso,
		PinNames sclk, PinNames nss) {

	edma_config_t userConfig;
	k22SpiHandle_t *handle;
	MapSpiIdToHandle(spiId, &handle);

	/* DMA MUX init */
	DMAMUX_Init(handle->dmamuxType);
	DMAMUX_SetSource(handle->dmamuxType, handle->rxDmaRequestChannel,
			(uint8_t) handle->rxDmaRequestSource);
	DMAMUX_EnableChannel(handle->dmamuxType, handle->rxDmaRequestChannel);
	DMAMUX_SetSource(handle->dmamuxType, handle->txDmaRequestChannel,
			(uint8_t) handle->txDmaRequestSource);
	DMAMUX_EnableChannel(handle->dmamuxType, handle->txDmaRequestChannel);

	EDMA_GetDefaultConfig(&userConfig);
	EDMA_Init(handle->dmaType, &userConfig);

	DSPI_MasterInit(handle->type, &(handle->masterConfig),
	SPI_DEFAULT_MASTER_CLK_FREQ);

	/* Set up spi master */
	memset(&(handle->spiEdmaMasterRxToRxDataHandle), 0,
			sizeof(handle->spiEdmaMasterRxToRxDataHandle));

#if (!(defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET))
	memset(&(handle->spiEdmaMasterTxToIntermediaryHandle), 0,
			sizeof(handle->spiEdmaMasterTxToIntermediaryHandle));
#endif
	memset(&(handle->spiEdmaMasterIntermediaryToTxHandle), 0,
			sizeof(handle->spiEdmaMasterIntermediaryToTxHandle));

	volatile edma_handle_t *test;

	test = &(handle->spiEdmaMasterRxToRxDataHandle);
	test->tail = 8;

	EDMA_CreateHandle(&(handle->spiEdmaMasterRxToRxDataHandle), handle->dmaType,
			handle->rxDmaRequestChannel);
#if (!(defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET))
	EDMA_CreateHandle(&(handle->spiEdmaMasterTxToIntermediaryHandle),
			handle->dmaType, handle->intermediaryDmaRequestChannel);
#endif
	EDMA_CreateHandle(&(handle->spiEdmaMasterIntermediaryToTxHandle),
			handle->dmaType, handle->txDmaRequestChannel);
#if (defined(FSL_FEATURE_DSPI_HAS_GASKET) && FSL_FEATURE_DSPI_HAS_GASKET)
	DSPI_MasterTransferCreateHandleEDMA(handle->type, &(handle->spiEdmaHandle),
			SPI_MasterUserCallback,
			handle, &(handle->spiEdmaMasterRxToRxDataHandle), NULL,
			&(handle->spiEdmaMasterIntermediaryToTxHandle));
#else
	DSPI_MasterTransferCreateHandleEDMA(handle->type, &(handle->spiEdmaHandle),
			SPI_MasterUserCallback, handle,
			&(handle->spiEdmaMasterRxToRxDataHandle),
			&(handle->spiEdmaMasterTxToIntermediaryHandle),
			&(handle->spiEdmaMasterIntermediaryToTxHandle));
#endif
}

void SpiDeInit(Spi_t *obj) {
	k22SpiHandle_t *handle;
	MapSpiIdToHandle(obj->SpiId, &handle);

	EDMA_Deinit(handle->dmaType);
	DMAMUX_Deinit(handle->dmamuxType);
	DSPI_Deinit(handle->type);
}

uint16_t SpiInOut(Spi_t *obj, uint16_t outData) {
	k22SpiHandle_t *handle;
	MapSpiIdToHandle(obj->SpiId, &handle);

	handle->masterTxData[1] = (uint8_t) ((outData & 0xFF00) >> 8);
	handle->masterTxData[0] = (uint8_t) (outData & 0x00FF);

	/* Start master transfer, send data to slave */
	handle->isTransferCompleted = false;
	handle->masterXfer.txData = handle->masterTxData;
	handle->masterXfer.rxData = handle->masterRxData;
	handle->masterXfer.dataSize = TRANSFER_SIZE;
	handle->masterXfer.configFlags = handle->transferConfigFlags;
	if (kStatus_Success
			!= DSPI_MasterTransferEDMA(handle->type, &(handle->spiEdmaHandle),
					&(handle->masterXfer))) {
		printf("There is error when start DSPI_MasterTransferEDMA \r\n ");
	}

	/* Wait until transfer completed */
	while (!(handle->isTransferCompleted)) {
	}

	return (((uint16_t) handle->masterRxData[1] << 8) | handle->masterRxData[0]);
}

/*!
 * Since the LoRaMac-node stack's Spi enum (SpiId_t) goes from SPI_1 to SPI_2,
 * but the K22 SDK goes from SPI0 to SPI1, a short mapping is required. This
 * method set the pointer "handle" to the corresponding handle of the given
 * spiId.
 */
void MapSpiIdToHandle(SpiId_t spiId, k22SpiHandle_t **handle) {

	switch (spiId) {
	case SPI_1:
		*handle = &SpiHandle0;
		break;
	case SPI_2:
		*handle = &SpiHandle1;
		break;
	default:
		break;
	}
}

void SPI_MasterUserCallback(SPI_Type *base, dspi_master_edma_handle_t *handle,
		status_t status, void *userData) {
	k22SpiHandle_t *spiHandle = userData;

	spiHandle->isTransferCompleted = true;
}

