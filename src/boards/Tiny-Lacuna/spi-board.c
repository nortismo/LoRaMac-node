/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
 *
 * \author    Diego Bienz
 */

#include <stdio.h>
#include "board.h"
#include "board-config.h"
#include "spi-board.h"
#include "fsl_dspi.h"

#define SPI_DEFAULT_MASTER_CLK_FREQ 			CLOCK_GetFreq(DSPI0_CLK_SRC)
/* Having a TRANSFER_SIZE of one byte is on purpose, since the communication of multiple
 * bytes is handles in the upper layer */
#define TRANSFER_SIZE 							1

/*!
 * k22SpiHandle_t is the struct enabling the user to use two different SPIs of the K22.
 * The handle holds all the information to the SPI, the DMA and the buffers.
 * The configuratoin of the handles is static, see the configuration definitions below.
 */
typedef struct {
	SPI_Type *type;
	dspi_master_config_t masterConfig;
	dspi_transfer_t masterXfer;
	uint8_t masterRxData[TRANSFER_SIZE];
	uint8_t masterTxData[TRANSFER_SIZE];
	uint32_t transferConfigFlags;
} k22SpiHandle_t;

/*!
 * Configurations for the different SPIs of the board
 */
/* Configuration for Spi0 */
static k22SpiHandle_t SpiHandle0 = { .type = RADIO_SPI_MASTER_BASEADDR,
		.masterConfig.whichCtar = RADIO_SPI_CONF_CTAR_SELECTION,
		.masterConfig.ctarConfig.baudRate = RADIO_SPI_CONF_TRANSFER_BAUDRATE,
		.masterConfig.ctarConfig.bitsPerFrame =
		RADIO_SPI_CONF_CTAR_BIT_PER_FRAME, .masterConfig.ctarConfig.cpol =
		RADIO_SPI_CONF_CTAR_CPOL, .masterConfig.ctarConfig.cpha =
		RADIO_SPI_CONF_CTAR_CPHA, .masterConfig.ctarConfig.direction =
		RADIO_SPI_CONF_CTAR_DIR,
		.masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 64,
		.masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 64,
		.masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 256,
		.masterConfig.enableContinuousSCK =
		RADIO_SPI_CONF_CONT_SCK, .masterConfig.enableRxFifoOverWrite =
		RADIO_SPI_CONF_RX_FIFO_OVERWRITE,
		.masterConfig.enableModifiedTimingFormat =
		RADIO_SPI_CONF_MOD_TIMING_FORMAT, .masterConfig.samplePoint =
		RADIO_SPI_CONF_SAMPLE_POINT, .transferConfigFlags =
		RADIO_SPI_TRANSFER_CONFIG_FLAGS };

/* Configuration for Spi1. NOT USED YET  */
static k22SpiHandle_t SpiHandle1;

void MapSpiIdToHandle(SpiId_t spiId, k22SpiHandle_t **handle);

/*!
 * CAUTION: Configurations like Pin-Muxing and Clocks need to be preconfigured in BOARD_InitPins.
 * This init function doesn't care about the defined pins
 */
void SpiInit(Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso,
		PinNames sclk, PinNames nss) {

	k22SpiHandle_t *handle;
	MapSpiIdToHandle(spiId, &handle);
	DSPI_MasterInit(handle->type, &(handle->masterConfig),
	SPI_DEFAULT_MASTER_CLK_FREQ);
}

void SpiDeInit(Spi_t *obj) {
	k22SpiHandle_t *handle;
	MapSpiIdToHandle(obj->SpiId, &handle);

	DSPI_Deinit(handle->type);
}

/*!
 * CAUTION: Make sure interrupts are not disabled, since
 * this method makes use of EDMA with interrupts.
 */
uint16_t SpiInOut(Spi_t *obj, uint16_t outData) {

	k22SpiHandle_t *handle;
	MapSpiIdToHandle(obj->SpiId, &handle);

	handle->masterRxData[0] = 0x00;
	handle->masterTxData[0] = (uint8_t) (outData);

	/* Start master transfer, send data to slave */
	handle->masterXfer.txData = handle->masterTxData;
	handle->masterXfer.rxData = handle->masterRxData;
	handle->masterXfer.dataSize = TRANSFER_SIZE;
	handle->masterXfer.configFlags = handle->transferConfigFlags;
	if (kStatus_Success
			!= (DSPI_MasterTransferBlocking(handle->type, &(handle->masterXfer)))) {
		printf("There is error when start DSPI_MasterTransferEDMA \r\n ");
	}

	return handle->masterRxData[0];
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
