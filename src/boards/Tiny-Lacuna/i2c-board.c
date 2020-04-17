/*!
 * \file      i2c-board.c
 *
 * \brief     Target board I2C driver implementation
 *
 * \author    Diego Bienz
 */

#include "board-config.h"
#include "i2c-board.h"
#include "i2c.h"
#include "fsl_i2c.h"
#include "fsl_i2c_edma.h"
#include "fsl_dmamux.h"

/*!
 *  The value of the maximal timeout for I2C waiting loops
 */
#define TIMEOUT_MAX                                 0x8000
#define I2C_DEFAULT_DATA_LENGTH 					33U
#define I2C_DEFAULT_MASTER_CLK_FREQ 				CLOCK_GetFreq(I2C0_CLK_SRC)

/*!
 * k22I2cHandle_t is the struct enabling the user to use two different I2Cs of the K22.
 * The handle holds all the information to the I2C, the DMA and the buffers.
 * The configuration of the handles is static, see the configuration definitions below.
 */
typedef struct {
	I2C_Type *type;
	uint32_t baudRate;
	DMAMUX_Type *dmamuxType;
	DMA_Type *dmaType;
	uint32_t dmaRequestSource;
	uint32_t dmaRequestChannel;
	i2c_master_transfer_t masterXfer;
	i2c_master_edma_handle_t i2cEdmaHandle;
	edma_handle_t edmaHandle;
	volatile bool isTransferCompleted;
	volatile bool lastTransferSuccessful;
	I2cAddrSize i2cInternalAddrSize;
} k22I2cHandle_t;

/*!
 * Configurations for the different I2Cs of the board
 */
/* Configuration for I2C0 */
static k22I2cHandle_t I2cHandle0;

/* Configuration for I2C1. NOT USED YET */
static k22I2cHandle_t I2cHandle1 = { .type = SE_I2C_MASTER_BASEADDR, .baudRate =
SE_I2C_BAUDRATE, .dmamuxType = SE_I2C_MASTER_DMAMUX_BASEADDR, .dmaType =
SE_I2C_MASTER_DMA_BASEADDR, .dmaRequestSource =
SE_I2C_MASTER_DMA_REQUEST_SOURCE, .dmaRequestChannel =
SE_I2C_MASTER_DMA_CHANNEL, .isTransferCompleted = true,
		.lastTransferSuccessful = true, .i2cInternalAddrSize =
		SE_I2C_INTERNAL_ADDRESS_SIZE };

void MapI2cIdToHandle(I2cId_t i2cId, k22I2cHandle_t **handle);

static void I2C_MasterCallback(I2C_Type *base, i2c_master_edma_handle_t *handle,
		status_t status, void *userData);

/*!
 * CAUTION: Configurations like Pin-Muxing and Clocks need to be preconfigured in BOARD_InitPins.
 * This init function doesn't care about the defined pins
 */
void I2cMcuInit(I2c_t *obj, I2cId_t i2cId, PinNames scl, PinNames sda) {
	i2c_master_config_t i2cConfig;
	edma_config_t edmaConfig;
	k22I2cHandle_t *handle;

	obj->I2cId = i2cId;
	MapI2cIdToHandle(i2cId, &handle);

	DMAMUX_Init(handle->dmamuxType);
	EDMA_GetDefaultConfig(&edmaConfig);
	EDMA_Init(handle->dmaType, &edmaConfig);

	I2C_MasterGetDefaultConfig(&i2cConfig);
	i2cConfig.baudRate_Bps = handle->baudRate;

	I2C_MasterInit(handle->type, &i2cConfig,
	I2C_DEFAULT_MASTER_CLK_FREQ);

	memset(&(handle->i2cEdmaHandle), 0, sizeof(handle->i2cEdmaHandle));
	memset(&(handle->masterXfer), 0, sizeof(handle->masterXfer));

	DMAMUX_SetSource(handle->dmamuxType, handle->dmaRequestChannel,
			handle->dmaRequestSource);
	DMAMUX_EnableChannel(handle->dmamuxType, handle->dmaRequestChannel);

	EDMA_CreateHandle(&(handle->edmaHandle), handle->dmaType,
			handle->dmaRequestChannel);

	I2C_MasterCreateEDMAHandle(handle->type, &(handle->i2cEdmaHandle),
			I2C_MasterCallback, handle, &(handle->edmaHandle));
}

void I2cMcuFormat(I2c_t *obj, I2cMode mode, I2cDutyCycle dutyCycle,
bool I2cAckEnable, I2cAckAddrMode AckAddrMode, uint32_t I2cFrequency) {
	/*!
	 * This function doesn't do anything!
	 * Configurations like Pin-Muxing and Clocks need to be preconfigured in BOARD_InitPins.
	 * This init function doesn't care about the defined pins
	 */
}

void I2cMcuDeInit(I2c_t *obj) {
	k22I2cHandle_t *handle;
	MapI2cIdToHandle(obj->I2cId, &handle);

	EDMA_Deinit(handle->dmaType);
	DMAMUX_Deinit(handle->dmamuxType);
	I2C_MasterDeinit(handle->type);
}

void I2cSetAddrSize(I2c_t *obj, I2cAddrSize addrSize) {
	k22I2cHandle_t *handle;
	MapI2cIdToHandle(obj->I2cId, &handle);

	handle->i2cInternalAddrSize = addrSize;
}

uint8_t I2cMcuWriteBuffer(I2c_t *obj, uint8_t deviceAddr, uint16_t addr,
		uint8_t *buffer, uint16_t size) {

	k22I2cHandle_t *handle;
	MapI2cIdToHandle(obj->I2cId, &handle);

	handle->masterXfer.slaveAddress = deviceAddr;
	handle->masterXfer.direction = kI2C_Write;
	handle->masterXfer.subaddress = (uint32_t) addr;
	handle->masterXfer.subaddressSize = handle->i2cInternalAddrSize + 1;
	handle->masterXfer.data = buffer;
	handle->masterXfer.dataSize = size;
	handle->masterXfer.flags = kI2C_TransferDefaultFlag;

	handle->isTransferCompleted = false;

	I2C_MasterTransferEDMA(handle->type, &(handle->i2cEdmaHandle),
			&(handle->masterXfer));

	while (!handle->isTransferCompleted) {
	}

	return handle->lastTransferSuccessful;
}

uint8_t I2cMcuReadBuffer(I2c_t *obj, uint8_t deviceAddr, uint16_t addr,
		uint8_t *buffer, uint16_t size) {

	volatile status_t res;

	k22I2cHandle_t *handle;
	MapI2cIdToHandle(obj->I2cId, &handle);

	handle->masterXfer.slaveAddress = deviceAddr;
	handle->masterXfer.direction = kI2C_Read;
	handle->masterXfer.subaddress = (uint32_t) addr;
	handle->masterXfer.subaddressSize = handle->i2cInternalAddrSize + 1;
	handle->masterXfer.data = buffer;
	handle->masterXfer.dataSize = size;
	handle->masterXfer.flags = kI2C_TransferDefaultFlag;

	handle->isTransferCompleted = false;

	res = I2C_MasterTransferEDMA(handle->type, &(handle->i2cEdmaHandle),
			&(handle->masterXfer));

	uint16_t counter = 100;

	while (!handle->isTransferCompleted && counter > 99) {
		counter++;
	}

	return handle->lastTransferSuccessful;
}

uint8_t I2cMcuWaitStandbyState(I2c_t *obj, uint8_t deviceAddr) {
	return true;
}

/*!
 * Since the LoRaMac-node stack's I2C enum (I2cId_t) goes from I2C_1 to I2C_2,
 * but the K22 SDK goes from I2C0 to I2C1, a short mapping is required. This
 * method set the pointer "handle" to the corresponding handle of the given
 * i2cId.
 */
void MapI2cIdToHandle(I2cId_t i2cId, k22I2cHandle_t **handle) {

	switch (i2cId) {
	case I2C_1:
		*handle = &I2cHandle0;
		break;
	case I2C_2:
		*handle = &I2cHandle1;
		break;
	default:
		break;
	}
}

static void I2C_MasterCallback(I2C_Type *base, i2c_master_edma_handle_t *handle,
		status_t status, void *userData) {
	k22I2cHandle_t *i2cHandle = userData;

	i2cHandle->isTransferCompleted = true;

	/* Signal transfer success when received success status. */
	if (status == kStatus_Success) {
		i2cHandle->lastTransferSuccessful = true;
	} else {
		i2cHandle->lastTransferSuccessful = false;
	}
}
