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
	i2c_master_config_t config;
	uint32_t defaultBaudRate;
	i2c_master_transfer_t masterXfer;
	I2cAddrSize i2cInternalAddrSize;
} k22I2cHandle_t;

/*!
 * Configurations for the different I2Cs of the board
 */
/* Configuration for I2C0 */
static k22I2cHandle_t I2cHandle0;

/* Configuration for I2C1. NOT USED YET */
static k22I2cHandle_t I2cHandle1 = { .type = SE_I2C_MASTER_BASEADDR,
		.defaultBaudRate =
		SE_I2C_BAUDRATE, .i2cInternalAddrSize =
		SE_I2C_INTERNAL_ADDRESS_SIZE };

void MapI2cIdToHandle(I2cId_t i2cId, k22I2cHandle_t **handle);

/*!
 * CAUTION: Configurations like Pin-Muxing and Clocks need to be preconfigured in BOARD_InitPins.
 * This init function doesn't care about the defined pins
 */
void I2cMcuInit(I2c_t *obj, I2cId_t i2cId, PinNames scl, PinNames sda) {
	k22I2cHandle_t *handle;

	obj->I2cId = i2cId;
	MapI2cIdToHandle(i2cId, &handle);

	I2C_MasterGetDefaultConfig(&(handle->config));
	handle->config.baudRate_Bps = handle->defaultBaudRate;

	I2C_MasterInit(handle->type, &(handle->config),
	I2C_DEFAULT_MASTER_CLK_FREQ);

	memset(&(handle->masterXfer), 0, sizeof(handle->masterXfer));
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

	I2C_MasterDeinit(handle->type);
}

void I2cSetAddrSize(I2c_t *obj, I2cAddrSize addrSize) {
	k22I2cHandle_t *handle;
	MapI2cIdToHandle(obj->I2cId, &handle);

	handle->i2cInternalAddrSize = addrSize;
}

/*!
 * CAUTION: Make sure interrupts are not disabled, since
 * this method makes use of EDMA with interrupts.
 */
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

	return I2C_MasterTransferBlocking(handle->type, &(handle->masterXfer));
}

/*!
 * CAUTION: Make sure interrupts are not disabled, since
 * this method makes use of EDMA with interrupts.
 */
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

	return I2C_MasterTransferBlocking(handle->type, &(handle->masterXfer));
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
