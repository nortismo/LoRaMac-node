/*!
 * \file      se050-board.c
 *
 * \brief     Target board SE050 driver implementation
 *
 * \author    Diego Bienz
 */

#include "board-config.h"
#include "se050-board.h"
#include "i2c.h"

SecureElementStatus_t SecureElementMcuInit(
		SecureElementNvmEvent seNvmCtxChanged) {
	return SECURE_ELEMENT_ERROR;
}

SecureElementStatus_t SecureElementMcuRestoreNvmCtx(void *seNvmCtx) {
	return SECURE_ELEMENT_ERROR;
}

void* SecureElementMcuGetNvmCtx(size_t *seNvmCtxSize) {

}

SecureElementStatus_t SecureElementMcuSetKey(KeyIdentifier_t keyID,
		uint8_t *key) {
	return SECURE_ELEMENT_ERROR;
}

SecureElementStatus_t SecureElementMcuComputeAesCmac(uint8_t *micBxBuffer,
		uint8_t *buffer, uint16_t size, KeyIdentifier_t keyID, uint32_t *cmac) {
	return SECURE_ELEMENT_ERROR;
}

SecureElementStatus_t SecureElementMcuVerifyAesCmac(uint8_t *buffer,
		uint16_t size, uint32_t expectedCmac, KeyIdentifier_t keyID) {
	return SECURE_ELEMENT_ERROR;
}

SecureElementStatus_t SecureElementMcuAesEncrypt(uint8_t *buffer, uint16_t size,
		KeyIdentifier_t keyID, uint8_t *encBuffer) {
	return SECURE_ELEMENT_ERROR;
}

SecureElementStatus_t SecureElementMcuDeriveAndStoreKey(Version_t version,
		uint8_t *input, KeyIdentifier_t rootKeyID, KeyIdentifier_t targetKeyID) {
	return SECURE_ELEMENT_ERROR;
}

SecureElementStatus_t SecureElementMcuRandomNumber(uint32_t *randomNum) {
	return SECURE_ELEMENT_ERROR;
}

SecureElementStatus_t SecureElementMcuSetDevEui(uint8_t *devEui) {
	return SECURE_ELEMENT_ERROR;
}

uint8_t* SecureElementMcuGetDevEui(void) {
	return -1;
}

SecureElementStatus_t SecureElementMcuSetJoinEui(uint8_t *joinEui) {
	return SECURE_ELEMENT_ERROR;
}

uint8_t* SecureElementMcuGetJoinEui(void) {
	return -1;
}
