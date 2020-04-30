/*!
 * \file      se050.c
 *
 * \brief     Secure Element driver for the SE050 from NXP
 *
 * \author    Diego Bienz
 */
#ifndef __SE050_H__
#define __SE050_H__

#include "secure-element.h"
#include <stdint.h>
#include "se050-board.h"


SecureElementStatus_t SecureElementInit(SecureElementNvmEvent seNvmCtxChanged) {
	return SecureElementMcuInit(seNvmCtxChanged);
}

SecureElementStatus_t SecureElementRestoreNvmCtx(void *seNvmCtx) {
	return SecureElementMcuRestoreNvmCtx(seNvmCtx);
}

void* SecureElementGetNvmCtx(size_t *seNvmCtxSize) {
	SecureElementMcuGetNvmCtx(seNvmCtxSize);
}

SecureElementStatus_t SecureElementSetKey(KeyIdentifier_t keyID, uint8_t *key) {
	return SecureElementMcuSetKey(keyID, key);
}

SecureElementStatus_t SecureElementComputeAesCmac(uint8_t *micBxBuffer,
		uint8_t *buffer, uint16_t size, KeyIdentifier_t keyID, uint32_t *cmac) {
	return SecureElementMcuComputeAesCmac(micBxBuffer, buffer, size, keyID,
			cmac);
}

SecureElementStatus_t SecureElementVerifyAesCmac(uint8_t *buffer, uint16_t size,
		uint32_t expectedCmac, KeyIdentifier_t keyID) {
	return SecureElementMcuVerifyAesCmac(buffer, size, expectedCmac, keyID);
}

SecureElementStatus_t SecureElementAesEncrypt(uint8_t *buffer, uint16_t size,
		KeyIdentifier_t keyID, uint8_t *encBuffer) {
	return SecureElementMcuAesEncrypt(buffer, size, keyID, encBuffer);
}

SecureElementStatus_t SecureElementDeriveAndStoreKey(Version_t version,
		uint8_t *input, KeyIdentifier_t rootKeyID, KeyIdentifier_t targetKeyID) {
	return SecureElementMcuDeriveAndStoreKey(version, input, rootKeyID,
			targetKeyID);
}

SecureElementStatus_t SecureElementRandomNumber(uint32_t *randomNum) {
	return SecureElementMcuRandomNumber(random);
}

SecureElementStatus_t SecureElementSetDevEui(uint8_t *devEui) {
	return SecureElementMcuSetDevEui(devEui);
}

uint8_t* SecureElementGetDevEui(void) {
	return SecureElementMcuGetDevEui();
}

SecureElementStatus_t SecureElementSetJoinEui(uint8_t *joinEui) {
	return SecureElementMcuSetJoinEui(joinEui);
}

uint8_t* SecureElementGetJoinEui(void) {
	return SecureElementMcuGetJoinEui();
}

#endif //  __SE050_H__
