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
#include "aes.h"
#include "cmac.h"
#include <stdint.h>
#include "se050-board.h"
#include <ex_sss_boot.h>

#define KEY_SIZE 16
#define NUM_OF_KEYS 16

/*!
 * Identifier value pair type for Keys
 */
typedef struct sKey {
	/*
	 * Key identifier
	 */
	KeyIdentifier_t KeyID;
	/*
	 * Key value
	 */
	uint8_t KeyValue[KEY_SIZE];
} Key_t;
/*
 * Secure Element Non Volatile Context structure
 */
typedef struct sSecureElementNvCtx {
	/*
	 * DevEUI storage
	 */
	uint8_t DevEui[SE_EUI_SIZE];
	/*
	 * Join EUI storage
	 */
	uint8_t JoinEui[SE_EUI_SIZE];
	/*
	 * AES computation context variable
	 */
	aes_context AesContext;
	/*
	 * CMAC computation context variable
	 */
	AES_CMAC_CTX AesCmacCtx[1];
	/*
	 * Key List
	 */
	Key_t KeyList[NUM_OF_KEYS];
} SecureElementNvCtx_t;

/*
 * Module context
 */
static SecureElementNvCtx_t SeNvmCtx;

/*
 * Security Subsystem Context
 */
ex_sss_boot_ctx_t sss_ctx;
static sss_status_t Applet_Identify(sss_se05x_session_t *pSession);

SecureElementStatus_t SecureElementInit(SecureElementNvmEvent seNvmCtxChanged) {

	/*
	 uint8_t itr = 0;

	 // Initialize context with defined key identifiers
	 SeNvmCtx.KeyList[itr++].KeyID = APP_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = GEN_APP_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = NWK_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = J_S_INT_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = J_S_ENC_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = F_NWK_S_INT_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = S_NWK_S_INT_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = NWK_S_ENC_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = APP_S_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_ROOT_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_KE_KEY;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_KEY_0;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_APP_S_KEY_0;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_NWK_S_KEY_0;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_KEY_1;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_APP_S_KEY_1;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_NWK_S_KEY_1;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_KEY_2;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_APP_S_KEY_2;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_NWK_S_KEY_2;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_KEY_3;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_APP_S_KEY_3;
	 SeNvmCtx.KeyList[itr++].KeyID = MC_NWK_S_KEY_3;
	 SeNvmCtx.KeyList[itr].KeyID = SLOT_RAND_ZERO_KEY;
	 */

	sss_status_t status = kStatus_SSS_Fail;

	ex_sss_boot_open_host_session(&sss_ctx);

	sss_se05x_session_t *pSession = (sss_se05x_session_t*) &(sss_ctx.session);

	status = Applet_Identify(pSession);
	if (status != kStatus_SSS_Success) {
		/* handle error */
	}

	status = JCOP4_GetDataIdentify();
	if (status != kStatus_SSS_Success) {
		/* handle error */
	}

	status = kStatus_SSS_Success;
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

#define CHECK_FEATURE_PRESENT(AppletConfig, ITEM)               \
    if (((kSE05x_AppletConfig_##ITEM) ==                        \
            ((AppletConfig) & (kSE05x_AppletConfig_##ITEM)))) { \
        /* LOG_I("With    " #ITEM); */                          \
    }                                                           \
    else {                                                      \
        /* LOG_I("WithOut " #ITEM); */                          \
    }

static sss_status_t Applet_Identify(sss_se05x_session_t *pSession) {
	sss_status_t status = kStatus_SSS_Fail;
	smStatus_t sw_status;
	SE05x_Result_t result = kSE05x_Result_NA;
	uint8_t uid[255];
	size_t uidLen = sizeof(uid);
	uint8_t applet_version[7];
	size_t applet_versionLen = sizeof(applet_version);

	sw_status = Se05x_API_CheckObjectExists(&pSession->s_ctx,
			kSE05x_AppletResID_UNIQUE_ID, &result);
	if (SM_OK != sw_status) {
		/* Handle error */
	}

	sw_status = Se05x_API_ReadObject(&pSession->s_ctx,
			kSE05x_AppletResID_UNIQUE_ID, 0, (uint16_t) uidLen, uid, &uidLen);
	if (SM_OK != sw_status) {
		/* Handle error */
	}

	// VersionInfo is a 7 - byte value consisting of :
	// - 1 - byte Major applet version
	// - 1 - byte Minor applet version
	// - 1 - byte patch applet version
	// - 2 - byte AppletConfig, indicating the supported applet features
	// - 2-byte Secure Box version: major version (MSB) concatenated with minor version (LSB).

	sw_status = Se05x_API_GetVersion(&pSession->s_ctx, applet_version,
			&applet_versionLen);
	if (SM_OK != sw_status) {
		/* Handle error */
	}

	{
		U16 AppletConfig = applet_version[3] << 8 | applet_version[4];
		CHECK_FEATURE_PRESENT(AppletConfig, ECDAA);
		CHECK_FEATURE_PRESENT(AppletConfig, ECDSA_ECDH_ECDHE);
		CHECK_FEATURE_PRESENT(AppletConfig, EDDSA);
		CHECK_FEATURE_PRESENT(AppletConfig, DH_MONT);
		CHECK_FEATURE_PRESENT(AppletConfig, HMAC);
		CHECK_FEATURE_PRESENT(AppletConfig, RSA_PLAIN);
		CHECK_FEATURE_PRESENT(AppletConfig, RSA_CRT);
		CHECK_FEATURE_PRESENT(AppletConfig, AES);
		CHECK_FEATURE_PRESENT(AppletConfig, DES);
		CHECK_FEATURE_PRESENT(AppletConfig, PBKDF);
		CHECK_FEATURE_PRESENT(AppletConfig, TLS);
		CHECK_FEATURE_PRESENT(AppletConfig, MIFARE);
		CHECK_FEATURE_PRESENT(AppletConfig, I2CM);
	}

	status = kStatus_SSS_Success;
	return status;
}

#endif //  __SE050_H__
