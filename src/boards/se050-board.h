/*!
 * \file      se050-board.h
 *
 * \brief     Target board SE050 driver API
 *
 *
 * \author    Diego Bienz
 */
#ifndef _SE050_BOARD_H_
#define _SE050_BOARD_H_

#include "secure-element.h"

/*!
 * Signature of callback function to be called by the Secure Element driver when the
 * non volatile context have to be stored.
 *
 */
typedef void (*SecureElementNvmEvent)(void);

/*!
 * Initialization of Secure Element driver
 *
 * \param[IN]     seNvmCtxChanged           - Callback function which will be called  when the
 *                                            non-volatile context have to be stored.
 * \retval                                  - Status of the operation
 */
SecureElementStatus_t SecureElementMcuInit(
		SecureElementNvmEvent seNvmCtxChanged);

/*!
 * Restores the internal nvm context from passed pointer.
 *
 * \param[IN]     seNvmCtx         - Pointer to non-volatile module context to be restored.
 * \retval                         - Status of the operation
 */
SecureElementStatus_t SecureElementMcuRestoreNvmCtx(void *seNvmCtx);

/*!
 * Returns a pointer to the internal non-volatile context.
 *
 * \param[IN]     seNvmCtxSize    - Size of the module non volatile context
 * \retval                        - Points to a structure where the module store its non volatile context
 */
void* SecureElementMcuGetNvmCtx(size_t *seNvmCtxSize);

/*!
 * Sets a key
 *
 * \param[IN]  keyID          - Key identifier
 * \param[IN]  key            - Key value
 * \retval                    - Status of the operation
 */
SecureElementStatus_t SecureElementMcuSetKey(KeyIdentifier_t keyID,
		uint8_t *key);

/*!
 * Computes a CMAC of a message using provided initial Bx block
 *
 * \param[IN]  micBxBuffer    - Buffer containing the initial Bx block
 * \param[IN]  buffer         - Data buffer
 * \param[IN]  size           - Data buffer size
 * \param[IN]  keyID          - Key identifier to determine the AES key to be used
 * \param[OUT] cmac           - Computed cmac
 * \retval                    - Status of the operation
 */
SecureElementStatus_t SecureElementMcuComputeAesCmac(uint8_t *micBxBuffer,
		uint8_t *buffer, uint16_t size, KeyIdentifier_t keyID, uint32_t *cmac);

/*!
 * Verifies a CMAC (computes and compare with expected cmac)
 *
 * \param[IN]  buffer         - Data buffer
 * \param[IN]  size           - Data buffer size
 * \param[in]  expectedCmac   - Expected cmac
 * \param[IN]  keyID          - Key identifier to determine the AES key to be used
 * \retval                    - Status of the operation
 */
SecureElementStatus_t SecureElementMcuVerifyAesCmac(uint8_t *buffer,
		uint16_t size, uint32_t expectedCmac, KeyIdentifier_t keyID);

/*!
 * Encrypt a buffer
 *
 * \param[IN]  buffer         - Data buffer
 * \param[IN]  size           - Data buffer size
 * \param[IN]  keyID          - Key identifier to determine the AES key to be used
 * \param[OUT] encBuffer      - Encrypted buffer
 * \retval                    - Status of the operation
 */
SecureElementStatus_t SecureElementMcuAesEncrypt(uint8_t *buffer, uint16_t size,
		KeyIdentifier_t keyID, uint8_t *encBuffer);

/*!
 * Derives and store a key
 *
 * \param[IN]  version        - LoRaWAN specification version currently in use.
 * \param[IN]  input          - Input data from which the key is derived ( 16 byte )
 * \param[IN]  rootKeyID      - Key identifier of the root key to use to perform the derivation
 * \param[IN]  targetKeyID    - Key identifier of the key which will be derived
 * \retval                    - Status of the operation
 */
SecureElementStatus_t SecureElementMcuDeriveAndStoreKey(Version_t version,
		uint8_t *input, KeyIdentifier_t rootKeyID, KeyIdentifier_t targetKeyID);

/*!
 * Generates a random number
 *
 * \param[OUT] randomNum      - 32 bit random number
 * \retval                    - Status of the operation
 */
SecureElementStatus_t SecureElementMcuRandomNumber(uint32_t *randomNum);

/*!
 * Sets the DevEUI
 *
 * \param[IN] devEui          - Pointer to the 16-byte devEUI
 * \retval                    - Status of the operation
 */
SecureElementStatus_t SecureElementMcuSetDevEui(uint8_t *devEui);

/*!
 * Gets the DevEUI
 *
 * \retval                    - Pointer to the 16-byte devEUI
 */
uint8_t* SecureElementMcuGetDevEui(void);

/*!
 * Sets the JoinEUI
 *
 * \param[IN] joinEui         - Pointer to the 16-byte joinEui
 * \retval                    - Status of the operation
 */
SecureElementStatus_t SecureElementMcuSetJoinEui(uint8_t *joinEui);

/*!
 * Gets the DevEUI
 *
 * \retval                    - Pointer to the 16-byte joinEui
 */
uint8_t* SecureElementMcuGetJoinEui(void);

#endif /* _SE050_BOARD_H_ */
