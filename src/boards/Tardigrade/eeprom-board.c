/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */

#include "utilities.h"
#include "eeprom-board.h"
#include "fsl_iap.h"
#include "fsl_iap_ffr.h"

flash_config_t flashInstance;
bool ErasingOnGoing = false;

/************************************************
*
* Total Flash Size: 256 KB (244 KB available for customer)
* Sector Size: 32 KB
* Page Size: 512 Bytes
*
************************************************/

/*!
 * \brief Initializes the EEPROM emulation module.
 */
void EepromMcuInit( void ){

    if (FLASH_Init(&flashInstance) != kStatus_Success)
    {
        // Fatal error, endless loop.
        while ( 1 )
        {
        }
    }
}

void EepromMcuGetUuid( uint8_t *uuid ){
	FFR_GetUUID(&flashInstance, uuid);
}

/*!
 * \brief Indicates if an erasing operation is on going.
 *
 * \retval isEradingOnGoing Returns true is an erasing operation is on going.
 */
bool EepromMcuIsErasingOnGoing( void )
{
    return ErasingOnGoing;
}

/*!
 * CAUTION: The address and size has to be a multiple of the page size (512 Bytes)
 */
uint8_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	uint32_t status = FAIL;
	uint32_t failedAddress, failedData;

	if(size % 512 != 0 || addr % 512 != 0){
		return FAIL;
	}

	ErasingOnGoing = true;
	status = FLASH_Erase(&flashInstance, (uint32_t) addr, (uint32_t) size, kFLASH_ApiEraseKey);
	ErasingOnGoing = false;

    if(status != kStatus_Success){
    	return FAIL;
    }

    status = FLASH_VerifyErase(&flashInstance, (uint32_t) addr, (uint32_t) size);

    if(status != kStatus_Success){
    	return FAIL;
    }

    status = FLASH_Program(&flashInstance, (uint32_t) addr, buffer, (uint32_t) size);

    if(status != kStatus_Success){
    	return FAIL;
    }

    status = FLASH_VerifyProgram(&flashInstance, (uint32_t) addr, (uint32_t) size, buffer, &failedAddress,
                                     &failedData);

    if(status != kStatus_Success){
    	return FAIL;
    }

    return SUCCESS;
}

/*!
 * CAUTION: The address and size has to be a multiple of the page size (512 Bytes)
 */
uint8_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
	uint32_t status = FAIL;

	if(size % 512 != 0 || addr % 512 != 0){
		return FAIL;
	}

	status = FLASH_Read(&flashInstance, (uint32_t) addr, buffer, (uint32_t) size);

    if(status != kStatus_Success){
    	return FAIL;
    }

    return SUCCESS;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
	for(;;){
		/* This method is not allowed on this platform. */
	}
}

uint8_t EepromMcuGetDeviceAddr( void )
{
    return FAIL;
}
