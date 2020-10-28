/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */

#include <stdio.h>
#include "utilities.h"
#include "eeprom-board.h"
#include "fsl_iap.h"

flash_config_t flashInstance;

/*!
 * \brief Initializes the EEPROM emulation module.
 */
void EepromMcuInit( void ){

    if (FLASH_Init(&flashInstance) != kStatus_Success)
    {
        printf( "Flash wasn't properly initialized\r\n" );
        // Fatal error, endless loop.
        while ( 1 )
        {
        }
    }
}

void EepromMcuGetUuid( uint8_t *uuid ){
	FFR_GetUUID(&flashInstance, uuid);
}

uint8_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t status = FAIL;



    return status;
}

uint8_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t status = FAIL;



    return status;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
    assert_param( FAIL );
}

uint8_t EepromMcuGetDeviceAddr( void )
{
    assert_param( FAIL );
    return FAIL;
}
