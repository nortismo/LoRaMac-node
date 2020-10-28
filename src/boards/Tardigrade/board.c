/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */

#include <stdio.h>
#include "LPC55S16.h"
#include "utilities.h"
#include "uart.h"
#include "board-config.h"
#include "board.h"
#include "sx126x-board.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "gpio.h"
#include "rtc-board.h"
#include "spi-board.h"
#include "delay.h"
#include "gps.h"
#include "i2c-board.h"
#include "eeprom-board.h"

/*!
 * Unique Devices IDs
 */
#define         ID1                                 ( 0x1FFF7590 )
#define         ID2                                 ( 0x1FFF7594 )
#define         ID3                                 ( 0x1FFF7594 )


/*!
 * Uart objects
 */
Uart_t Uart0;  // Board Uart
Uart_t Uart1;  // GPS

/*!
 * I2c objects
 */
I2c_t I2c0;  // Secure Element

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * \brief Initializes the EEPROM emulation driver to access the flash.
 *
 * \remark This function is defined in eeprom-board.c file
 */
void EepromMcuInit( void );

/*!
 * \brief Get the UUID from the eeprom driver.
 *
 * \remark This function is defined in eeprom-board.c file
 */
void EepromMcuGetUuid( uint8_t *uuid  );

void BoardCriticalSectionBegin( uint32_t *mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    __set_PRIMASK( *mask );
	__enable_irq();
}

void BoardInitPeriph( void )
{
	GpsInit();
}

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {
        BOARD_InitPins();
        SystemClockConfig( );

        // Configure your terminal for 8 Bits data (7 data bit + 1 parity bit), no parity and no flow ctrl
        UartInit( &Uart0, UART_1, NC, NC );
        UartConfig( &Uart0, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

        RtcInit( );

        BoardUnusedIoInit( );
    }
    else
    {
        SystemClockReConfig( );
    }
    //SPI for LoRa transceiver
    SpiInit( &SX126x.Spi, SPI_1, NC, NC, NC, NC );
    SX126xIoInit( );
    if (McuInitialized == false) {
    	McuInitialized = true;
    	SX126xIoDbgInit();
    	SX126xIoTcxoInit();
	}
    //I2C for Secure Element
    I2cInit(&I2c0, I2C_1, NC, NC);

    EepromMcuInit();
}

void BoardResetMcu( void )
{
    CRITICAL_SECTION_BEGIN( );

    //Restart system
    NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
	//SpiDeInit(&SX126x.Spi);
	//SX126xIoDeInit();
}

uint32_t BoardGetRandomSeed( void )
{
	uint32_t seed[4];
	uint8_t uuid[16];
	BoardGetUniqueId(uuid);

	seed[0] = 0;
	for(int i = 0; i < 4; i++){
		seed[0] += uuid[i] << 8*i;
	}
	seed[1] = 0;
	for(int i = 0; i < 4; i++){
		seed[1] += uuid[i+4] << 8*i;
	}
	seed[2] = 0;
	for(int i = 0; i < 4; i++){
		seed[2] += uuid[i+8] << 8*i;
	}
	seed[3] = 0;
	for(int i = 0; i < 4; i++){
		seed[3] += uuid[i+12] << 8*i;
	}

    return seed[0] ^ seed[1] ^ seed[2] ^ seed[3];
}

void BoardGetUniqueId( uint8_t *id )
{
	EepromMcuGetUuid(id);
}

/**
  * TODO: Implement method
  */
uint16_t BoardBatteryMeasureVoltage( void )
{
    return 0;
}

/**
  * TODO: Implement method
  */
uint32_t BoardGetBatteryVoltage( void )
{
    return 0;
}

/**
  * TODO: Implement method
  */
uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}

static void BoardUnusedIoInit( void )
{
    // Nothing to do
}

void SystemClockConfig( void )
{
    BOARD_InitBootClocks();
}

void SystemClockReConfig( void )
{
    BOARD_InitBootClocks();
}

void SysTick_Handler( void )
{
	// SysTick interrupt. Could be used to react on that.
}

/**
  * TODO: Implement following mock
  */
uint8_t GetBoardPowerSource( void )
{
    // if( UsbIsConnected == false )
    // {
    //     return BATTERY_POWER;
    // }
    // else
    // {
    //     return USB_POWER;
    // }
    return USB_POWER;
}

/**
  * \brief Enters Low Power Stop Mode
  *
  * \note ARM exits the function when waking up
  */
/**
  * TODO: Implement Lpm
  */
void LpmEnterStopMode( void)
{
    // CRITICAL_SECTION_BEGIN( );
    //
    // BoardDeInitMcu( );
    //
    // CRITICAL_SECTION_END( );
    //
    // Enter Stop Mode
}

/*!
 * \brief Exits Low Power Stop Mode
 */
/**
  * TODO: Implement Lpm
  */
void LpmExitStopMode( void )
{
    // Disable IRQ while the MCU is not running on HSI
    // CRITICAL_SECTION_BEGIN( );
    //
    // Initilizes the peripherals
    // BoardInitMcu( );
    //
    // CRITICAL_SECTION_END( );
}

/*!
 * \brief Enters Low Power Sleep Mode
 *
 * \note ARM exits the function when waking up
 */
/**
  * TODO: Implement Sleep
  */
void LpmEnterSleepMode( void)
{

}

/**
  * TODO: Implement Low Power Handler
  */
void BoardLowPowerHandler( void )
{

}

#if !defined ( __CC_ARM )

/*
 * Function to be used by stdout for printf etc
 */
int _write( int fd, const void *buf, size_t count )
{
    while( UartPutBuffer( &Uart0, ( uint8_t* )buf, ( uint16_t )count ) != 0 ){ };
    return count;
}

/*
 * Function to be used by stdin for scanf etc
 */
int _read( int fd, const void *buf, size_t count )
{
    size_t bytesRead = 0;
    while( UartGetBuffer( &Uart0, ( uint8_t* )buf, count, ( uint16_t* )&bytesRead ) != 0 ){ };
    // Echo back the character
    while( UartPutBuffer( &Uart0, ( uint8_t* )buf, ( uint16_t )bytesRead ) != 0 ){ };
    return bytesRead;
}

#else

#include <stdio.h>

// Keil compiler
int fputc( int c, FILE *stream )
{
    while( UartPutChar( &Uart0, ( uint8_t )c ) != 0 );
    return c;
}

int fgetc( FILE *stream )
{
    uint8_t c = 0;
    while( UartGetChar( &Uart0, &c ) != 0 );
    // Echo back the character
    while( UartPutChar( &Uart0, c ) != 0 );
    return ( int )c;
}

#endif

#ifdef USE_FULL_ASSERT

#include <stdio.h>

/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %lu\n", file, line) */

    printf( "Wrong parameters value: file %s on line %lu\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
