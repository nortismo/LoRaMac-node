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
#include "spi.h"
#include "delay.h"
#include "gps.h"
#include "i2c.h"
#include "lpm-board.h"
#include "eeprom-board.h"
#include "fsl_power.h"

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
/*!
 * \brief Indicates if an erasing operation is on going.
 *
 * \remark This function is defined in eeprom-board.c file
 *
 * \retval isEradingOnGoing Returns true is an erasing operation is on going.
 */
bool EepromMcuIsErasingOnGoing( void );

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
	BOARD_InitPins();
	SystemClockConfig( );

	// Configure your terminal for 8 Bits data (7 data bit + 1 parity bit), no parity and no flow ctrl
	UartInit( &Uart0, UART_1, NC, NC );
	UartConfig( &Uart0, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

	RtcInit( );

	BoardUnusedIoInit( );

    //SPI for LoRa transceiver
    SpiInit( &SX126x.Spi, SPI_1, NC, NC, NC, NC );
    SX126xIoInit( );
	SX126xIoDbgInit();
	SX126xIoTcxoInit();
	SX126xReset();

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
	GpsStop();
	if(Uart0.IsInitialized) {
		UartDeInit(&Uart0);
	}
	if(Uart1.IsInitialized) {
		UartDeInit(&Uart1);
	}
	SpiDeInit(&SX126x.Spi);
	I2cDeInit(&I2c0);
	SX126xIoDeInit();
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

/*!
 * \brief Enters the deepest power down mode
 * A reset is executed automatically on wake up
 *
 * IMPORTANT: SRAM retention defines which RAM sections should be retained while deep power down. It is important to place
 * variables in the retained sections if they are used after wake up from deep power down. One can place a variable
 * in a specified section as follows:
 *
 * bool __attribute__((section ("m_usb_bdt"))) myBool = false;
 *
 * While the defined section m_usb_bdt is defined in the linker script.
 */
void BoardEnterDeepPowerDown( void ){
	BoardDeInitMcu();
    POWER_EnterDeepPowerDown(BOARD_EXCLUDE_FROM_DEEP_POWERDOWN, BOARD_SRAM_RETENTION_DEEP_POWERDOWN, BOARD_WAKEUP_INTERRUPTS_DEEP_POWERDOWN, 0);
}

/*!
 * \brief Enters off Power Mode
 */
void LpmEnterOffMode( void ){
    POWER_EnterPowerDown(BOARD_EXCLUDE_FROM_POWERDOWN, BOARD_SRAM_RETENTION_POWERDOWN, BOARD_WAKEUP_INTERRUPTS_POWERDOWN, 1);
}

/*!
 * \brief Exits off Power Mode
 */
void LpmExitOffMode( void ){
	Uart0.IsInitialized = false;
	BoardInitMcu();
}

/**
  * \brief Enters Low Power Stop Mode
  *
  * \note ARM exits the function when waking up
  */
void LpmEnterStopMode( void)
{
    // Enter Deep Sleep Mode
    POWER_EnterDeepSleep(BOARD_EXCLUDE_FROM_DEEPSLEEP, BOARD_SRAM_RETENTION_DEEPSLEEP, BOARD_WAKEUP_INTERRUPTS_DEEPSLEEP, 0);
}

/*!
 * \brief Exits Low Power Stop Mode
 */
void LpmExitStopMode( void )
{
    CRITICAL_SECTION_BEGIN( );
    SystemClockReConfig();
    CRITICAL_SECTION_END( );
}

/*!
 * \brief Enters Low Power Sleep Mode
 *
 * \note ARM exits the function when waking up
 */
void LpmEnterSleepMode( void)
{
	POWER_EnterSleep();
}

void BoardLowPowerHandler( void )
{
    // Wait for any cleanup to complete before entering standby/shutdown mode
    while( EepromMcuIsErasingOnGoing( ) == true ){ }

    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending
     * and cortex will not enter low power anyway
     */

    LpmEnterLowPower( );

    __enable_irq( );
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
