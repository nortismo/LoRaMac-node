/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
 *
 * \author    Diego Bienz
 */

#include "utilities.h"

#include "uart.h"
#include "rtc-board.h"
#include "board-config.h"
#include "MK22F51212.h"
#include "board.h"
#include "clock_config.h"
#include "gpio.h"


/*!
 * LED GPIO pins objects
 */
//TODO: GPIO
Gpio_t Led1;
Gpio_t Led2;

/*
 * MCU objects
 */
Uart_t Uart0;

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * Used to measure and calibrate the system wake-up time from STOP mode
 */
static void CalibrateSystemWakeupTime( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * UART0 FIFO buffers size
 */
#define UART0_FIFO_TX_SIZE                                1024
#define UART0_FIFO_RX_SIZE                                1024

uint8_t Uart0TxBuffer[UART0_FIFO_TX_SIZE];
uint8_t Uart0RxBuffer[UART0_FIFO_RX_SIZE];

/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
static volatile bool SystemWakeupTimeCalibrated = false;

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnCalibrateSystemWakeupTimeTimerEvent( void* context )
{
    //TODO: Check function
    RtcSetMcuWakeUpTime( );
    SystemWakeupTimeCalibrated = true;
}

//TODO:
/* Initialize debug console. */
//static void BOARDInitDebugConsole(void)
//{
//    uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
//
//    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
//}


void BoardCriticalSectionBegin( uint32_t *mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    __set_PRIMASK( *mask );
}

void BoardInitPeriph( void )
{

}

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {
    	//TODO: Pin-muxing
        //BOARD_InitPins();

        //TODO: Debug-Console
        //BOARD_InitDebugConsole();

        // LEDs
        GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
        GpioWrite( &Led1, 0 );
        GpioWrite( &Led2, 0 );

        SystemClockConfig( );

    	//TODO: Debug-Console
        //FifoInit( &Uart0.FifoTx, Uart0TxBuffer, UART0_FIFO_TX_SIZE );
        //FifoInit( &Uart0.FifoRx, Uart0RxBuffer, UART0_FIFO_RX_SIZE );

        // Configure your terminal for 8 Bits data (7 data bit + 1 parity bit), no parity and no flow ctrl
        //UartInit( &Uart0, UART_1, UART_TX, UART_RX );
        //UartConfig( &Uart0, RX_TX, BOARD_DEBUG_UART_BAUDRATE, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

        RtcInit( );



        BoardUnusedIoInit( );
    }
    else
    {
        SystemClockReConfig( );
    }

    //TODO: Integration of SPI
    //SpiInit( &SX126x.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS );
    
    //TODO: Integration of SX1262
    //SX126xIoInit( );

    if( McuInitialized == false )
    {
        McuInitialized = true;
        //TODO: Integration of SX1262
        //SX126xIoDbgInit( );
        //SX126xIoTcxoInit( );
    }
}

void BoardResetMcu( void )
{
    CRITICAL_SECTION_BEGIN( );

    //Restart system
    //TODO: Integrate the K22 CMSIS and reset the system here.
    //NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
    //TODO: Integration of SPI
    //SpiDeInit( &SX1276.Spi );
    //TODO: Integration of SX1262
    //SX126xIoDeInit( );
}

uint32_t BoardGetRandomSeed( void )
{
    //TODO: Integration of random from SE050
    return 0;
}

void BoardGetUniqueId( uint8_t *id )
{
    //TODO: Integration of id from SE050
}

static void BoardUnusedIoInit( void )
{
    /*!
     * Nothing to initialize as of yet 
     */
    return;
}

void SystemClockConfig( void )
{
	//TODO: Initialisation of Oscillator, Systemclock and Peripheral Clock
	// I don't know how to do that right now!

	BOARD_InitBootClocks();
}

void CalibrateSystemWakeupTime( void )
{
    if( SystemWakeupTimeCalibrated == false )
    {
        TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
        TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
        TimerStart( &CalibrateSystemWakeupTimeTimer );
        while( SystemWakeupTimeCalibrated == false )
        {

        }
    }
}

void SystemClockReConfig( void )
{
    //TODO: Check what to do here and if needed
}

void SysTick_Handler( void )
{
	//TODO: HAL of K22 needs to be integrated.
    //HAL_IncTick( );
    //HAL_SYSTICK_IRQHandler( );
}


/*
 * Function to be used by stdout for printf etc
 */
int _write( int fd, const void *buf, size_t count )
{
    //TODO
	//while( UartPutBuffer( &Uart0, ( uint8_t* )buf, ( uint16_t )count ) != 0 ){ };
    return count;
}

/*
 * Function to be used by stdin for scanf etc
 */
int _read( int fd, const void *buf, size_t count )
{
    size_t bytesRead = 0;
    //TODO
    //while( UartGetBuffer( &Uart0, ( uint8_t* )buf, count, ( uint16_t* )&bytesRead ) != 0 ){ };
    // Echo back the character
    //while( UartPutBuffer( &Uart0, ( uint8_t* )buf, ( uint16_t )bytesRead ) != 0 ){ };
    return bytesRead;
}
