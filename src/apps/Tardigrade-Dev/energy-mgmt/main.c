/*!
 * \file      main.c
 *
 * \brief     This is a test application for power measurements on the Tardigrade
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */

#include <stdlib.h>
#include <stdio.h>
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "gps.h"
#include "lpm-board.h"
#include "radio.h"
#include "sx126x.h"
#include "timer.h"
#include "delay.h"


#define MILISECONDS_TO_WAIT_BEFORE_LOW_POWER_MODE       5000

/*!
 * Low Power Modes of the LPC55
 */
typedef enum
{
    SLEEP = 0,
    DEEP_SLEEP,
    POWER_DOWN,
    DEEP_POWER_DOWN
}LowPowerModes_t;

/*!
 * Strings for the Low Power Modes
 */
static const char* LowPowerModeStrings[] =
{
    "Sleep",
    "Deep Sleep",
    "Power Down", 
    "Deep Power Down",
};

/*!
 * Function to be executed before going into low power mode
 */
static void DoBeforeGoingToLowPower( void );
/*!
 * Selected Low Power Mode to be used
 */
static LowPowerModes_t selectedMode = DEEP_POWER_DOWN;
/*!
 * Timer used to initialize the configured mode MILISECONDS_TO_WAIT_BEFORE_LOW_POWER_MODE seconds after start up
 */
static TimerEvent_t ModeInitializingTimer;
/*!
 * Function executed on ModeInitializingTimer event
 */
static void OnModeInitializingTimerEvent( void* context );
/*!
 * Bool which is set to true as soon as the board should go to low power
 */
static bool GoingToLowPower = false;
/*!
 * Access to the externally defined radio
 */
extern SX126x_t SX126x;

/*!
 * Main application entry point.
 */
int main( void )
{
    /* Normal initialization of the board and the peripherals */ 
    BoardInitMcu( );
    BoardInitPeriph( );

    printf("\r\n\r\n#################################################\r\n");
    printf("### TARDIGRADE DEVELOPMENT APP: POWER MEASURE ###\r\n");
    printf("#################################################\r\n\r\n");

    printf("Selected Low Power Mode:\t%s\r\n", LowPowerModeStrings[selectedMode]);
    printf("Configures MS to wait:\t\t%i\r\n", MILISECONDS_TO_WAIT_BEFORE_LOW_POWER_MODE);
    printf("\r\n");

    uint32_t msCounter = 0;

    TimerInit( &ModeInitializingTimer, OnModeInitializingTimerEvent );
    TimerSetValue( &ModeInitializingTimer, MILISECONDS_TO_WAIT_BEFORE_LOW_POWER_MODE );
    TimerStart( &ModeInitializingTimer );

    while(true){
        if(GoingToLowPower){
            printf("\r\nGoing into Low Power Mode: %s\r\n...\r\n\r\n", LowPowerModeStrings[selectedMode]);
            switch(selectedMode){
                case SLEEP:
                	DoBeforeGoingToLowPower();
                    while(true){
                        LpmEnterSleepMode();
                    }
                    break;
                case DEEP_SLEEP:
                	DoBeforeGoingToLowPower();
                    while(true){
                        LpmEnterStopMode();
                    }
                    break;
                case POWER_DOWN:
                	DoBeforeGoingToLowPower();
                    while(true){
                        LpmEnterOffMode();
                    }
                    break;
                case DEEP_POWER_DOWN:
                	DoBeforeGoingToLowPower();
                    while(true){
                        BoardEnterDeepPowerDown();
                    }
                    break;
            }
        }
        else{
            if(msCounter % 1000 == 0){
                printf("Waiting for %li more seconds...\r\n", (MILISECONDS_TO_WAIT_BEFORE_LOW_POWER_MODE - msCounter) / 1000);

            }
            DelayMs(10);
            msCounter = msCounter + 10;
        }
    }
}

static void OnModeInitializingTimerEvent( void* context ){
    TimerStop( &ModeInitializingTimer );
    GoingToLowPower = true;
}

static void DoBeforeGoingToLowPower( void ){
	BoardDeInitMcu();
}
