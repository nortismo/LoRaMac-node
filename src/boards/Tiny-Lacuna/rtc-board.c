/*!
 * \file      rtc-board.h
 *
 * \brief     Target board RTC timer and low power modes management
 *
 * \author    Diego Bienz
 */

#include "rtc-board.h"
#include "fsl_rtc.h"

#define MIN_ALARM_DELAY                             3  // TODO: Check this value


/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;
static volatile bool RtcTimeoutPendingInterrupt = false;
static volatile bool RtcTimeoutPendingPolling = false;

typedef enum AlarmStates_e
{
    ALARM_STOPPED = 0,
    ALARM_RUNNING = !ALARM_STOPPED
} AlarmStates_t;

/*!
 * RTC timer context
 */
typedef struct
{
    uint32_t Time;  // Reference time
    uint32_t Delay; // Reference Timeout duration
    AlarmStates_t AlarmState;
}RtcTimerContext_t;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

/*!
 * @brief Override the RTC IRQ handler.
 */
void RTC_IRQHandler(void)
{
    if (RTC_GetStatusFlags(RTC) & kRTC_AlarmFlag)
    {
        /* Clear alarm flag */
        RTC_ClearStatusFlags(RTC, kRTC_AlarmInterruptEnable);

        RtcTimerContext.AlarmState = ALARM_STOPPED;
        // Because of one shot the task will be removed after the callback
        RtcTimeoutPendingInterrupt = false;
    }
}

void RtcInit( void )
{
    if( RtcInitialized == false )
    {
     	rtc_config_t* rtc_config;
     	RTC_GetDefaultConfig (rtc_config);
    	RTC_Init(RTC, rtc_config);

    	/* Select RTC clock source */
    	RTC_SetClockSource(RTC);

        /* Enable at the NVIC */
        EnableIRQ(RTC_IRQn);
        RtcInitialized = true;
    }
}

uint32_t RtcGetMinimumTimeout( void )
{
    return( MIN_ALARM_DELAY );
}

/*!
 * Driver doesn't support MS nor ticks. The FSL RTC driver
 * operates with rtc_datetime_t.
 */
uint32_t RtcMs2Tick( TimerTime_t milliseconds )
{
    return 0;
}

/*!
 * Driver doesn't support MS nor ticks. The FSL RTC driver
 * operates with rtc_datetime_t.
 */
TimerTime_t RtcTick2Ms( uint32_t tick )
{
    return 0;
}

/*!
 * Driver doesn't support MS nor ticks. The FSL RTC driver
 * operates with rtc_datetime_t.
 */
void RtcDelayMs( TimerTime_t milliseconds )
{
	for(;;)
	{
		/*!
		 * Driver doesn't support MS nor ticks. The FSL RTC driver
		 * operates with rtc_datetime_t.
		 */
	}
}

void RtcSetMcuWakeUpTime( void ){
	//TODO: Implement

	/*!
	 * Not implemented yet.
	 */
}

int16_t RtcGetMcuWakeUpTime( void ){
	//TODO: Implement

	/*!
	 * Not implemented yet.
	 */
	return 0;
}

void RtcSetAlarm( uint32_t timeout ){
	RtcStartAlarm( timeout );
}

void RtcStopAlarm( void ){
	RtcTimerContext.AlarmState = ALARM_STOPPED;
    RTC_DisableInterrupts(RTC, kRTC_AlarmInterruptEnable);
}

void RtcStartAlarm( uint32_t timeout ){
	rtc_datetime_t date;

	CRITICAL_SECTION_BEGIN( );
    RtcStopAlarm( );

    RtcTimeoutPendingInterrupt = true;
    RtcTimeoutPendingPolling = false;

    RtcTimerContext.AlarmState = ALARM_RUNNING;

    RtcTimerContext.Delay = timeout;

    RTC_EnableInterrupts(RTC, kRTC_AlarmInterruptEnable);
    if ((timeout < 1U) || (timeout > 9U))
    {
    	printf("Invalid input format\r\n");
        return;
    }
    /* Get date time and add offset*/
    RTC_GetDatetime(RTC, &date);
    date.second += timeout;
    if (date.second > 59U)
    {
        date.second -= 60U;
        date.minute += 1U;
        if (date.minute > 59U)
        {
            date.minute -= 60U;
            date.hour += 1U;
            if (date.hour > 23U)
            {
                date.hour -= 24U;
                date.day += 1U;
            }
        }
    }

    /* Set the datetime for alarm */
    if (!RTC_SetAlarm(RTC, &date) == kStatus_Success)
    {
        printf("Failed to set alarm. Alarm time is not in the future\r\n");
        return;
    }
}


uint32_t RtcSetTimerContext( void )
{
    RtcTimerContext.Time = ( uint32_t )RTC->TSR;
    return ( uint32_t )RtcTimerContext.Time;
}

uint32_t RtcGetTimerContext( void )
{
	return RtcTimerContext.Time;
}

uint32_t RtcGetCalendarTime( uint16_t *milliseconds ){
	return ( uint32_t )RTC->TSR;
}

uint32_t RtcGetTimerValue( void ) {
	return RTC->TSR;
}

uint32_t RtcGetTimerElapsedTime( void ){
	return ( uint32_t)( RTC->TSR - RtcTimerContext.Time );
}

void RtcBkupWrite( uint32_t data0, uint32_t data1 ){
	//TODO: Implement

	/*!
	 * Not implemented yet.
	 */
}

void RtcBkupRead( uint32_t* data0, uint32_t* data1 )
{
	//TODO: Implement

	/*!
	 * Not implemented yet.
	 */
}

void RtcProcess( void )
{
    CRITICAL_SECTION_BEGIN( );

    if( (  RtcTimerContext.AlarmState == ALARM_RUNNING ) && ( RtcTimeoutPendingPolling == true ) )
    {
        if( RtcGetTimerElapsedTime( ) >= RtcTimerContext.Delay )
        {
            RtcTimerContext.AlarmState = ALARM_STOPPED;
            // Because of one shot the task will be removed after the callback
            RtcTimeoutPendingPolling = false;
        }
    }
    CRITICAL_SECTION_END( );
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
	/*!
	 * Not implemented yet.
	 */
	return period;
}

