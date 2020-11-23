/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */

#include "board.h"
#include "board-config.h"
#include "rtc-board.h"
#include "lpm-board.h"
#include "timer.h"
#include "gps.h"
#include "fsl_rtc.h"
#include "fsl_ostimer.h"
#include "fsl_power.h"

#define OSTIMER_REF					OSTIMER
#define OSTIMER_CLK_FREQ        	32768
/*!
 * RTC timer context
 */
typedef struct {
	uint32_t Time;  // Reference time
} RtcTimerContext_t;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

/*!
 * State if an alarm is pending
 */
static bool PendingAlarm = false;

/*!
 * Date from the last GPS update
 */
rtc_datetime_t lastGpsUpdate;

/*!
 * External Nmea GPS data, stored in gps.c
 */
extern NmeaGpsData_t NmeaGpsData;

/*!
 * Callback function for the OS Timer
 */
void RtcOSTimerCallback(void);

/*!
 * Compare two rtc_datetimes and return true if they are the same
 */
static bool RtcDatetimeEqual(rtc_datetime_t* one, rtc_datetime_t* two);

/*!
 * Check if the databuffer (either UtcTime or Date) contains valid data
 */
static bool IsGpsDateTimeValid(char* dataBuffer);

void RtcInit(void) {

	/* OS Timer initialization */
	OSTIMER_Init(OSTIMER_REF);

	/* RTC initialization */
	RTC_Init(RTC);

	lastGpsUpdate.year = 2020U;
	lastGpsUpdate.month = 10U;
	lastGpsUpdate.day = 7U;
	lastGpsUpdate.hour = 12U;
	lastGpsUpdate.minute = 0;
	lastGpsUpdate.second = 0;

	/* RTC time counter has to be stopped before setting the date & time in the TSR register */
	RTC_EnableTimer(RTC, false);

	/* Set RTC time to default */
	RTC_SetDatetime(RTC, &lastGpsUpdate);

	/* When working under Normal Mode, the interrupt is controlled by NVIC. */
	EnableIRQ(RTC_IRQn);

	/* Start the RTC time counter */
	RTC_EnableTimer(RTC, true);
}

uint32_t RtcGetMinimumTimeout(void) {
	return RtcMs2Tick(1);
}

uint32_t RtcMs2Tick(TimerTime_t milliseconds) {
	return MSEC_TO_COUNT(milliseconds, OSTIMER_CLK_FREQ);
}

TimerTime_t RtcTick2Ms(uint32_t tick) {
	return COUNT_TO_MSEC(tick, OSTIMER_CLK_FREQ);
}

void RtcDelayMs(TimerTime_t milliseconds) {
	uint64_t delayTicks = 0;
	uint64_t refTicks = RtcGetTimerValue();

	delayTicks = RtcMs2Tick(milliseconds);

	// Wait delay ms
	while (((RtcGetTimerValue() - refTicks)) < delayTicks) {
		__NOP();
	}
}

void RtcSetAlarm(uint32_t timeout) {
	RtcStartAlarm(timeout);
}

void RtcStopAlarm(void) {
	PendingAlarm = false;
}

void RtcStartAlarm(uint32_t timeout) {

	CRITICAL_SECTION_BEGIN();
	RtcStopAlarm();
	PendingAlarm = true;

    /* Set the match value with unit of ticks. */
    OSTIMER_SetMatchValue(OSTIMER_REF, timeout + RtcGetTimerValue(), RtcOSTimerCallback);

	CRITICAL_SECTION_END();
}

uint32_t RtcSetTimerContext(void) {
	RtcTimerContext.Time = RtcGetTimerValue();
	return RtcTimerContext.Time;
}

uint32_t RtcGetTimerContext(void) {
	return RtcTimerContext.Time;
}

uint32_t RtcGetCalendarTime(uint16_t *milliseconds) {
	*milliseconds = 0;
	uint32_t a, b;
    do
    {
        a = RTC->COUNT;
        b = RTC->COUNT;
    } while (a != b);

	return b;
}

uint32_t RtcGetTimerValue(void) {
	return OSTIMER_GetCurrentTimerValue(OSTIMER_REF);
}

uint32_t RtcGetTimerElapsedTime(void) {
	return (uint32_t)(RtcGetTimerValue() - RtcTimerContext.Time);
}

void RtcBkupWrite(uint32_t data0, uint32_t data1) {
	/**
	  * Not used on this bpard (yet).
	  */
}

void RtcBkupRead(uint32_t *data0, uint32_t *data1) {
	/**
	  * Not used on this bpard (yet).
	  */
}

void RtcProcess(void) {
	/* Update time from GPS if GPS has fix and the time is valid */
	if(GpsHasFix() && IsGpsDateTimeValid(NmeaGpsData.NmeaDate) && IsGpsDateTimeValid(NmeaGpsData.NmeaUtcTime)){
		rtc_datetime_t date;

		date.year = ((uint16_t) (NmeaGpsData.NmeaDate[4] - 48) * 10 + (uint16_t) (NmeaGpsData.NmeaDate[5] - 48)) + 2000;
		date.month = (uint16_t) (NmeaGpsData.NmeaDate[2] - 48) * 10 + (uint16_t) (NmeaGpsData.NmeaDate[3] - 48);
		date.day = (uint16_t) (NmeaGpsData.NmeaDate[0] - 48) * 10 + (uint16_t) (NmeaGpsData.NmeaDate[1] - 48);
		date.hour = (uint16_t) (NmeaGpsData.NmeaUtcTime[0] - 48) * 10 + (uint16_t) (NmeaGpsData.NmeaUtcTime[1]  - 48);
		date.minute =  (uint16_t) (NmeaGpsData.NmeaUtcTime[2]  - 48) * 10 + (uint16_t) (NmeaGpsData.NmeaUtcTime[3]  - 48);
		date.second = (uint16_t) (NmeaGpsData.NmeaUtcTime[4] - 48) * 10 + (uint16_t) (NmeaGpsData.NmeaUtcTime[5]  - 48);

		/* Only update if the last update is not the same */
		if(RtcDatetimeEqual(&date, &lastGpsUpdate) == false){
			lastGpsUpdate = date;

			RTC_EnableTimer(RTC, false);
			RTC_SetDatetime(RTC, &date);
			RTC_EnableTimer(RTC, true);
		}
	}
}

TimerTime_t RtcTempCompensation(TimerTime_t period, float temperature) {
	/**
	  * Not used on this bpard (yet).
	  */
	return period;
}

void RtcOSTimerCallback(void){
	if(PendingAlarm){
		RtcStopAlarm();
		TimerIrqHandler();
	}
}

static bool RtcDatetimeEqual(rtc_datetime_t* one, rtc_datetime_t* two){
	if(one->year != two->year){
		return false;
	}
	if(one->month != two->month){
		return false;
	}
	if(one->day != two->day){
		return false;
	}
	if(one->hour != two->hour){
		return false;
	}
	if(one->minute != two->minute){
		return false;
	}
	if(one->second != two->second){
		return false;
	}
	return true;
}

static bool IsGpsDateTimeValid(char* dataBuffer){
	for(int i = 0; i < 6; i++){
		if(dataBuffer[i] == 0){
			return false;
		}
	}
	return true;
}
