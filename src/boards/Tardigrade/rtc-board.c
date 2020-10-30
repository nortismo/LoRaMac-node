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
#include "fsl_ctimer.h"
#include "fsl_rtc.h"

#define CTIMER          			CTIMER0  /* Timer 0 */
#define CTIMER_CLK_FREQ 			CLOCK_GetCTimerClkFreq(0U)
#define CTIMER_MATCH_OUT	 		kCTIMER_Match_0

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

static ctimer_config_t config;
static ctimer_match_config_t matchConfig;

/*!
 * Callback for the ctimer match
 */
void CTIMER_Callback(uint32_t flags);

/* Array of function pointers for callback for each channel */
ctimer_callback_t ctimer_callback_table[] = {
		CTIMER_Callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

/**
 * TODO: Implement possibility to set time from GNSS
 */
void RtcInit(void) {

	CTIMER_GetDefaultConfig(&config);
	CTIMER_Init(CTIMER, &config);

	CTIMER_RegisterCallBack(CTIMER, &ctimer_callback_table[0], kCTIMER_SingleCallback);

	/* RTC initialization */
	RTC_Init(RTC);
	rtc_datetime_t date;

	date.year = 2020U;
	date.month = 10U;
	date.day = 7U;
	date.hour = 12U;
	date.minute = 0;
	date.second = 0;

	/* RTC time counter has to be stopped before setting the date & time in the TSR register */
	RTC_EnableTimer(RTC, false);

	/* Set RTC time to default */
	RTC_SetDatetime(RTC, &date);

	/* When working under Normal Mode, the interrupt is controlled by NVIC. */
	EnableIRQ(RTC_IRQn);

	/* Start the RTC time counter */
	RTC_EnableTimer(RTC, true);
}

uint32_t RtcGetMinimumTimeout(void) {
	return RtcMs2Tick(1);
}

uint32_t RtcMs2Tick(TimerTime_t milliseconds) {
	return MSEC_TO_COUNT(milliseconds, CTIMER_CLK_FREQ);
}

TimerTime_t RtcTick2Ms(uint32_t tick) {
	return COUNT_TO_MSEC(tick, CTIMER_CLK_FREQ);
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
	LpmSetOffMode(LPM_RTC_ID, LPM_ENABLE);
	CTIMER_StopTimer(CTIMER);
}

void RtcStartAlarm(uint32_t timeout) {

	CRITICAL_SECTION_BEGIN();
	RtcStopAlarm();
	LpmSetOffMode(LPM_RTC_ID, LPM_DISABLE);

    /* Configuration 0 */
	matchConfig.enableCounterReset = false;
	matchConfig.enableCounterStop  = false;
	matchConfig.matchValue         = timeout + RtcGetTimerValue();
	matchConfig.outControl         = kCTIMER_Output_Toggle;
	matchConfig.outPinInitState    = false;
	matchConfig.enableInterrupt    = true;

    CTIMER_SetupMatch(CTIMER, CTIMER_MATCH_OUT, &matchConfig);
    CTIMER_StartTimer(CTIMER);

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
	return CTIMER_GetTimerCountValue(CTIMER);
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
	/**
	  * Not used on this bpard (yet).
	  * This driver is working with interrupts
	  */
}

TimerTime_t RtcTempCompensation(TimerTime_t period, float temperature) {
	/**
	  * Not used on this bpard (yet).
	  */
	return period;
}

void CTIMER_Callback(uint32_t flags){
	RtcStopAlarm();
	TimerIrqHandler();
}
