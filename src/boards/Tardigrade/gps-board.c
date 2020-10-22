/*!
 * \file      gps-board.c
 *
 * \brief     Target board GPS driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */

#include "board-config.h"
#include "board.h"
#include "gpio.h"
#include "gps.h"
#include "uart.h"
#include "gps-board.h"
#include "string.h"
#include "delay.h"

/*!
 * Pin and Uart definition
 */
extern Uart_t Uart1;
static Gpio_t gpsResetPin;
static Gpio_t gpsPpsPin;

/*!
 * \brief Buffer holding the  raw data received from the gps
 */
static uint8_t NmeaString[128];

/*!
 * \brief Maximum number of data byte that we will accept from the GPS
 */
static volatile uint8_t NmeaStringSize = 0;

void GpsMcuOnPpsSignal(void *context) {
	bool parseData = false;

	GpsPpsHandler(&parseData);

	if (parseData == true) {
		/* If data should be parsed, enabled uart and wait for receving the gps info */
		UartInit(&Uart1, UART_2, NC, NC);
		UartConfig(&Uart1, RX_TX, GNSS_UART_BAUDRATE, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL);
	}
}

void GpsMcuInvertPpsTrigger(void) {
	/* Nothing to do. Not implemented */
}

/*!
 * CAUTION:
 * The pin configuration (muxing, clock, etc.) is made with the pin_mux.* of the board.
 * You can also use the pin configuration tool of NXP.
 * This init function doesn't care about the defined pins
 */
void GpsMcuInit(void) {
	NmeaStringSize = 0;
	Uart1.IrqNotify = GpsMcuIrqNotify;

	/* Reset GPS module */
	GpsMcuStop();
	DelayMs(10);
	GpsMcuStart();

	GpioInit(&gpsPpsPin, GNSS_PPS_PIN, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
	GpioSetInterrupt(&gpsPpsPin, IRQ_FALLING_EDGE, IRQ_VERY_LOW_PRIORITY, &GpsMcuOnPpsSignal);
}

void GpsMcuStart(void) {
	GpioInit(&gpsResetPin, GNSS_RESET_PIN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
}

void GpsMcuStop(void) {
	GpioInit(&gpsResetPin, GNSS_RESET_PIN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
}

void GpsMcuProcess(void) {
	/* The processing is based on the PPS interrupt */
}

void GpsMcuIrqNotify(UartNotifyId_t id) {
	uint8_t data;
	if (id == UART_NOTIFY_RX) {
		if (UartGetChar(&Uart1, &data) == 0) {
			if ((data == '$') || (NmeaStringSize >= 127)) {
				NmeaStringSize = 0;
			}

			NmeaString[NmeaStringSize++] = (int8_t) data;

			if (data == '\n') {
				NmeaString[NmeaStringSize++] = '\0';
				GpsParseGpsData((int8_t*) NmeaString, NmeaStringSize);
				UartDeInit(&Uart1);
			}
		}
	}
}
