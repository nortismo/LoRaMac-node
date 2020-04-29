/*!
 * \file      gps-board.c
 *
 * \brief     Target board GPS driver implementation
 *
 * \author    Diego Bienz
 */
#include "board-config.h"
#include "board.h"
#include "gpio.h"
#include "gps.h"
#include "uart.h"
#include "gps-board.h"

#define PMTK_LOCUS_STARTLOG "$PMTK185,0*22\r\n" ///< Start logging data

/*!
 * \brief Buffer holding the  raw data received from the gps
 */
static uint8_t NmeaString[128];

/*!
 * \brief Maximum number of data byte that we will accept from the GPS
 */
static volatile uint8_t NmeaStringSize = 0;

extern Uart_t Uart0;

void GpsMcuOnPpsSignal(void *context) {
	/* Signal not used */
}

void GpsMcuInvertPpsTrigger(void) {

}

void GpsMcuInit(void) {

	char *cmd = PMTK_LOCUS_STARTLOG;

	UartInit(&Uart0, UART_1, GPS_UART_FAKE_PIN, GPS_UART_FAKE_PIN);
	UartConfig(&Uart0, RX_TX, GPS_UART_BAUDRATE, UART_8_BIT, UART_1_STOP_BIT,
			NO_PARITY, NO_FLOW_CTRL);

	UartPutBuffer(&Uart0, (uint8_t*) cmd, strlen(cmd));
}

void GpsMcuStart(void) {
	/* Signal not used
	 * GPS Receiver is always running
	 */
}

void GpsMcuStop(void) {
	/* Signal not used
	 * GPS Receiver is always running
	 */
}

void GpsMcuProcess(void) {
	uint8_t data;

	while (UartGetChar(&Uart0, &data) == 0 && data != '\n') {
		if ((data == '$') || (NmeaStringSize >= 127)) {
			NmeaStringSize = 0;
		}

		NmeaString[NmeaStringSize++] = (int8_t) data;
	}

	if (data == '\n') {
		NmeaString[NmeaStringSize++] = '\0';
		GpsParseGpsData((int8_t*) NmeaString, NmeaStringSize);
	}
}

void GpsMcuIrqNotify(UartNotifyId_t id) {
	/* No IRQ used */
}
