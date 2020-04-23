/*!
 * \file      main.c
 *
 * \brief     Application for the development of the Tiny-Lacuna board layer.
 *            Mainly used for testing purposes of new functions.
 *
 * \author    Diego Bienz
 */

#include <stdio.h>
#include <string.h>
#include "board.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "gpio.h"

TimerEvent_t timer_event;

/*!
 * Example for a GPIO interrupt
 * Variable is set as soon as GPIO
 * interrupt happens on PTD_5
 */
extern uint8_t testIrq;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1;
extern Gpio_t Led2;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

/**
 * Main application entry point.
 */

/*!
 * Example timer callback
 */
void test_callback(void) {
	printf("Test callback");
}

/**
 * Main application entry point.
 */
int main(void) {
	// Target board initialization
	BoardInitMcu();
	BoardInitPeriph();

	// Radio initialization
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;

	Radio.Init(&RadioEvents);

	/*!
	 * Example of how to init the timer and start it.
	 * We don't use that here.
	 */
	//TimerInit(&timer_event, test_callback);
	//TimerStart(&timer_event);
	/* Busy delay */
	DelayMs(200);

	printf("\r\n### Start of development application ###\r\n");
	printf("BOARD: Tiny-Lacuna\r\n");
	printf("see: https://github.com/nortismo/LoRaMac-node\r\n\r\n");

	while (1) {
		/* If GPIO interrupt happens, print Juhuu. Else heartbeat. */
		if (testIrq == 1) {
			printf("Juhuuu\r\n");
			testIrq = 0;
		} else {
			printf("heartbeat!\r\n");
		}
		DelayMs(200);
	}
}

void OnTxDone(void) {
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
}

void OnTxTimeout(void) {
}

void OnRxTimeout(void) {
}

void OnRxError(void) {
}
