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

TimerEvent_t timer_event;

/*!
 * Example for a GPIO interrupt
 * Variable is set as soon as GPIO
 * interrupt happens on PTD_5
 */
extern uint8_t testIrq;

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
