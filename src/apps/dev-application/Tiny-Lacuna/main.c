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
		DelayMs(1000);
		printf("heartbeat!\r\n");
	}
}
