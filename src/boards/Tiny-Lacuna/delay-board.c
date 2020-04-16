/*!
 * \file      delay-board.c
 *
 * \brief     Target board delay implementation
 *
 * \author    Diego Bienz
 */
#include "board.h"
#include "delay-board.h"

#define MAX_SYSTICK 4294967295

extern volatile uint32_t sysTickCountDown;

/*!
 * Method makes use of the external sysTickCountDown defined in board.c of Tiny-Lacuna.
 * As long as the sysTickCountDown is not 0, it will be decremented every millisecond.
 */
void DelayMsMcu(uint32_t ms) {
	sysTickCountDown = ms;
	while (sysTickCountDown != 0U) {
	}
}
