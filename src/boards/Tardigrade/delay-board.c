/*!
 * \file      delay-board.c
 *
 * \brief     Target board delay implementation
 * 
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */

#include "board.h"
#include "delay-board.h"
#include "clock_config.h"

void DelayMsMcu(uint32_t ms) {
	uint32_t ticks = 0UL;
	uint32_t count = ms;

	/* Make a 1 milliseconds counter. */
	ticks = SystemCoreClock / 1000UL;
	assert((ticks - 1UL) <= SysTick_LOAD_RELOAD_Msk);

	/* Enable the SysTick for counting. */
	SysTick->LOAD = (uint32_t) (ticks - 1UL);
	SysTick->VAL = 0UL;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	for (; count > 0U; count--) {
		/* Wait for the SysTick counter reach 0. */
		while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {
		}
	}

	/* Disable SysTick. */
	SysTick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
	SysTick->LOAD = 0UL;
	SysTick->VAL = 0UL;
}
