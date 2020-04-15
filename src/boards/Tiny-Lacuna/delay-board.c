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

void
DelayMsMcu (uint32_t ms)
{
  sysTickCountDown = ms;
  while (sysTickCountDown != 0U)
    {
    }
}
