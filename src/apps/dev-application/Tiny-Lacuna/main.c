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
#include "delay-board.h"
#include "rtc-board.h"
#include "timer.h"

TimerEvent_t timer_event;

void
test_callback (void)
{
  printf ("Test callback");
}

/**
 * Main application entry point.
 */
int
main (void)
{
  // Target board initialization
  BoardInitMcu ();
  BoardInitPeriph ();

  TimerInit (&timer_event, test_callback);
  TimerStart (&timer_event);

  DelayMs (10000);

  printf ("Test");

  while (1)
    {
    }
}
