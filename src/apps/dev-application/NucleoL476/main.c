/*!
 * \file      main.c
 *
 * \brief     Application for the development of the Tiny-Lacuna board layer.
 *            Mainly used for testing purposes of new functions.
 *
 * \author    Diego Bienz
 */

#include <stdio.h>
#include "board.h"
#include "delay.h"

/**
 * Main application entry point.
 */
int
main (void)
{
  // Target board initialization
  BoardInitMcu ();
  BoardInitPeriph ();

  DelayMs (2000);

  printf("Test\r\n");

  while (1)
    {
      printf("Test\r\n");
      DelayMs (1000);
    }
}



