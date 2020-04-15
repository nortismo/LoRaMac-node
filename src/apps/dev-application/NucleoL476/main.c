/*!
 * \file      main.c
 *
 * \brief     Application for the development of the Tiny-Lacuna board layer.
 *            Mainly used for testing purposes of new functions.
 *
 * \author    Diego Bienz
 */

#include <string.h>
#include "board.h"
#include <stdio.h>

/**
 * Main application entry point.
 */
int
main (void)
{
  // Target board initialization
  BoardInitMcu ();
  BoardInitPeriph ();

  printf ("Test");

  while (1)
    {
    }
}
