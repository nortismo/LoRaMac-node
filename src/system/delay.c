/*!
 * \file      delay.c
 *
 * \brief     Delay implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */

/* Unable to use HAL delay in combination with FreeRTOS */
//#include "delay-board.h"
#include "delay.h"
/* Replaced wait method with another implementation, not depending on systick */
#include "McuWait.h"

void Delay(float s)
{
    DelayMs(s * 1000.0f);
}

void DelayMs(uint32_t ms)
{
    McuWait_Waitms(ms);
}
