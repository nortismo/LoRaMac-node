/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Diego Bienz
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME 5

    /*!
 * Board MCU pins definitions
 */

#define RADIOS_RESET PB_6

#define RADIOS_MOSI PB_15
#define RADIOS_MISO PB_14
#define RADIOS_SCLK PB_13

#define RADIO1_NSS PA_1
#define RADIO2_NSS PA_5
#define RADIO3_NSS PB_4
#define RADIO4_NSS PA_10
#define RADIO5_NSS PA_9
#define RADIO6_NSS PA_11
#define RADIO7_NSS PB_2
#define RADIO8_NSS PB_8

#define RADIO1_DIO_0 PC_1
#define RADIO2_DIO_0 PA_4
#define RADIO3_DIO_0 PB_0
#define RADIO4_DIO_0 PA_2
#define RADIO5_DIO_0 PA_12
#define RADIO6_DIO_0 PB_12
#define RADIO7_DIO_0 PA_8
#define RADIO8_DIO_0 PB_5

#define LED_1 PB_5
#define LED_2 PA_5
#define LED_3 PB_6
#define LED_4 PB_7

#define LED_GREEN LED_1
#define LED_RED1 LED_2
#define LED_BLUE LED_3
#define LED_RED2 LED_4

#define BTN_1 PB_2

#define OSC_LSE_IN PC_14
#define OSC_LSE_OUT PC_15

#define OSC_HSE_IN PH_0
#define OSC_HSE_OUT PH_1

#define SWCLK PA_14
#define SWDAT PA_13

#define I2C_SCL PB_8
#define I2C_SDA PB_9

#define UART_TX PA_2
#define UART_RX PA_3

// Debug pins definition.
#define RADIO_DBG_PIN_TX PB_13
#define RADIO_DBG_PIN_RX PB_14

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
