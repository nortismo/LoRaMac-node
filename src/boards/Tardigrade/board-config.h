/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * General definitions
 */


/**
 * Usart definitions
 */
#define LPC_NUMBER_OF_USARTS                    2

#if(LPC_NUMBER_OF_USARTS > 0)
#define LPC_USART1_TYPE                         USART0
#define LPC_USART1_CLK_FRQ                      CLOCK_GetFlexCommClkFreq(0U)
#define LPC_USART1_IRQn                         FLEXCOMM0_IRQn
#define LPC_USART1_IRQ_HANDLER                  FLEXCOMM0_IRQHandler
#define LPC_USART1_IRQ_ENABLE                   kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable
#define LPC_USART1_IRQ_FLAGS                    kUSART_RxFifoNotEmptyFlag | kUSART_RxError
#endif
#if(LPC_NUMBER_OF_USARTS > 1)
#define LPC_USART2_TYPE							USART3
#define LPC_USART2_CLK_FRQ						CLOCK_GetFlexCommClkFreq(3U)
#define LPC_USART2_IRQn							FLEXCOMM3_IRQn
#define LPC_USART2_IRQ_HANDLER					FLEXCOMM3_IRQHandler
#define LPC_USART2_IRQ_ENABLE					kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable
#define LPC_USART2_IRQ_FLAGS					kUSART_RxFifoNotEmptyFlag | kUSART_RxError
#endif

/**
 * SPI definitions
 */
#define LPC_NUMBER_OF_SPI                       1

#if(LPC_NUMBER_OF_SPI > 0)
#define LPC_SPI1_TYPE                           SPI1
#define LPC_SPI1_CLK_FRQ                        CLOCK_GetFlexCommClkFreq(1U)
#define LPC_SPI1_CONFIG_POLARITY                kSPI_ClockPolarityActiveHigh
#define LPC_SPI1_CONFIG_PHASE                   kSPI_ClockPhaseFirstEdge
#define LPC_SPI1_CONFIG_DIRECTION               kSPI_MsbFirst
#define LPC_SPI1_CONFIG_BAUDRATE                500000U
#define LPC_SPI1_CONFIG_DATAWIDTH               kSPI_Data8Bits
#define LPC_SPI1_CONFIG_SS                      kSPI_Ssel0
#define LPC_SPI1_CONFIG_SPOL                    kSPI_SpolActiveAllLow
#endif
#if(LPC_NUMBER_OF_SPI > 1)
#define LPC_SPI2_TYPE                           SPI7
#define LPC_SPI2_CLK_FRQ                        CLOCK_GetFlexCommClkFreq(7U)
#define LPC_SPI2_CONFIG_POLARITY                kSPI_ClockPolarityActiveHigh
#define LPC_SPI2_CONFIG_PHASE                   kSPI_ClockPhaseFirstEdge
#define LPC_SPI2_CONFIG_DIRECTION               kSPI_MsbFirst
#define LPC_SPI2_CONFIG_BAUDRATE                500000U
#define LPC_SPI2_CONFIG_DATAWIDTH               kSPI_Data8Bits
#define LPC_SPI2_CONFIG_SS                      kSPI_Ssel0
#define LPC_SPI2_CONFIG_SPOL                    kSPI_SpolActiveAllLow

#endif


/**
 * Radio definitions
 */
#define RADIO_NSS_PIN							PIO0_1
#define RADIO_TCXO_WAKEUP_TIME                  5
#define RADIO_RESET_PIN							PIO0_25
#define RADIO_ANT_SWITCH_PIN					PIO0_6
#define RADIO_BUSY_PIN							PIO0_24
#define RADIO_DIO_1_PIN							PIO0_28

/**
 * GNSS definitions
 */
#define GNSS_UART_BAUDRATE						9600
#define GNSS_PPS_PIN							PIO0_0
#define GNSS_RESET_PIN							PIO0_4

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
