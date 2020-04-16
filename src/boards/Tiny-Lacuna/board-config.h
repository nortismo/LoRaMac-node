/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
 *
 * \author    Diego Bienz
 */

#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * The rtc instance used for board.
 */
#define BOARD_RTC_FUNC_BASEADDR RTC

/*!
 * Definition of the LEDs
 */
#define LED_1                                       PTD_6
#define LED_2                                       PTD_7

/*!
 * SPI definition
 */
#define RADIO_MOSI                                  PTD_2
#define RADIO_MISO                                  PTD_3
#define RADIO_SCLK                                  PTD_1
#define RADIO_NSS                                   PTD_4

/*!
 * DEBUG UART configuration
 */
#define DEBUG_UART_FAKE_PIN							NC
#define DEBUG_UART_BAUDRATE                 		115200
#define DEBUG_UART									UART1_BASE
#define DEBUG_UART_DMAMUX_BASEADDR					DMAMUX0
#define DEBUG_UART_DMA_BASEADDR 					DMA0
#define DEBUG_UART_TX_DMA_REQUEST 					kDmaRequestMux0UART1Tx
#define DEBUG_UART_RX_DMA_REQUEST 					kDmaRequestMux0UART1Rx

/*!
 * GPIO interrupt configuration
 */
#define BOARD_PORTA_IRQ_HANDLER PORTA_IRQHandler
#define BOARD_PORTB_IRQ_HANDLER PORTB_IRQHandler
#define BOARD_PORTC_IRQ_HANDLER PORTC_IRQHandler
#define BOARD_PORTD_IRQ_HANDLER PORTD_IRQHandler
#define BOARD_PORTE_IRQ_HANDLER PORTE_IRQHandler

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
