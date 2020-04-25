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
 * TCXO configuration
 */
#define TCXO_WAKEUP_TIME                     		5
#define TCXO_VOLTAGE_IN						 		TCXO_CTRL_1_7V

/*!
 * Pin configuration:
 * The pin configuration is made with the pin_mux.* of the board.
 * You can also use the pin configuration tool of NXP.
 */

/*!
 * Definition of the LEDs
 */
#define LED_1                                       PTD_6
#define LED_2                                       PTD_7

/*!
 * RADIO SPI configuration
 * CAUTION: SPI and UART share the same DMA controller. The developer
 * is responsible to make sure that the different handles uses
 * different DMA channels for their signals.
 */
#define RADIO_SPI_MASTER_BASEADDR 					SPI0
#define RADIO_SPI_MASTER_DMAMUX_BASEADDR 			DMAMUX
#define RADIO_SPI_MASTER_DMA_BASEADDR 				DMA0
#define RADIO_SPI_MASTER_DMA_RX_REQUEST_SOURCE 		kDmaRequestMux0SPI0Rx
#define RADIO_SPI_MASTER_DMA_TX_REQUEST_SOURCE 		kDmaRequestMux0SPI0Tx
#define RADIO_SPI_MASTER_DMA_TX_CHANNEL 			3U
#define RADIO_SPI_MASTER_DMA_RX_CHANNEL 			2U
#define RADIO_SPI_MASTER_DMA_INTERMEDIARY_CHANNEL	4U
#define RADIO_SPI_CONF_TRANSFER_BAUDRATE			500000U
#define RADIO_SPI_CONF_CTAR_SELECTION			    kDSPI_Ctar1
#define RADIO_SPI_CONF_CTAR_BIT_PER_FRAME			8
#define RADIO_SPI_CONF_CTAR_CPOL					kDSPI_ClockPolarityActiveHigh
#define RADIO_SPI_CONF_CTAR_CPHA					kDSPI_ClockPhaseFirstEdge
#define RADIO_SPI_CONF_CTAR_DIR						kDSPI_MsbFirst
#define RADIO_SPI_CONF_CS							NULL
#define RADIO_SPI_CONF_CS_LOW_HIGH_ACTIVE			kDSPI_PcsActiveLow
#define RADIO_SPI_CONF_SAMPLE_POINT					kDSPI_SckToSin0Clock
#define RADIO_SPI_CONF_CONT_SCK						false
#define RADIO_SPI_CONF_RX_FIFO_OVERWRITE			false
#define RADIO_SPI_CONF_MOD_TIMING_FORMAT			false
#define RADIO_SPI_TRANSFER_CONFIG_FLAGS				(kDSPI_MasterCtar1 | kDSPI_MasterPcs1 | kDSPI_MasterPcsContinuous)
#define RADIO_SPI_FAKE_PIN							NC
#define RADIO_NSS									PTD_4
#define RADIO_BUSY									PTA_13
#define RADIO_DIO_1									PTA_12
#define RADIO_RESET									PTA_5
#define RADIO_ANT_SWITCH_POWER						PTB_18

/*!
 * DEBUG UART configuration
 * CAUTION: SPI and UART share the same DMA controller. The developer
 * is responsible to make sure that the different handles uses
 * different DMA channels for their signals.
 */
#define DEBUG_UART_FAKE_PIN							NC
#define DEBUG_UART_BAUDRATE                 		115200
#define DEBUG_UART									UART1_BASE
#define DEBUG_UART_DMAMUX_BASEADDR					DMAMUX0
#define DEBUG_UART_DMA_BASEADDR 					DMA0
#define DEBUG_UART_TX_DMA_REQUEST 					kDmaRequestMux0UART1Tx
#define DEBUG_UART_RX_DMA_REQUEST 					kDmaRequestMux0UART1Rx
#define DEBUG_UART_TX_DMA_DEFAULT_CHANNEL 			0U
#define DEBUG_UART_RX_DMA_DEFAULT_CHANNEL 			1U

/*!
 * SE I2C configuration
 */
#define SE_I2C_MASTER_BASEADDR 						I2C1
#define SE_I2C_MASTER_DMAMUX_BASEADDR 				DMAMUX
#define SE_I2C_MASTER_DMA_BASEADDR					DMA0
#define SE_I2C_MASTER_DMA_REQUEST_SOURCE 			kDmaRequestMux0I2C1
#define SE_I2C_MASTER_DMA_CHANNEL 					5U
#define SE_I2C_INTERNAL_ADDRESS_SIZE				I2C_ADDR_SIZE_8
#define SE_I2C_BAUDRATE 							100000U
#define SE_I2C_FAKE_PIN								NC

/*!
 * GPIO interrupt configuration
 */
#define BOARD_PORTA_IRQ_HANDLER                     PORTA_IRQHandler
#define BOARD_PORTB_IRQ_HANDLER                     PORTB_IRQHandler
#define BOARD_PORTC_IRQ_HANDLER                     PORTC_IRQHandler
#define BOARD_PORTD_IRQ_HANDLER                     PORTD_IRQHandler
#define BOARD_PORTE_IRQ_HANDLER                     PORTE_IRQHandler

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
