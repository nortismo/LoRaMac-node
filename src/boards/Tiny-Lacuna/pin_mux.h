/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define SOPT5_UART1TXSRC_UART_TX 0x00u /*!<@brief UART 1 transmit data source select: UART1_TX pin */

/*! @name PORTC3 (number 46), J1[14]
  @{ */
#define BOARD_INITPINS_CLKOUT_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_CLKOUT_PIN 3U     /*!<@brief PORTC pin index: 3 */
                                         /* @} */

/*! @name PORTC4 (number 49), J8[P2]/J24[9]/uSD_card_CS
  @{ */
#define BOARD_INITPINS_SD_CARD_DAT3_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_SD_CARD_DAT3_PIN 4U     /*!<@brief PORTC pin index: 4 */
                                               /* @} */

/*! @name PORTD5 (number 62), J2[12]/BLUE_LED
  @{ */
#define BOARD_INITPINS_LEDRGB_BLUE_GPIO GPIOD /*!<@brief GPIO device name: GPIOD */
#define BOARD_INITPINS_LEDRGB_BLUE_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_LEDRGB_BLUE_PIN 5U     /*!<@brief PORTD pin index: 5 */
                                              /* @} */

/*! @name PORTD2 (number 59), J1[2]/J8[P3]/uSD_SPI_MOSI
  @{ */
#define BOARD_INITPINS_SD_CARD_CMD_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_SD_CARD_CMD_PIN 2U     /*!<@brief PORTD pin index: 2 */
                                              /* @} */

/*! @name PORTD3 (number 60), J1[4]/J8[P7]/SPI0_SIN/uSD_SPI_MISO
  @{ */
#define BOARD_INITPINS_SD_CARD_DAT0_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_SD_CARD_DAT0_PIN 3U     /*!<@brief PORTD pin index: 3 */
                                               /* @} */

/*! @name PORTD4 (number 61), J2[6]/SPI0_PCS1/LLWU_P14
  @{ */
#define BOARD_INITPINS_RF_CS_PORT PORTD /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_RF_CS_PIN 4U     /*!<@brief PORTD pin index: 4 */
                                        /* @} */

/*! @name PORTC11 (number 56), J2[7]/I2C1_SDA
  @{ */
#define BOARD_INITPINS_RF_CE_PORT PORTC /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_RF_CE_PIN 11U    /*!<@brief PORTC pin index: 11 */
                                        /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
