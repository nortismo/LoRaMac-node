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

/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_13_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 5. */
#define PIO0_13_FUNC_ALT5 0x05u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO0_13_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_14_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 6. */
#define PIO0_14_FUNC_ALT6 0x06u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO0_14_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_17_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_17_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_19_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 7. */
#define PIO0_19_FUNC_ALT7 0x07u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO0_19_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_1_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_1_FUNC_ALT0 0x00u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO0_1_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_20_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 7. */
#define PIO0_20_FUNC_ALT7 0x07u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO0_20_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_22_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_22_FUNC_ALT0 0x00u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_25_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_25_FUNC_ALT0 0x00u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO0_25_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_28_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_28_FUNC_ALT0 0x00u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO0_28_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_29_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 1. */
#define PIO0_29_FUNC_ALT1 0x01u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO0_29_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_2_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 1. */
#define PIO0_2_FUNC_ALT1 0x01u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO0_2_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_30_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 1. */
#define PIO0_30_FUNC_ALT1 0x01u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO0_30_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_3_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 1. */
#define PIO0_3_FUNC_ALT1 0x01u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO0_3_MODE_PULL_UP 0x02u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_6_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO0_6_FUNC_ALT0 0x00u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO0_6_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO0_7_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 4. */
#define PIO0_7_FUNC_ALT4 0x04u
/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO0_7_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select Digital mode.: Enable Digital mode. Digital input is enabled. */
#define PIO1_0_DIGIMODE_DIGITAL 0x01u
/*!
 * @brief Selects pin function.: Alternative connection 0. */
#define PIO1_0_FUNC_ALT0 0x00u
/*!
 * @brief Selects function mode (on-chip pull-up/pull-down resistor control).: Pull-up. Pull-up resistor enabled. */
#define PIO1_0_MODE_PULL_UP 0x02u

/*! @name PIO0_1 (number 2), RADIO_NSS
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_RADIO_NSS_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_RADIO_NSS_GPIO_PIN_MASK (1U << 1U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_RADIO_NSS_PORT 0U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_RADIO_NSS_PIN 1U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_RADIO_NSS_PIN_MASK (1U << 1U)      /*!<@brief PORT pin mask */
                                                          /* @} */

/*! @name PIO0_6 (number 57), RADIO_ANT_SW
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_RADIO_ANT_SW_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_RADIO_ANT_SW_GPIO_PIN_MASK (1U << 6U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_RADIO_ANT_SW_PORT 0U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_RADIO_ANT_SW_PIN 6U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_RADIO_ANT_SW_PIN_MASK (1U << 6U)      /*!<@brief PORT pin mask */
                                                             /* @} */

/*! @name PIO0_28 (number 44), RADIO_DIO1
  @{ */
#define BOARD_INITPINS_RADIO_DIO1_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_RADIO_DIO1_PIN 28U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_RADIO_DIO1_PIN_MASK (1U << 28U)      /*!<@brief PORT pin mask */
                                                            /* @} */

/*! @name PIO0_17 (number 3), RADIO_RESET
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_RADIO_RESET_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_RADIO_RESET_GPIO_PIN_MASK (1U << 17U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_RADIO_RESET_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_RADIO_RESET_PIN 17U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_RADIO_RESET_PIN_MASK (1U << 17U)      /*!<@brief PORT pin mask */
                                                             /* @} */

/*! @name PIO1_0 (number 4), RADIO_BUSY
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_RADIO_BUSY_GPIO GPIO                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_RADIO_BUSY_GPIO_PIN_MASK (1U << 0U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_RADIO_BUSY_PORT 1U                  /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_RADIO_BUSY_PIN 0U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_RADIO_BUSY_PIN_MASK (1U << 0U)      /*!<@brief PORT pin mask */
                                                           /* @} */

/*! @name PIO0_22 (number 50), GNSS_RESET
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_GNSS_RESET_GPIO GPIO                 /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_GNSS_RESET_GPIO_PIN_MASK (1U << 22U) /*!<@brief GPIO pin mask */
#define BOARD_INITPINS_GNSS_RESET_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_GNSS_RESET_PIN 22U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_GNSS_RESET_PIN_MASK (1U << 22U)      /*!<@brief PORT pin mask */
                                                            /* @} */

/*! @name PIO0_25 (number 51), GNSS_PPS
  @{ */
#define BOARD_INITPINS_GNSS_PPS_PORT 0U                   /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_GNSS_PPS_PIN 25U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_GNSS_PPS_PIN_MASK (1U << 25U)      /*!<@brief PORT pin mask */
                                                          /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void); /* Function assigned for the Cortex-M33 */

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
