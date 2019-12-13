/*!
 * \file      multiRfm96w.h
 *
 * \brief     RFM96W driver implementation for multiple
 *            transceivers in parallel. Highly derived
 *            from SX1276 driver
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 *
 * \author    (sx1276.h) Miguel Luis ( Semtech )
 *
 * \author    (sx1276.h) Gregory Cristian ( Semtech )
 */

#ifndef __MULTIRFM96W_H__
#define __MULTIRFM96W_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"
#include "spi.h"
#include "multiRadio.h"
#include "multiRfm96wRegs-Fsk.h"
#include "multiRfm96wRegs-LoRa.h"

/*!
 * Radio wake-up time from sleep
 */
#define RADIO_WAKEUP_TIME 1 // [ms]

/*!
 * Sync word for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD 0x12

/*!
 * Sync word for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD 0x34

  /*!
 * Radio FSK modem parameters
 */
  typedef struct
  {
    int8_t Power;
    uint32_t Fdev;
    uint32_t Bandwidth;
    uint32_t BandwidthAfc;
    uint32_t Datarate;
    uint16_t PreambleLen;
    bool FixLen;
    uint8_t PayloadLen;
    bool CrcOn;
    bool IqInverted;
    bool RxContinuous;
    uint32_t TxTimeout;
    uint32_t RxSingleTimeout;
  } RadioFskSettings_t;

  /*!
 * Radio FSK packet handler state
 */
  typedef struct
  {
    uint8_t PreambleDetected;
    uint8_t SyncWordDetected;
    int8_t RssiValue;
    int32_t AfcValue;
    uint8_t RxGain;
    uint16_t Size;
    uint16_t NbBytes;
    uint8_t FifoThresh;
    uint8_t ChunkSize;
  } RadioFskPacketHandler_t;

  /*!
 * Radio LoRa modem parameters
 */
  typedef struct
  {
    int8_t Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    bool LowDatarateOptimize;
    uint8_t Coderate;
    uint16_t PreambleLen;
    bool FixLen;
    uint8_t PayloadLen;
    bool CrcOn;
    bool FreqHopOn;
    uint8_t HopPeriod;
    bool IqInverted;
    bool RxContinuous;
    uint32_t TxTimeout;
    bool PublicNetwork;
  } RadioLoRaSettings_t;

  /*!
 * Radio LoRa packet handler state
 */
  typedef struct
  {
    int8_t SnrValue;
    int16_t RssiValue;
    uint8_t Size;
  } RadioLoRaPacketHandler_t;

  /*!
 * Radio Settings
 */
  typedef struct
  {
    RadioState_t State;
    RadioModems_t Modem;
    uint32_t Channel;
    RadioFskSettings_t Fsk;
    RadioFskPacketHandler_t FskPacketHandler;
    RadioLoRaSettings_t LoRa;
    RadioLoRaPacketHandler_t LoRaPacketHandler;
  } RadioSettings_t;

  /*!
 * Radio hardware and global parameters
 */
  typedef struct MULTIRFM96W_s
  {
    Gpio_t Reset; //Must be the same on each radio
    Gpio_t DIO0;
    Gpio_t DIO1;
    Gpio_t DIO2;
    Gpio_t DIO3;
    Gpio_t DIO4;
    Gpio_t DIO5;
    Spi_t Spi; //Must be the same on each radio
    RadioSettings_t Settings;
  } MULTIRFM96W_t;

  /*!
 * Hardware IO IRQ callback function definition
 */
  typedef void(DioIrqHandler)(void *context);

/*!
 * MULTIRFM96W definitions
 */
#define XTAL_FREQ 32000000
#define FREQ_STEP 61.03515625

#define RX_BUFFER_SIZE 256

  /*!
 * ============================================================================
 * Public functions prototypes
 * ============================================================================
 */

  /*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
  void MULTIRFM96WInit(RadioIndex_t radio, RadioEvents_t *events);

  /*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
  RadioState_t MULTIRFM96WGetStatus(RadioIndex_t radio);

  /*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
  void MULTIRFM96WSetModem(RadioIndex_t radio, RadioModems_t modem);

  /*!
 * \brief Sets the channel configuration
 *
 * \param [IN] freq         Channel RF frequency
 */
  void MULTIRFM96WSetChannel(RadioIndex_t radio, uint32_t freq);

  /*!
 * \brief Checks if the channel is free for the given time
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] freq       Channel RF frequency
 * \param [IN] rssiThresh RSSI threshold
 * \param [IN] maxCarrierSenseTime Max time while the RSSI is measured
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
  bool MULTIRFM96WIsChannelFree(RadioIndex_t radio, RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime);

  /*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either MULTIRFM96WSetRxConfig or
 *         MULTIRFM96WSetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
  uint32_t MULTIRFM96WRandom(RadioIndex_t radio);

  /*!
 * \brief Sets the reception parameters
 *
 * \remark When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout number of bytes
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] freqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] hopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
  void MULTIRFM96WSetRxConfig(RadioIndex_t radio, RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint32_t bandwidthAfc, uint16_t preambleLen,
                              uint16_t symbTimeout, bool fixLen,
                              uint8_t payloadLen,
                              bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                              bool iqInverted, bool rxContinuous);

  /*!
 * \brief Sets the transmission parameters
 *
 * \remark When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] freqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] hopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
  void MULTIRFM96WSetTxConfig(RadioIndex_t radio, RadioModems_t modem, int8_t power, uint32_t fdev,
                              uint32_t bandwidth, uint32_t datarate,
                              uint8_t coderate, uint16_t preambleLen,
                              bool fixLen, bool crcOn, bool freqHopOn,
                              uint8_t hopPeriod, bool iqInverted, uint32_t timeout);

  /*!
 * \brief Computes the packet time on air in ms for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] pktLen     Packet payload length
 *
 * \retval airTime        Computed airTime (ms) for the given packet payload length
 */
  uint32_t MULTIRFM96WGetTimeOnAir(RadioIndex_t radio, RadioModems_t modem, uint8_t pktLen);

  /*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
  void MULTIRFM96WSend(RadioIndex_t radio, uint8_t *buffer, uint8_t size);

  /*!
 * \brief Sets the radio in sleep mode
 */
  void MULTIRFM96WSetSleep(RadioIndex_t radio);

  /*!
 * \brief Sets the radio in standby mode
 */
  void MULTIRFM96WSetStby(RadioIndex_t radio);

  /*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms] [0: continuous, others timeout]
 */
  void MULTIRFM96WSetRx(RadioIndex_t radio, uint32_t timeout);

  /*!
 * \brief Start a Channel Activity Detection
 */
  void MULTIRFM96WStartCad(RadioIndex_t radio);

  /*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param [IN]: freq       Channel RF frequency
 * \param [IN]: power      Sets the output power [dBm]
 * \param [IN]: time       Transmission mode timeout [s]
 */
  void MULTIRFM96WSetTxContinuousWave(RadioIndex_t radio, uint32_t freq, int8_t power, uint16_t time);

  /*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
  int16_t MULTIRFM96WReadRssi(RadioIndex_t radio, RadioModems_t modem);

  /*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
  void MULTIRFM96WWrite(RadioIndex_t radio, uint16_t addr, uint8_t data);

  /*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
  uint8_t MULTIRFM96WRead(RadioIndex_t radio, uint16_t addr);

  /*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
  void MULTIRFM96WWriteBuffer(RadioIndex_t radio, uint16_t addr, uint8_t *buffer, uint8_t size);

  /*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
  void MULTIRFM96WReadBuffer(RadioIndex_t radio, uint16_t addr, uint8_t *buffer, uint8_t size);

  /*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
  void MULTIRFM96WSetMaxPayloadLength(RadioIndex_t radio, RadioModems_t modem, uint8_t max);

  /*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
  void MULTIRFM96WSetPublicNetwork(RadioIndex_t radio, bool enable);

  /*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
  uint32_t MULTIRFM96WGetWakeupTime(RadioIndex_t radio);

#ifdef __cplusplus
}
#endif

#endif // __MULTIRFM96W_H__
