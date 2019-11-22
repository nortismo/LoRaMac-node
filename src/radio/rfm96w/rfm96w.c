/*!
 * \file      rfm96w.c
 *
 * \brief     RFM96W driver implementation. Highly derived from SX1276 driver
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 *
 * \author    (sx1276.c) Miguel Luis ( Semtech )
 *
 * \author    (sx1276.c) Gregory Cristian ( Semtech )
 *
 * \author    (sx1276.c) Wael Guibene ( Semtech )
 */

#include <math.h>
#include <string.h>
#include "utilities.h"
#include "timer.h"
#include "radio.h"
#include "delay.h"
#include "rfm96w.h"
#include "rfm96w-board.h"

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t Addr;
    uint8_t Value;
} RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t RegValue;
} FskBandwidth_t;

/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration(void);

/*!
 * \brief Sets the RFM96W in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
void RFM96WSetTx(uint32_t timeout);

/*!
 * \brief Writes the buffer contents to the RFM96W FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void RFM96WWriteFifo(uint8_t *buffer, uint8_t size);

/*!
 * \brief Reads the contents of the RFM96W FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void RFM96WReadFifo(uint8_t *buffer, uint8_t size);

/*!
 * \brief Sets the RFM96W operating mode
 *
 * \param [IN] opMode New operating mode
 */
void RFM96WSetOpMode(uint8_t opMode);

/*
 * RFM96W DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void RFM96WOnDio0Irq(void *context);

/*!
 * \brief DIO 1 IRQ callback
 */
void RFM96WOnDio1Irq(void *context);

/*!
 * \brief DIO 2 IRQ callback
 */
void RFM96WOnDio2Irq(void *context);

/*!
 * \brief DIO 3 IRQ callback
 */
void RFM96WOnDio3Irq(void *context);

/*!
 * \brief DIO 4 IRQ callback
 */
void RFM96WOnDio4Irq(void *context);

/*!
 * \brief DIO 5 IRQ callback
 */
void RFM96WOnDio5Irq(void *context);

/*!
 * \brief Tx & Rx timeout timer callback
 */
void RFM96WOnTimeoutIrq(void *context);

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in RFM96W-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF -164
#define RSSI_OFFSET_HF -157

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
    {
        {2600, 0x17},
        {3100, 0x0F},
        {3900, 0x07},
        {5200, 0x16},
        {6300, 0x0E},
        {7800, 0x06},
        {10400, 0x15},
        {12500, 0x0D},
        {15600, 0x05},
        {20800, 0x14},
        {25000, 0x0C},
        {31300, 0x04},
        {41700, 0x13},
        {50000, 0x0B},
        {62500, 0x03},
        {83333, 0x12},
        {100000, 0x0A},
        {125000, 0x02},
        {166700, 0x11},
        {200000, 0x09},
        {250000, 0x01},
        {300000, 0x00}, // Invalid Bandwidth
};

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
RFM96W_t RFM96W;

/*!
 * Hardware DIO IRQ callback initialization
 */
DioIrqHandler *DioIrq[] = {RFM96WOnDio0Irq, RFM96WOnDio1Irq,
                           RFM96WOnDio2Irq, RFM96WOnDio3Irq,
                           RFM96WOnDio4Irq, NULL};

/*!
 * Tx and Rx timers
 */
TimerEvent_t TxTimeoutTimer;
TimerEvent_t RxTimeoutTimer;
TimerEvent_t RxTimeoutSyncWord;

/*
 * Radio driver functions implementation
 */

void RFM96WInit(RadioEvents_t *events)
{
    uint8_t i;

    RadioEvents = events;

    // Initialize driver timeout timers
    TimerInit(&TxTimeoutTimer, RFM96WOnTimeoutIrq);
    TimerInit(&RxTimeoutTimer, RFM96WOnTimeoutIrq);
    TimerInit(&RxTimeoutSyncWord, RFM96WOnTimeoutIrq);

    RFM96WReset();

    RxChainCalibration();

    //Disable High Power (Pmax to 17dBm instead of 20dBm)
    RFM96WWrite(REG_LR_PADAC, (RFM96WRead(REG_LR_PADAC) & RFLR_PADAC_20DBM_MASK | RFLR_PADAC_20DBM_OFF));

    RFM96WSetOpMode(RF_OPMODE_SLEEP);

    RFM96WIoIrqInit(DioIrq);

    for (i = 0; i < sizeof(RadioRegsInit) / sizeof(RadioRegisters_t); i++)
    {
        RFM96WSetModem(RadioRegsInit[i].Modem);
        RFM96WWrite(RadioRegsInit[i].Addr, RadioRegsInit[i].Value);
    }

    RFM96WSetModem(MODEM_FSK);

    //Enable low frequency
    RFM96WWrite(REG_OPMODE, (RFM96WRead(REG_OPMODE) & RFLR_OPMODE_FREQMODE_ACCESS_MASK | RFLR_OPMODE_FREQMODE_ACCESS_LF));

    //LNA gain set by the internal AGC loop
    RFM96WWrite(REG_LR_MODEMCONFIG3, (RFM96WRead(REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_AGCAUTO_MASK | RFLR_MODEMCONFIG3_AGCAUTO_ON));

    //Reset FIFOTX Pointer
    RFM96WWrite(REG_LR_FIFOTXBASEADDR, 0);

    //Reset FIFO ADDR Pointer
    RFM96WWrite(REG_LR_FIFOADDRPTR, 0);

    //Reset IRQ Flags
    RFM96WWrite(REG_LR_IRQFLAGS, 0xFF);

    //Reset PaRamp (Modulation shaping)
    RFM96WWrite(REG_LR_PARAMP, 0x08);

    RFM96W.Settings.State = RF_IDLE;
}

RadioState_t RFM96WGetStatus(void)
{
    return RFM96W.Settings.State;
}

void RFM96WSetChannel(uint32_t freq)
{
    RFM96W.Settings.Channel = freq;
    freq = (uint32_t)((double)freq / (double)FREQ_STEP);
    RFM96WWrite(REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
    RFM96WWrite(REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
    RFM96WWrite(REG_FRFLSB, (uint8_t)(freq & 0xFF));
}

bool RFM96WIsChannelFree(RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime)
{
    bool status = true;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;

    RFM96WSetSleep();

    RFM96WSetModem(modem);

    RFM96WSetChannel(freq);

    RFM96WSetOpMode(RF_OPMODE_RECEIVER);

    DelayMs(1);

    carrierSenseTime = TimerGetCurrentTime();

    // Perform carrier sense for maxCarrierSenseTime
    while (TimerGetElapsedTime(carrierSenseTime) < maxCarrierSenseTime)
    {
        rssi = RFM96WReadRssi(modem);

        if (rssi > rssiThresh)
        {
            status = false;
            break;
        }
    }
    RFM96WSetSleep();
    return status;
}

uint32_t RFM96WRandom(void)
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    RFM96WSetModem(MODEM_LORA);

    // Disable LoRa modem interrupts
    RFM96WWrite(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                         RFLR_IRQFLAGS_RXDONE |
                                         RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                         RFLR_IRQFLAGS_VALIDHEADER |
                                         RFLR_IRQFLAGS_TXDONE |
                                         RFLR_IRQFLAGS_CADDONE |
                                         RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                         RFLR_IRQFLAGS_CADDETECTED);

    // Set radio in continuous reception
    RFM96WSetOpMode(RF_OPMODE_RECEIVER);

    for (i = 0; i < 32; i++)
    {
        DelayMs(1);
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ((uint32_t)RFM96WRead(REG_LR_RSSIWIDEBAND) & 0x01) << i;
    }

    RFM96WSetSleep();

    return rnd;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration(void)
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = RFM96WRead(REG_PACONFIG);
    initialFreq = (double)(((uint32_t)RFM96WRead(REG_FRFMSB) << 16) |
                           ((uint32_t)RFM96WRead(REG_FRFMID) << 8) |
                           ((uint32_t)RFM96WRead(REG_FRFLSB))) *
                  (double)FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    RFM96WWrite(REG_PACONFIG, 0x00);

    // Launch Rx chain calibration for LF band
    RFM96WWrite(REG_IMAGECAL, (RFM96WRead(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while ((RFM96WRead(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
    {
    }

    // Sets a Frequency in HF band
    RFM96WSetChannel(868000000);

    // Launch Rx chain calibration for HF band
    RFM96WWrite(REG_IMAGECAL, (RFM96WRead(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while ((RFM96WRead(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
    {
    }

    // Restore context
    RFM96WWrite(REG_PACONFIG, regPaConfigInitVal);
    RFM96WSetChannel(initialFreq);
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t GetFskBandwidthRegValue(uint32_t bandwidth)
{
    uint8_t i;

    for (i = 0; i < (sizeof(FskBandwidths) / sizeof(FskBandwidth_t)) - 1; i++)
    {
        if ((bandwidth >= FskBandwidths[i].bandwidth) && (bandwidth < FskBandwidths[i + 1].bandwidth))
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while (1)
        ;
}

void RFM96WSetRxConfig(RadioModems_t modem, uint32_t bandwidth,
                       uint32_t datarate, uint8_t coderate,
                       uint32_t bandwidthAfc, uint16_t preambleLen,
                       uint16_t symbTimeout, bool fixLen,
                       uint8_t payloadLen,
                       bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                       bool iqInverted, bool rxContinuous)
{
    RFM96WSetModem(modem);

    switch (modem)
    {
    case MODEM_FSK:
    {
        RFM96W.Settings.Fsk.Bandwidth = bandwidth;
        RFM96W.Settings.Fsk.Datarate = datarate;
        RFM96W.Settings.Fsk.BandwidthAfc = bandwidthAfc;
        RFM96W.Settings.Fsk.FixLen = fixLen;
        RFM96W.Settings.Fsk.PayloadLen = payloadLen;
        RFM96W.Settings.Fsk.CrcOn = crcOn;
        RFM96W.Settings.Fsk.IqInverted = iqInverted;
        RFM96W.Settings.Fsk.RxContinuous = rxContinuous;
        RFM96W.Settings.Fsk.PreambleLen = preambleLen;
        RFM96W.Settings.Fsk.RxSingleTimeout = (uint32_t)(symbTimeout * ((1.0 / (double)datarate) * 8.0) * 1000);

        datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);
        RFM96WWrite(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
        RFM96WWrite(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

        RFM96WWrite(REG_RXBW, GetFskBandwidthRegValue(bandwidth));
        RFM96WWrite(REG_AFCBW, GetFskBandwidthRegValue(bandwidthAfc));

        RFM96WWrite(REG_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
        RFM96WWrite(REG_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

        if (fixLen == 1)
        {
            RFM96WWrite(REG_PAYLOADLENGTH, payloadLen);
        }
        else
        {
            RFM96WWrite(REG_PAYLOADLENGTH, 0xFF); // Set payload length to the maximum
        }

        RFM96WWrite(REG_PACKETCONFIG1,
                    (RFM96WRead(REG_PACKETCONFIG1) &
                     RF_PACKETCONFIG1_CRC_MASK &
                     RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                        ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                        (crcOn << 4));
        RFM96WWrite(REG_PACKETCONFIG2, (RFM96WRead(REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
    }
    break;
    case MODEM_LORA:
    {
        if (bandwidth > 2)
        {
            // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            while (1)
                ;
        }
        bandwidth += 7;
        RFM96W.Settings.LoRa.Bandwidth = bandwidth;
        RFM96W.Settings.LoRa.Datarate = datarate;
        RFM96W.Settings.LoRa.Coderate = coderate;
        RFM96W.Settings.LoRa.PreambleLen = preambleLen;
        RFM96W.Settings.LoRa.FixLen = fixLen;
        RFM96W.Settings.LoRa.PayloadLen = payloadLen;
        RFM96W.Settings.LoRa.CrcOn = crcOn;
        RFM96W.Settings.LoRa.FreqHopOn = freqHopOn;
        RFM96W.Settings.LoRa.HopPeriod = hopPeriod;
        RFM96W.Settings.LoRa.IqInverted = iqInverted;
        RFM96W.Settings.LoRa.RxContinuous = rxContinuous;

        if (datarate > 12)
        {
            datarate = 12;
        }
        else if (datarate < 6)
        {
            datarate = 6;
        }

        if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 8) && (datarate == 12)))
        {
            RFM96W.Settings.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            RFM96W.Settings.LoRa.LowDatarateOptimize = 0x00;
        }

        RFM96WWrite(REG_LR_MODEMCONFIG1,
                    (RFM96WRead(REG_LR_MODEMCONFIG1) &
                     RFLR_MODEMCONFIG1_BW_MASK &
                     RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                     RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                        (bandwidth << 4) | (coderate << 1) |
                        fixLen);

        RFM96WWrite(REG_LR_MODEMCONFIG2,
                    (RFM96WRead(REG_LR_MODEMCONFIG2) &
                     RFLR_MODEMCONFIG2_SF_MASK &
                     RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                     RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) |
                        (datarate << 4) | (crcOn << 2) |
                        ((symbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

        RFM96WWrite(REG_LR_MODEMCONFIG3,
                    (RFM96WRead(REG_LR_MODEMCONFIG3) &
                     RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                        (RFM96W.Settings.LoRa.LowDatarateOptimize << 3));

        RFM96WWrite(REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symbTimeout & 0xFF));

        RFM96WWrite(REG_LR_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
        RFM96WWrite(REG_LR_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

        if (fixLen == 1)
        {
            RFM96WWrite(REG_LR_PAYLOADLENGTH, payloadLen);
        }

        if (RFM96W.Settings.LoRa.FreqHopOn == true)
        {
            RFM96WWrite(REG_LR_PLLHOP, (RFM96WRead(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
            RFM96WWrite(REG_LR_HOPPERIOD, RFM96W.Settings.LoRa.HopPeriod);
        }

        if ((bandwidth == 9) && (RFM96W.Settings.Channel > RF_MID_BAND_THRESH))
        {
            // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
            RFM96WWrite(REG_LR_HIGHBWOPTIMIZE1, 0x02);
            RFM96WWrite(REG_LR_HIGHBWOPTIMIZE2, 0x64);
        }
        else if (bandwidth == 9)
        {
            // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
            RFM96WWrite(REG_LR_HIGHBWOPTIMIZE1, 0x02);
            RFM96WWrite(REG_LR_HIGHBWOPTIMIZE2, 0x7F);
        }
        else
        {
            // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
            RFM96WWrite(REG_LR_HIGHBWOPTIMIZE1, 0x03);
        }

        if (datarate == 6)
        {
            RFM96WWrite(REG_LR_DETECTOPTIMIZE,
                        (RFM96WRead(REG_LR_DETECTOPTIMIZE) &
                         RFLR_DETECTIONOPTIMIZE_MASK) |
                            RFLR_DETECTIONOPTIMIZE_SF6);
            RFM96WWrite(REG_LR_DETECTIONTHRESHOLD,
                        RFLR_DETECTIONTHRESH_SF6);
        }
        else
        {
            RFM96WWrite(REG_LR_DETECTOPTIMIZE,
                        (RFM96WRead(REG_LR_DETECTOPTIMIZE) &
                         RFLR_DETECTIONOPTIMIZE_MASK) |
                            RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
            RFM96WWrite(REG_LR_DETECTIONTHRESHOLD,
                        RFLR_DETECTIONTHRESH_SF7_TO_SF12);
        }
    }
    break;
    }
}

void RFM96WSetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                       uint32_t bandwidth, uint32_t datarate,
                       uint8_t coderate, uint16_t preambleLen,
                       bool fixLen, bool crcOn, bool freqHopOn,
                       uint8_t hopPeriod, bool iqInverted, uint32_t timeout)
{
    RFM96WSetModem(modem);

    RFM96WSetRfTxPower(power);

    switch (modem)
    {
    case MODEM_FSK:
    {
        RFM96W.Settings.Fsk.Power = power;
        RFM96W.Settings.Fsk.Fdev = fdev;
        RFM96W.Settings.Fsk.Bandwidth = bandwidth;
        RFM96W.Settings.Fsk.Datarate = datarate;
        RFM96W.Settings.Fsk.PreambleLen = preambleLen;
        RFM96W.Settings.Fsk.FixLen = fixLen;
        RFM96W.Settings.Fsk.CrcOn = crcOn;
        RFM96W.Settings.Fsk.IqInverted = iqInverted;
        RFM96W.Settings.Fsk.TxTimeout = timeout;

        fdev = (uint16_t)((double)fdev / (double)FREQ_STEP);
        RFM96WWrite(REG_FDEVMSB, (uint8_t)(fdev >> 8));
        RFM96WWrite(REG_FDEVLSB, (uint8_t)(fdev & 0xFF));

        datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);
        RFM96WWrite(REG_BITRATEMSB, (uint8_t)(datarate >> 8));
        RFM96WWrite(REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

        RFM96WWrite(REG_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
        RFM96WWrite(REG_PREAMBLELSB, preambleLen & 0xFF);

        RFM96WWrite(REG_PACKETCONFIG1,
                    (RFM96WRead(REG_PACKETCONFIG1) &
                     RF_PACKETCONFIG1_CRC_MASK &
                     RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                        ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                        (crcOn << 4));
        RFM96WWrite(REG_PACKETCONFIG2, (RFM96WRead(REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
    }
    break;
    case MODEM_LORA:
    {
        RFM96W.Settings.LoRa.Power = power;
        if (bandwidth > 2)
        {
            // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            while (1)
                ;
        }
        bandwidth += 7;
        RFM96W.Settings.LoRa.Bandwidth = bandwidth;
        RFM96W.Settings.LoRa.Datarate = datarate;
        RFM96W.Settings.LoRa.Coderate = coderate;
        RFM96W.Settings.LoRa.PreambleLen = preambleLen;
        RFM96W.Settings.LoRa.FixLen = fixLen;
        RFM96W.Settings.LoRa.FreqHopOn = freqHopOn;
        RFM96W.Settings.LoRa.HopPeriod = hopPeriod;
        RFM96W.Settings.LoRa.CrcOn = crcOn;
        RFM96W.Settings.LoRa.IqInverted = iqInverted;
        RFM96W.Settings.LoRa.TxTimeout = timeout;

        if (datarate > 12)
        {
            datarate = 12;
        }
        else if (datarate < 6)
        {
            datarate = 6;
        }
        if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 8) && (datarate == 12)))
        {
            RFM96W.Settings.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            RFM96W.Settings.LoRa.LowDatarateOptimize = 0x00;
        }

        if (RFM96W.Settings.LoRa.FreqHopOn == true)
        {
            RFM96WWrite(REG_LR_PLLHOP, (RFM96WRead(REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
            RFM96WWrite(REG_LR_HOPPERIOD, RFM96W.Settings.LoRa.HopPeriod);
        }

        RFM96WWrite(REG_LR_MODEMCONFIG1,
                    (RFM96WRead(REG_LR_MODEMCONFIG1) &
                     RFLR_MODEMCONFIG1_BW_MASK &
                     RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                     RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                        (bandwidth << 4) | (coderate << 1) |
                        fixLen);

        RFM96WWrite(REG_LR_MODEMCONFIG2,
                    (RFM96WRead(REG_LR_MODEMCONFIG2) &
                     RFLR_MODEMCONFIG2_SF_MASK &
                     RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) |
                        (datarate << 4) | (crcOn << 2));

        RFM96WWrite(REG_LR_MODEMCONFIG3,
                    (RFM96WRead(REG_LR_MODEMCONFIG3) &
                     RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                        (RFM96W.Settings.LoRa.LowDatarateOptimize << 3));

        RFM96WWrite(REG_LR_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
        RFM96WWrite(REG_LR_PREAMBLELSB, preambleLen & 0xFF);

        if (datarate == 6)
        {
            RFM96WWrite(REG_LR_DETECTOPTIMIZE,
                        (RFM96WRead(REG_LR_DETECTOPTIMIZE) &
                         RFLR_DETECTIONOPTIMIZE_MASK) |
                            RFLR_DETECTIONOPTIMIZE_SF6);
            RFM96WWrite(REG_LR_DETECTIONTHRESHOLD,
                        RFLR_DETECTIONTHRESH_SF6);
        }
        else
        {
            RFM96WWrite(REG_LR_DETECTOPTIMIZE,
                        (RFM96WRead(REG_LR_DETECTOPTIMIZE) &
                         RFLR_DETECTIONOPTIMIZE_MASK) |
                            RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
            RFM96WWrite(REG_LR_DETECTIONTHRESHOLD,
                        RFLR_DETECTIONTHRESH_SF7_TO_SF12);
        }
    }
    break;
    }
}

uint32_t RFM96WGetTimeOnAir(RadioModems_t modem, uint8_t pktLen)
{
    uint32_t airTime = 0;

    switch (modem)
    {
    case MODEM_FSK:
    {
        airTime = round((8 * (RFM96W.Settings.Fsk.PreambleLen + ((RFM96WRead(REG_SYNCCONFIG) & ~RF_SYNCCONFIG_SYNCSIZE_MASK) + 1) + ((RFM96W.Settings.Fsk.FixLen == 0x01) ? 0.0 : 1.0) + (((RFM96WRead(REG_PACKETCONFIG1) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK) != 0x00) ? 1.0 : 0) + pktLen + ((RFM96W.Settings.Fsk.CrcOn == 0x01) ? 2.0 : 0)) /
                         RFM96W.Settings.Fsk.Datarate) *
                        1000);
    }
    break;
    case MODEM_LORA:
    {
        double bw = 0.0;
        // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
        switch (RFM96W.Settings.LoRa.Bandwidth)
        {
        //case 0: // 7.8 kHz
        //    bw = 7800;
        //    break;
        //case 1: // 10.4 kHz
        //    bw = 10400;
        //    break;
        //case 2: // 15.6 kHz
        //    bw = 15600;
        //    break;
        //case 3: // 20.8 kHz
        //    bw = 20800;
        //    break;
        //case 4: // 31.2 kHz
        //    bw = 31200;
        //    break;
        //case 5: // 41.4 kHz
        //    bw = 41400;
        //    break;
        //case 6: // 62.5 kHz
        //    bw = 62500;
        //    break;
        case 7: // 125 kHz
            bw = 125000;
            break;
        case 8: // 250 kHz
            bw = 250000;
            break;
        case 9: // 500 kHz
            bw = 500000;
            break;
        }

        // Symbol rate : time for one symbol (secs)
        double rs = bw / (1 << RFM96W.Settings.LoRa.Datarate);
        double ts = 1 / rs;
        // time of preamble
        double tPreamble = (RFM96W.Settings.LoRa.PreambleLen + 4.25) * ts;
        // Symbol length of payload and time
        double tmp = ceil((8 * pktLen - 4 * RFM96W.Settings.LoRa.Datarate +
                           28 + 16 * RFM96W.Settings.LoRa.CrcOn -
                           (RFM96W.Settings.LoRa.FixLen ? 20 : 0)) /
                          (double)(4 * (RFM96W.Settings.LoRa.Datarate -
                                        ((RFM96W.Settings.LoRa.LowDatarateOptimize > 0) ? 2 : 0)))) *
                     (RFM96W.Settings.LoRa.Coderate + 4);
        double nPayload = 8 + ((tmp > 0) ? tmp : 0);
        double tPayload = nPayload * ts;
        // Time on air
        double tOnAir = tPreamble + tPayload;
        // return ms secs
        airTime = floor(tOnAir * 1000 + 0.999);
    }
    break;
    }
    return airTime;
}

void RFM96WSend(uint8_t *buffer, uint8_t size)
{
    uint32_t txTimeout = 0;

    switch (RFM96W.Settings.Modem)
    {
    case MODEM_FSK:
    {
        RFM96W.Settings.FskPacketHandler.NbBytes = 0;
        RFM96W.Settings.FskPacketHandler.Size = size;

        if (RFM96W.Settings.Fsk.FixLen == false)
        {
            RFM96WWriteFifo((uint8_t *)&size, 1);
        }
        else
        {
            RFM96WWrite(REG_PAYLOADLENGTH, size);
        }

        if ((size > 0) && (size <= 64))
        {
            RFM96W.Settings.FskPacketHandler.ChunkSize = size;
        }
        else
        {
            memcpy1(RxTxBuffer, buffer, size);
            RFM96W.Settings.FskPacketHandler.ChunkSize = 32;
        }

        // Write payload buffer
        RFM96WWriteFifo(buffer, RFM96W.Settings.FskPacketHandler.ChunkSize);
        RFM96W.Settings.FskPacketHandler.NbBytes += RFM96W.Settings.FskPacketHandler.ChunkSize;
        txTimeout = RFM96W.Settings.Fsk.TxTimeout;
    }
    break;
    case MODEM_LORA:
    {
        if (RFM96W.Settings.LoRa.IqInverted == true)
        {
            RFM96WWrite(REG_LR_INVERTIQ, ((RFM96WRead(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON));
            RFM96WWrite(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
        }
        else
        {
            RFM96WWrite(REG_LR_INVERTIQ, ((RFM96WRead(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
            RFM96WWrite(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
        }

        RFM96W.Settings.LoRaPacketHandler.Size = size;

        // Initializes the payload size
        RFM96WWrite(REG_LR_PAYLOADLENGTH, size);

        // Full buffer used for Tx
        RFM96WWrite(REG_LR_FIFOTXBASEADDR, 0);
        RFM96WWrite(REG_LR_FIFOADDRPTR, 0);

        // FIFO operations can not take place in Sleep mode
        if ((RFM96WRead(REG_OPMODE) & ~RF_OPMODE_MASK) == RF_OPMODE_SLEEP)
        {
            RFM96WSetStby();
            DelayMs(1);
        }
        // Write payload buffer
        RFM96WWriteFifo(buffer, size);
        txTimeout = RFM96W.Settings.LoRa.TxTimeout;
    }
    break;
    }

    RFM96WSetTx(txTimeout);
}

void RFM96WSetSleep(void)
{
    TimerStop(&RxTimeoutTimer);
    TimerStop(&TxTimeoutTimer);
    TimerStop(&RxTimeoutSyncWord);

    RFM96WSetOpMode(RF_OPMODE_SLEEP);

    // Disable TCXO radio is in SLEEP mode
    RFM96WSetBoardTcxo(false);

    RFM96W.Settings.State = RF_IDLE;
}

void RFM96WSetStby(void)
{
    TimerStop(&RxTimeoutTimer);
    TimerStop(&TxTimeoutTimer);
    TimerStop(&RxTimeoutSyncWord);

    RFM96WSetOpMode(RF_OPMODE_STANDBY);
    RFM96W.Settings.State = RF_IDLE;
}

void RFM96WSetRx(uint32_t timeout)
{
    bool rxContinuous = false;
    TimerStop(&TxTimeoutTimer);

    switch (RFM96W.Settings.Modem)
    {
    case MODEM_FSK:
    {
        rxContinuous = RFM96W.Settings.Fsk.RxContinuous;

        // DIO0=PayloadReady
        // DIO1=FifoLevel
        // DIO2=SyncAddr
        // DIO3=FifoEmpty
        // DIO4=Preamble
        // DIO5=ModeReady
        RFM96WWrite(REG_DIOMAPPING1, (RFM96WRead(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK &
                                      RF_DIOMAPPING1_DIO1_MASK &
                                      RF_DIOMAPPING1_DIO2_MASK) |
                                         RF_DIOMAPPING1_DIO0_00 |
                                         RF_DIOMAPPING1_DIO1_00 |
                                         RF_DIOMAPPING1_DIO2_11);

        RFM96WWrite(REG_DIOMAPPING2, (RFM96WRead(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK &
                                      RF_DIOMAPPING2_MAP_MASK) |
                                         RF_DIOMAPPING2_DIO4_11 |
                                         RF_DIOMAPPING2_MAP_PREAMBLEDETECT);

        RFM96W.Settings.FskPacketHandler.FifoThresh = RFM96WRead(REG_FIFOTHRESH) & 0x3F;

        RFM96WWrite(REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);

        RFM96W.Settings.FskPacketHandler.PreambleDetected = false;
        RFM96W.Settings.FskPacketHandler.SyncWordDetected = false;
        RFM96W.Settings.FskPacketHandler.NbBytes = 0;
        RFM96W.Settings.FskPacketHandler.Size = 0;
    }
    break;
    case MODEM_LORA:
    {
        if (RFM96W.Settings.LoRa.IqInverted == true)
        {
            RFM96WWrite(REG_LR_INVERTIQ, ((RFM96WRead(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF));
            RFM96WWrite(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
        }
        else
        {
            RFM96WWrite(REG_LR_INVERTIQ, ((RFM96WRead(REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
            RFM96WWrite(REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
        }

        // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
        if (RFM96W.Settings.LoRa.Bandwidth < 9)
        {
            RFM96WWrite(REG_LR_DETECTOPTIMIZE, RFM96WRead(REG_LR_DETECTOPTIMIZE) & 0x7F);
            RFM96WWrite(REG_LR_IFFREQ2, 0x00);
            switch (RFM96W.Settings.LoRa.Bandwidth)
            {
            case 0: // 7.8 kHz
                RFM96WWrite(REG_LR_IFFREQ1, 0x48);
                RFM96WSetChannel(RFM96W.Settings.Channel + 7810);
                break;
            case 1: // 10.4 kHz
                RFM96WWrite(REG_LR_IFFREQ1, 0x44);
                RFM96WSetChannel(RFM96W.Settings.Channel + 10420);
                break;
            case 2: // 15.6 kHz
                RFM96WWrite(REG_LR_IFFREQ1, 0x44);
                RFM96WSetChannel(RFM96W.Settings.Channel + 15620);
                break;
            case 3: // 20.8 kHz
                RFM96WWrite(REG_LR_IFFREQ1, 0x44);
                RFM96WSetChannel(RFM96W.Settings.Channel + 20830);
                break;
            case 4: // 31.2 kHz
                RFM96WWrite(REG_LR_IFFREQ1, 0x44);
                RFM96WSetChannel(RFM96W.Settings.Channel + 31250);
                break;
            case 5: // 41.4 kHz
                RFM96WWrite(REG_LR_IFFREQ1, 0x44);
                RFM96WSetChannel(RFM96W.Settings.Channel + 41670);
                break;
            case 6: // 62.5 kHz
                RFM96WWrite(REG_LR_IFFREQ1, 0x40);
                break;
            case 7: // 125 kHz
                RFM96WWrite(REG_LR_IFFREQ1, 0x40);
                break;
            case 8: // 250 kHz
                RFM96WWrite(REG_LR_IFFREQ1, 0x40);
                break;
            }
        }
        else
        {
            RFM96WWrite(REG_LR_DETECTOPTIMIZE, RFM96WRead(REG_LR_DETECTOPTIMIZE) | 0x80);
        }

        rxContinuous = RFM96W.Settings.LoRa.RxContinuous;

        if (RFM96W.Settings.LoRa.FreqHopOn == true)
        {
            RFM96WWrite(REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                        //RFLR_IRQFLAGS_RXDONE |
                        //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                        RFLR_IRQFLAGS_VALIDHEADER |
                            RFLR_IRQFLAGS_TXDONE |
                            RFLR_IRQFLAGS_CADDONE |
                            //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                            RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=RxDone, DIO2=FhssChangeChannel
            RFM96WWrite(REG_DIOMAPPING1, (RFM96WRead(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00);
        }
        else
        {
            //uncommented RFLR_IRQFLAGS_RXTIMEOUT
            RFM96WWrite(REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                        //RFLR_IRQFLAGS_RXDONE |
                        //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                        RFLR_IRQFLAGS_VALIDHEADER |
                            RFLR_IRQFLAGS_TXDONE |
                            RFLR_IRQFLAGS_CADDONE |
                            RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                            RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=RxDone
            RFM96WWrite(REG_DIOMAPPING1, (RFM96WRead(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);
        }
        RFM96WWrite(REG_LR_FIFORXBASEADDR, 0);
        RFM96WWrite(REG_LR_FIFOADDRPTR, 0);
    }
    break;
    }

    memset(RxTxBuffer, 0, (size_t)RX_BUFFER_SIZE);

    RFM96W.Settings.State = RF_RX_RUNNING;
    if (timeout != 0)
    {
        TimerSetValue(&RxTimeoutTimer, timeout);
        TimerStart(&RxTimeoutTimer);
    }

    if (RFM96W.Settings.Modem == MODEM_FSK)
    {
        RFM96WSetOpMode(RF_OPMODE_RECEIVER);

        TimerSetValue(&RxTimeoutSyncWord, RFM96W.Settings.Fsk.RxSingleTimeout);
        TimerStart(&RxTimeoutSyncWord);
    }
    else
    {
        if (rxContinuous == true)
        {
            RFM96WSetOpMode(RFLR_OPMODE_RECEIVER);
        }
        else
        {
            RFM96WSetOpMode(RFLR_OPMODE_RECEIVER_SINGLE);
        }
    }
}

void RFM96WSetTx(uint32_t timeout)
{
    TimerStop(&RxTimeoutTimer);

    TimerSetValue(&TxTimeoutTimer, timeout);

    switch (RFM96W.Settings.Modem)
    {
    case MODEM_FSK:
    {
        // DIO0=PacketSent
        // DIO1=FifoEmpty
        // DIO2=FifoFull
        // DIO3=FifoEmpty
        // DIO4=LowBat
        // DIO5=ModeReady
        RFM96WWrite(REG_DIOMAPPING1, (RFM96WRead(REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK &
                                      RF_DIOMAPPING1_DIO1_MASK &
                                      RF_DIOMAPPING1_DIO2_MASK) |
                                         RF_DIOMAPPING1_DIO1_01);

        RFM96WWrite(REG_DIOMAPPING2, (RFM96WRead(REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK &
                                      RF_DIOMAPPING2_MAP_MASK));
        RFM96W.Settings.FskPacketHandler.FifoThresh = RFM96WRead(REG_FIFOTHRESH) & 0x3F;
    }
    break;
    case MODEM_LORA:
    {
        if (RFM96W.Settings.LoRa.FreqHopOn == true)
        {
            RFM96WWrite(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                 RFLR_IRQFLAGS_RXDONE |
                                                 RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                 RFLR_IRQFLAGS_VALIDHEADER |
                                                 //RFLR_IRQFLAGS_TXDONE |
                                                 RFLR_IRQFLAGS_CADDONE |
                                                 //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                 RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=TxDone, DIO2=FhssChangeChannel
            RFM96WWrite(REG_DIOMAPPING1, (RFM96WRead(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00);
        }
        else
        {
            RFM96WWrite(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                 RFLR_IRQFLAGS_RXDONE |
                                                 RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                 RFLR_IRQFLAGS_VALIDHEADER |
                                                 //RFLR_IRQFLAGS_TXDONE |
                                                 RFLR_IRQFLAGS_CADDONE |
                                                 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                 RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=TxDone
            RFM96WWrite(REG_DIOMAPPING1, (RFM96WRead(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);
        }
    }
    break;
    }

    RFM96W.Settings.State = RF_TX_RUNNING;
    TimerStart(&TxTimeoutTimer);
    RFM96WSetOpMode(RF_OPMODE_TRANSMITTER);
}

void RFM96WStartCad(void)
{
    switch (RFM96W.Settings.Modem)
    {
    case MODEM_FSK:
    {
    }
    break;
    case MODEM_LORA:
    {
        RFM96WWrite(REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                             RFLR_IRQFLAGS_RXDONE |
                                             RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                             RFLR_IRQFLAGS_VALIDHEADER |
                                             RFLR_IRQFLAGS_TXDONE |
                                             //RFLR_IRQFLAGS_CADDONE |
                                             RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                                                              //RFLR_IRQFLAGS_CADDETECTED
        );

        // DIO3=CADDone
        RFM96WWrite(REG_DIOMAPPING1, (RFM96WRead(REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO3_MASK) | RFLR_DIOMAPPING1_DIO3_00);

        RFM96W.Settings.State = RF_CAD;
        RFM96WSetOpMode(RFLR_OPMODE_CAD);
    }
    break;
    default:
        break;
    }
}

void RFM96WSetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time)
{
    uint32_t timeout = (uint32_t)(time * 1000);

    RFM96WSetChannel(freq);

    RFM96WSetTxConfig(MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout);

    RFM96WWrite(REG_PACKETCONFIG2, (RFM96WRead(REG_PACKETCONFIG2) & RF_PACKETCONFIG2_DATAMODE_MASK));
    // Disable radio interrupts
    RFM96WWrite(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11);
    RFM96WWrite(REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10);

    TimerSetValue(&TxTimeoutTimer, timeout);

    RFM96W.Settings.State = RF_TX_RUNNING;
    TimerStart(&TxTimeoutTimer);
    RFM96WSetOpMode(RF_OPMODE_TRANSMITTER);
}

int16_t RFM96WReadRssi(RadioModems_t modem)
{
    int16_t rssi = 0;

    switch (modem)
    {
    case MODEM_FSK:
        rssi = -(RFM96WRead(REG_RSSIVALUE) >> 1);
        break;
    case MODEM_LORA:
        if (RFM96W.Settings.Channel > RF_MID_BAND_THRESH)
        {
            rssi = RSSI_OFFSET_HF + RFM96WRead(REG_LR_RSSIVALUE);
        }
        else
        {
            rssi = RSSI_OFFSET_LF + RFM96WRead(REG_LR_RSSIVALUE);
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

void RFM96WSetOpMode(uint8_t opMode)
{

    if (opMode != RF_OPMODE_SLEEP)
    {
        RFM96WSetBoardTcxo(true);
    }
    RFM96WWrite(REG_OPMODE, (RFM96WRead(REG_OPMODE) & RF_OPMODE_MASK) | opMode);
}

void RFM96WSetModem(RadioModems_t modem)
{
    if ((RFM96WRead(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_ON) != 0)
    {
        RFM96W.Settings.Modem = MODEM_LORA;
    }
    else
    {
        RFM96W.Settings.Modem = MODEM_FSK;
    }

    if (RFM96W.Settings.Modem == modem)
    {
        return;
    }

    RFM96W.Settings.Modem = modem;
    switch (RFM96W.Settings.Modem)
    {
    default:
    case MODEM_FSK:
        RFM96WSetOpMode(RF_OPMODE_SLEEP);
        RFM96WWrite(REG_OPMODE, (RFM96WRead(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_OFF);

        RFM96WWrite(REG_DIOMAPPING1, 0x00);
        RFM96WWrite(REG_DIOMAPPING2, 0x30); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        RFM96WSetOpMode(RF_OPMODE_SLEEP);
        RFM96WWrite(REG_OPMODE, (RFM96WRead(REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);

        RFM96WWrite(REG_DIOMAPPING1, 0x00);
        RFM96WWrite(REG_DIOMAPPING2, 0x00);
        break;
    }
}

void RFM96WWrite(uint16_t addr, uint8_t data)
{
    RFM96WWriteBuffer(addr, &data, 1);
}

uint8_t RFM96WRead(uint16_t addr)
{
    uint8_t data;
    RFM96WReadBuffer(addr, &data, 1);
    return data;
}

void RFM96WWriteBuffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    //NSS = 0;
    GpioWrite(&RFM96W.Spi.Nss, 0);

    SpiInOut(&RFM96W.Spi, addr | 0x80);
    for (i = 0; i < size; i++)
    {
        SpiInOut(&RFM96W.Spi, buffer[i]);
    }

    //NSS = 1;
    GpioWrite(&RFM96W.Spi.Nss, 1);
}

void RFM96WReadBuffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    //NSS = 0;
    GpioWrite(&RFM96W.Spi.Nss, 0);

    SpiInOut(&RFM96W.Spi, addr & 0x7F);

    for (i = 0; i < size; i++)
    {
        buffer[i] = SpiInOut(&RFM96W.Spi, 0);
    }

    //NSS = 1;
    GpioWrite(&RFM96W.Spi.Nss, 1);
}

void RFM96WWriteFifo(uint8_t *buffer, uint8_t size)
{
    RFM96WWriteBuffer(0, buffer, size);
}

void RFM96WReadFifo(uint8_t *buffer, uint8_t size)
{
    RFM96WReadBuffer(0, buffer, size);
}

void RFM96WSetMaxPayloadLength(RadioModems_t modem, uint8_t max)
{
    RFM96WSetModem(modem);

    switch (modem)
    {
    case MODEM_FSK:
        if (RFM96W.Settings.Fsk.FixLen == false)
        {
            RFM96WWrite(REG_PAYLOADLENGTH, max);
        }
        break;
    case MODEM_LORA:
        RFM96WWrite(REG_LR_PAYLOADMAXLENGTH, max);
        break;
    }
}

void RFM96WSetPublicNetwork(bool enable)
{
    RFM96WSetModem(MODEM_LORA);
    RFM96W.Settings.LoRa.PublicNetwork = enable;
    if (enable == true)
    {
        // Change LoRa modem SyncWord
        RFM96WWrite(REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD);
    }
    else
    {
        // Change LoRa modem SyncWord
        RFM96WWrite(REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD);
    }
}

uint32_t RFM96WGetWakeupTime(void)
{
    return RFM96WGetBoardTcxoWakeupTime() + RADIO_WAKEUP_TIME;
}

void RFM96WOnTimeoutIrq(void *context)
{
    switch (RFM96W.Settings.State)
    {
    case RF_RX_RUNNING:
        if (RFM96W.Settings.Modem == MODEM_FSK)
        {
            RFM96W.Settings.FskPacketHandler.PreambleDetected = false;
            RFM96W.Settings.FskPacketHandler.SyncWordDetected = false;
            RFM96W.Settings.FskPacketHandler.NbBytes = 0;
            RFM96W.Settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            RFM96WWrite(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                           RF_IRQFLAGS1_PREAMBLEDETECT |
                                           RF_IRQFLAGS1_SYNCADDRESSMATCH);
            RFM96WWrite(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

            if (RFM96W.Settings.Fsk.RxContinuous == true)
            {
                // Continuous mode restart Rx chain
                RFM96WWrite(REG_RXCONFIG, RFM96WRead(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                TimerStart(&RxTimeoutSyncWord);
            }
            else
            {
                RFM96W.Settings.State = RF_IDLE;
                TimerStop(&RxTimeoutSyncWord);
            }
        }
        if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
        {
            RadioEvents->RxTimeout();
        }
        break;
    case RF_TX_RUNNING:
        // Tx timeout shouldn't happen.
        // But it has been observed that when it happens it is a result of a corrupted SPI transfer
        // it depends on the platform design.
        //
        // The workaround is to put the radio in a known state. Thus, we re-initialize it.

        // BEGIN WORKAROUND

        // Reset the radio
        RFM96WReset();

        // Calibrate Rx chain
        RxChainCalibration();

        // Initialize radio default values
        RFM96WSetOpMode(RF_OPMODE_SLEEP);

        for (uint8_t i = 0; i < sizeof(RadioRegsInit) / sizeof(RadioRegisters_t); i++)
        {
            RFM96WSetModem(RadioRegsInit[i].Modem);
            RFM96WWrite(RadioRegsInit[i].Addr, RadioRegsInit[i].Value);
        }
        RFM96WSetModem(MODEM_FSK);

        // Restore previous network type setting.
        RFM96WSetPublicNetwork(RFM96W.Settings.LoRa.PublicNetwork);
        // END WORKAROUND

        RFM96W.Settings.State = RF_IDLE;
        if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL))
        {
            RadioEvents->TxTimeout();
        }
        break;
    default:
        break;
    }
}

void RFM96WOnDio0Irq(void *context)
{
    volatile uint8_t irqFlags = 0;

    switch (RFM96W.Settings.State)
    {
    case RF_RX_RUNNING:
        //TimerStop( &RxTimeoutTimer );
        // RxDone interrupt
        switch (RFM96W.Settings.Modem)
        {
        case MODEM_FSK:
            if (RFM96W.Settings.Fsk.CrcOn == true)
            {
                irqFlags = RFM96WRead(REG_IRQFLAGS2);
                if ((irqFlags & RF_IRQFLAGS2_CRCOK) != RF_IRQFLAGS2_CRCOK)
                {
                    // Clear Irqs
                    RFM96WWrite(REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                   RF_IRQFLAGS1_PREAMBLEDETECT |
                                                   RF_IRQFLAGS1_SYNCADDRESSMATCH);
                    RFM96WWrite(REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

                    TimerStop(&RxTimeoutTimer);

                    if (RFM96W.Settings.Fsk.RxContinuous == false)
                    {
                        TimerStop(&RxTimeoutSyncWord);
                        RFM96W.Settings.State = RF_IDLE;
                    }
                    else
                    {
                        // Continuous mode restart Rx chain
                        RFM96WWrite(REG_RXCONFIG, RFM96WRead(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                        TimerStart(&RxTimeoutSyncWord);
                    }

                    if ((RadioEvents != NULL) && (RadioEvents->RxError != NULL))
                    {
                        RadioEvents->RxError();
                    }
                    RFM96W.Settings.FskPacketHandler.PreambleDetected = false;
                    RFM96W.Settings.FskPacketHandler.SyncWordDetected = false;
                    RFM96W.Settings.FskPacketHandler.NbBytes = 0;
                    RFM96W.Settings.FskPacketHandler.Size = 0;
                    break;
                }
            }

            // Read received packet size
            if ((RFM96W.Settings.FskPacketHandler.Size == 0) && (RFM96W.Settings.FskPacketHandler.NbBytes == 0))
            {
                if (RFM96W.Settings.Fsk.FixLen == false)
                {
                    RFM96WReadFifo((uint8_t *)&RFM96W.Settings.FskPacketHandler.Size, 1);
                }
                else
                {
                    RFM96W.Settings.FskPacketHandler.Size = RFM96WRead(REG_PAYLOADLENGTH);
                }
                RFM96WReadFifo(RxTxBuffer + RFM96W.Settings.FskPacketHandler.NbBytes, RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes);
                RFM96W.Settings.FskPacketHandler.NbBytes += (RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes);
            }
            else
            {
                RFM96WReadFifo(RxTxBuffer + RFM96W.Settings.FskPacketHandler.NbBytes, RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes);
                RFM96W.Settings.FskPacketHandler.NbBytes += (RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes);
            }

            TimerStop(&RxTimeoutTimer);

            if (RFM96W.Settings.Fsk.RxContinuous == false)
            {
                RFM96W.Settings.State = RF_IDLE;
                TimerStop(&RxTimeoutSyncWord);
            }
            else
            {
                // Continuous mode restart Rx chain
                RFM96WWrite(REG_RXCONFIG, RFM96WRead(REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                TimerStart(&RxTimeoutSyncWord);
            }

            if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
            {
                RadioEvents->RxDone(RxTxBuffer, RFM96W.Settings.FskPacketHandler.Size, RFM96W.Settings.FskPacketHandler.RssiValue, 0);
            }
            RFM96W.Settings.FskPacketHandler.PreambleDetected = false;
            RFM96W.Settings.FskPacketHandler.SyncWordDetected = false;
            RFM96W.Settings.FskPacketHandler.NbBytes = 0;
            RFM96W.Settings.FskPacketHandler.Size = 0;
            break;
        case MODEM_LORA:
        {
            // Clear Irq
            RFM96WWrite(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

            irqFlags = RFM96WRead(REG_LR_IRQFLAGS);
            if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR)
            {
                // Clear Irq
                RFM96WWrite(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

                if (RFM96W.Settings.LoRa.RxContinuous == false)
                {
                    RFM96W.Settings.State = RF_IDLE;
                }
                TimerStop(&RxTimeoutTimer);

                if ((RadioEvents != NULL) && (RadioEvents->RxError != NULL))
                {
                    RadioEvents->RxError();
                }
                break;
            }

            // Returns SNR value [dB] rounded to the nearest integer value
            RFM96W.Settings.LoRaPacketHandler.SnrValue = (((int8_t)RFM96WRead(REG_LR_PKTSNRVALUE)) + 2) >> 2;

            int16_t rssi = RFM96WRead(REG_LR_PKTRSSIVALUE);
            if (RFM96W.Settings.LoRaPacketHandler.SnrValue < 0)
            {
                if (RFM96W.Settings.Channel > RF_MID_BAND_THRESH)
                {
                    RFM96W.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4) +
                                                                  RFM96W.Settings.LoRaPacketHandler.SnrValue;
                }
                else
                {
                    RFM96W.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4) +
                                                                  RFM96W.Settings.LoRaPacketHandler.SnrValue;
                }
            }
            else
            {
                if (RFM96W.Settings.Channel > RF_MID_BAND_THRESH)
                {
                    RFM96W.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4);
                }
                else
                {
                    RFM96W.Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4);
                }
            }

            RFM96W.Settings.LoRaPacketHandler.Size = RFM96WRead(REG_LR_RXNBBYTES);
            RFM96WWrite(REG_LR_FIFOADDRPTR, RFM96WRead(REG_LR_FIFORXCURRENTADDR));
            RFM96WReadFifo(RxTxBuffer, RFM96W.Settings.LoRaPacketHandler.Size);

            if (RFM96W.Settings.LoRa.RxContinuous == false)
            {
                RFM96W.Settings.State = RF_IDLE;
            }
            TimerStop(&RxTimeoutTimer);

            if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
            {
                RadioEvents->RxDone(RxTxBuffer, RFM96W.Settings.LoRaPacketHandler.Size, RFM96W.Settings.LoRaPacketHandler.RssiValue, RFM96W.Settings.LoRaPacketHandler.SnrValue);
            }
        }
        break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        TimerStop(&TxTimeoutTimer);
        // TxDone interrupt
        switch (RFM96W.Settings.Modem)
        {
        case MODEM_LORA:
            // Clear Irq
            RFM96WWrite(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
            // Intentional fall through
        case MODEM_FSK:
        default:
            RFM96W.Settings.State = RF_IDLE;
            if ((RadioEvents != NULL) && (RadioEvents->TxDone != NULL))
            {
                RadioEvents->TxDone();
            }
            break;
        }
        break;
    default:
        break;
    }
}

void RFM96WOnDio1Irq(void *context)
{
    switch (RFM96W.Settings.State)
    {
    case RF_RX_RUNNING:
        switch (RFM96W.Settings.Modem)
        {
        case MODEM_FSK:
            // Stop timer
            TimerStop(&RxTimeoutSyncWord);

            // FifoLevel interrupt
            // Read received packet size
            if ((RFM96W.Settings.FskPacketHandler.Size == 0) && (RFM96W.Settings.FskPacketHandler.NbBytes == 0))
            {
                if (RFM96W.Settings.Fsk.FixLen == false)
                {
                    RFM96WReadFifo((uint8_t *)&RFM96W.Settings.FskPacketHandler.Size, 1);
                }
                else
                {
                    RFM96W.Settings.FskPacketHandler.Size = RFM96WRead(REG_PAYLOADLENGTH);
                }
            }

            // ERRATA 3.1 - PayloadReady Set for 31.25ns if FIFO is Empty
            //
            //              When FifoLevel interrupt is used to offload the
            //              FIFO, the microcontroller should  monitor  both
            //              PayloadReady  and FifoLevel interrupts, and
            //              read only (FifoThreshold-1) bytes off the FIFO
            //              when FifoLevel fires
            if ((RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes) >= RFM96W.Settings.FskPacketHandler.FifoThresh)
            {
                RFM96WReadFifo((RxTxBuffer + RFM96W.Settings.FskPacketHandler.NbBytes), RFM96W.Settings.FskPacketHandler.FifoThresh - 1);
                RFM96W.Settings.FskPacketHandler.NbBytes += RFM96W.Settings.FskPacketHandler.FifoThresh - 1;
            }
            else
            {
                RFM96WReadFifo((RxTxBuffer + RFM96W.Settings.FskPacketHandler.NbBytes), RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes);
                RFM96W.Settings.FskPacketHandler.NbBytes += (RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes);
            }
            break;
        case MODEM_LORA:
            // Sync time out
            TimerStop(&RxTimeoutTimer);
            // Clear Irq
            RFM96WWrite(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT);

            RFM96W.Settings.State = RF_IDLE;
            if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
            {
                RadioEvents->RxTimeout();
            }
            break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        switch (RFM96W.Settings.Modem)
        {
        case MODEM_FSK:
            // FifoEmpty interrupt
            if ((RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes) > RFM96W.Settings.FskPacketHandler.ChunkSize)
            {
                RFM96WWriteFifo((RxTxBuffer + RFM96W.Settings.FskPacketHandler.NbBytes), RFM96W.Settings.FskPacketHandler.ChunkSize);
                RFM96W.Settings.FskPacketHandler.NbBytes += RFM96W.Settings.FskPacketHandler.ChunkSize;
            }
            else
            {
                // Write the last chunk of data
                RFM96WWriteFifo(RxTxBuffer + RFM96W.Settings.FskPacketHandler.NbBytes, RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes);
                RFM96W.Settings.FskPacketHandler.NbBytes += RFM96W.Settings.FskPacketHandler.Size - RFM96W.Settings.FskPacketHandler.NbBytes;
            }
            break;
        case MODEM_LORA:
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void RFM96WOnDio2Irq(void *context)
{
    switch (RFM96W.Settings.State)
    {
    case RF_RX_RUNNING:
        switch (RFM96W.Settings.Modem)
        {
        case MODEM_FSK:
            // Checks if DIO4 is connected. If it is not PreambleDetected is set to true.
            if (RFM96W.DIO4.port == NULL)
            {
                RFM96W.Settings.FskPacketHandler.PreambleDetected = true;
            }

            if ((RFM96W.Settings.FskPacketHandler.PreambleDetected == true) && (RFM96W.Settings.FskPacketHandler.SyncWordDetected == false))
            {
                TimerStop(&RxTimeoutSyncWord);

                RFM96W.Settings.FskPacketHandler.SyncWordDetected = true;

                RFM96W.Settings.FskPacketHandler.RssiValue = -(RFM96WRead(REG_RSSIVALUE) >> 1);

                RFM96W.Settings.FskPacketHandler.AfcValue = (int32_t)(double)(((uint16_t)RFM96WRead(REG_AFCMSB) << 8) |
                                                                              (uint16_t)RFM96WRead(REG_AFCLSB)) *
                                                            (double)FREQ_STEP;
                RFM96W.Settings.FskPacketHandler.RxGain = (RFM96WRead(REG_LNA) >> 5) & 0x07;
            }
            break;
        case MODEM_LORA:
            if (RFM96W.Settings.LoRa.FreqHopOn == true)
            {
                // Clear Irq
                RFM96WWrite(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                if ((RadioEvents != NULL) && (RadioEvents->FhssChangeChannel != NULL))
                {
                    RadioEvents->FhssChangeChannel((RFM96WRead(REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                }
            }
            break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        switch (RFM96W.Settings.Modem)
        {
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            if (RFM96W.Settings.LoRa.FreqHopOn == true)
            {
                // Clear Irq
                RFM96WWrite(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                if ((RadioEvents != NULL) && (RadioEvents->FhssChangeChannel != NULL))
                {
                    RadioEvents->FhssChangeChannel((RFM96WRead(REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                }
            }
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void RFM96WOnDio3Irq(void *context)
{
    switch (RFM96W.Settings.Modem)
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if ((RFM96WRead(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED) == RFLR_IRQFLAGS_CADDETECTED)
        {
            // Clear Irq
            RFM96WWrite(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
            if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL))
            {
                RadioEvents->CadDone(true);
            }
        }
        else
        {
            // Clear Irq
            RFM96WWrite(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE);
            if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL))
            {
                RadioEvents->CadDone(false);
            }
        }
        break;
    default:
        break;
    }
}

void RFM96WOnDio4Irq(void *context)
{
    switch (RFM96W.Settings.Modem)
    {
    case MODEM_FSK:
    {
        if (RFM96W.Settings.FskPacketHandler.PreambleDetected == false)
        {
            RFM96W.Settings.FskPacketHandler.PreambleDetected = true;
        }
    }
    break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

void RFM96WOnDio5Irq(void *context)
{
    switch (RFM96W.Settings.Modem)
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}
