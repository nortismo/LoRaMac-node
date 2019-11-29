/*!
 * \file      multiRfm96w.c
 *
 * \brief     Target board RFM96W driver implementation for
 *            multiple transceivers in parallel.
 *            Highly derived from SX1276 driver
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
#include "delay.h"
#include "multiRfm96w.h"
#include "multiRfm96w-board.h"

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
static void RxChainCalibration(RadioIndex_t radio);

static void InitTimers(RadioIndex_t radio);
static void Radio0TimeoutIrq(void *context);
static void Radio1TimeoutIrq(void *context);
static void Radio2TimeoutIrq(void *context);
static void Radio3TimeoutIrq(void *context);
static void Radio4TimeoutIrq(void *context);
static void Radio5TimeoutIrq(void *context);
static void Radio6TimeoutIrq(void *context);
static void Radio7TimeoutIrq(void *context);

static void Radio0OnDio0Irq(void *context);
static void Radio1OnDio0Irq(void *context);
static void Radio2OnDio0Irq(void *context);
static void Radio3OnDio0Irq(void *context);
static void Radio4OnDio0Irq(void *context);
static void Radio5OnDio0Irq(void *context);
static void Radio6OnDio0Irq(void *context);
static void Radio7OnDio0Irq(void *context);

static void Radio0OnDio1Irq(void *context);
static void Radio1OnDio1Irq(void *context);
static void Radio2OnDio1Irq(void *context);
static void Radio3OnDio1Irq(void *context);
static void Radio4OnDio1Irq(void *context);
static void Radio5OnDio1Irq(void *context);
static void Radio6OnDio1Irq(void *context);
static void Radio7OnDio1Irq(void *context);

static void Radio0OnDio2Irq(void *context);
static void Radio1OnDio2Irq(void *context);
static void Radio2OnDio2Irq(void *context);
static void Radio3OnDio2Irq(void *context);
static void Radio4OnDio2Irq(void *context);
static void Radio5OnDio2Irq(void *context);
static void Radio6OnDio2Irq(void *context);
static void Radio7OnDio2Irq(void *context);

static void Radio0OnDio3Irq(void *context);
static void Radio1OnDio3Irq(void *context);
static void Radio2OnDio3Irq(void *context);
static void Radio3OnDio3Irq(void *context);
static void Radio4OnDio3Irq(void *context);
static void Radio5OnDio3Irq(void *context);
static void Radio6OnDio3Irq(void *context);
static void Radio7OnDio3Irq(void *context);

static void Radio0OnDio4Irq(void *context);
static void Radio1OnDio4Irq(void *context);
static void Radio2OnDio4Irq(void *context);
static void Radio3OnDio4Irq(void *context);
static void Radio4OnDio4Irq(void *context);
static void Radio5OnDio4Irq(void *context);
static void Radio6OnDio4Irq(void *context);
static void Radio7OnDio4Irq(void *context);

static void Radio0OnDio5Irq(void *context);
static void Radio1OnDio5Irq(void *context);
static void Radio2OnDio5Irq(void *context);
static void Radio3OnDio5Irq(void *context);
static void Radio4OnDio5Irq(void *context);
static void Radio5OnDio5Irq(void *context);
static void Radio6OnDio5Irq(void *context);
static void Radio7OnDio5Irq(void *context);

/*!
 * \brief Sets the MULTIRFM96W in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
void MULTIRFM96WSetTx(RadioIndex_t radio, uint32_t timeout);

/*!
 * \brief Writes the buffer contents to the MULTIRFM96W FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void MULTIRFM96WWriteFifo(RadioIndex_t radio, uint8_t *buffer, uint8_t size);

/*!
 * \brief Reads the contents of the MULTIRFM96W FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void MULTIRFM96WReadFifo(RadioIndex_t radio, uint8_t *buffer, uint8_t size);

/*!
 * \brief Sets the MULTIRFM96W operating mode
 *
 * \param [IN] opMode New operating mode
 */
void MULTIRFM96WSetOpMode(RadioIndex_t radio, uint8_t opMode);

/*
 * MULTIRFM96W DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void MULTIRFM96WOnDio0Irq(RadioIndex_t radio, void *context);

/*!
 * \brief DIO 1 IRQ callback
 */
void MULTIRFM96WOnDio1Irq(RadioIndex_t radio, void *context);

/*!
 * \brief DIO 2 IRQ callback
 */
void MULTIRFM96WOnDio2Irq(RadioIndex_t radio, void *context);

/*!
 * \brief DIO 3 IRQ callback
 */
void MULTIRFM96WOnDio3Irq(RadioIndex_t radio, void *context);

/*!
 * \brief DIO 4 IRQ callback
 */
void MULTIRFM96WOnDio4Irq(RadioIndex_t radio, void *context);

/*!
 * \brief DIO 5 IRQ callback
 */
void MULTIRFM96WOnDio5Irq(RadioIndex_t radio, void *context);

/*!
 * \brief Tx & Rx timeout timer callback
 */
void MULTIRFM96WOnTimeoutIrq(RadioIndex_t radio, void *context);

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in MULTIRFM96W-board.h file
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
static uint8_t RxTxBuffer[NUMBER_OF_RADIOS][RX_BUFFER_SIZE];

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
MULTIRFM96W_t MULTIRFM96W[NUMBER_OF_RADIOS];

/*!
 * Hardware DIO IRQ callback initialization
 */
DioIrqHandler *DioIrq[] = {
    Radio0OnDio0Irq,
    Radio0OnDio1Irq,
    Radio0OnDio2Irq,
    Radio0OnDio3Irq,
    Radio0OnDio4Irq,
    Radio0OnDio5Irq,
    Radio1OnDio0Irq, //+6
    Radio1OnDio1Irq,
    Radio1OnDio2Irq,
    Radio1OnDio3Irq,
    Radio1OnDio4Irq,
    Radio1OnDio5Irq,
    Radio2OnDio0Irq, //+12
    Radio2OnDio1Irq,
    Radio2OnDio2Irq,
    Radio2OnDio3Irq,
    Radio2OnDio4Irq,
    Radio2OnDio5Irq,
    Radio3OnDio0Irq, //+18
    Radio3OnDio1Irq,
    Radio3OnDio2Irq,
    Radio3OnDio3Irq,
    Radio3OnDio4Irq,
    Radio3OnDio5Irq,
    Radio4OnDio0Irq, //+24
    Radio4OnDio1Irq,
    Radio4OnDio2Irq,
    Radio4OnDio3Irq,
    Radio4OnDio4Irq,
    Radio4OnDio5Irq,
    Radio5OnDio0Irq, //+30
    Radio5OnDio1Irq,
    Radio5OnDio2Irq,
    Radio5OnDio3Irq,
    Radio5OnDio4Irq,
    Radio5OnDio5Irq,
    Radio6OnDio0Irq, //+36
    Radio6OnDio1Irq,
    Radio6OnDio2Irq,
    Radio6OnDio3Irq,
    Radio6OnDio4Irq,
    Radio6OnDio5Irq,
    Radio7OnDio0Irq, //+42
    Radio7OnDio1Irq,
    Radio7OnDio2Irq,
    Radio7OnDio3Irq,
    Radio7OnDio4Irq,
    Radio7OnDio5Irq,

};

/*!
 * Tx and Rx timers
 */
TimerEvent_t TxTimeoutTimers[8];
TimerEvent_t RxTimeoutTimers[8];
TimerEvent_t RxTimeoutSyncWords[8];

/*
 * Radio driver functions implementation
 */

void MULTIRFM96WInit(RadioIndex_t radio, RadioEvents_t *events)
{
    uint8_t i;

    RadioEvents = events;

    // Initialize driver timeout timers
    InitTimers(radio);

    MULTIRFM96WReset(radio);

    RxChainCalibration(radio);

    //Disable High Power (Pmax to 17dBm instead of 20dBm)
    MULTIRFM96WWrite(radio, REG_LR_PADAC, (MULTIRFM96WRead(radio, REG_LR_PADAC) & RFLR_PADAC_20DBM_MASK | RFLR_PADAC_20DBM_OFF));

    MULTIRFM96WSetOpMode(radio, RF_OPMODE_SLEEP);

    MULTIRFM96WIoIrqInit(DioIrq);

    for (i = 0; i < sizeof(RadioRegsInit) / sizeof(RadioRegisters_t); i++)
    {
        MULTIRFM96WSetModem(radio, RadioRegsInit[i].Modem);
        MULTIRFM96WWrite(radio, RadioRegsInit[i].Addr, RadioRegsInit[i].Value);
    }

    MULTIRFM96WSetModem(radio, MODEM_FSK);

    //Enable low frequency
    MULTIRFM96WWrite(radio, REG_OPMODE, (MULTIRFM96WRead(radio, REG_OPMODE) & RFLR_OPMODE_FREQMODE_ACCESS_MASK | RFLR_OPMODE_FREQMODE_ACCESS_LF));

    //LNA gain set by the internal AGC loop
    MULTIRFM96WWrite(radio, REG_LR_MODEMCONFIG3, (MULTIRFM96WRead(radio, REG_LR_MODEMCONFIG3) & RFLR_MODEMCONFIG3_AGCAUTO_MASK | RFLR_MODEMCONFIG3_AGCAUTO_ON));

    //Reset FIFOTX Pointer
    MULTIRFM96WWrite(radio, REG_LR_FIFOTXBASEADDR, 0);

    //Reset FIFO ADDR Pointer
    MULTIRFM96WWrite(radio, REG_LR_FIFOADDRPTR, 0);

    //Reset IRQ Flags
    MULTIRFM96WWrite(radio, REG_LR_IRQFLAGS, 0xFF);

    //Reset PaRamp (Modulation shaping)
    MULTIRFM96WWrite(radio, REG_LR_PARAMP, 0x08);

    MULTIRFM96W[radio].Settings.State = RF_IDLE;
}

RadioState_t MULTIRFM96WGetStatus(RadioIndex_t radio)
{
    return MULTIRFM96W[radio].Settings.State;
}

void MULTIRFM96WSetChannel(RadioIndex_t radio, uint32_t freq)
{
    MULTIRFM96W[radio].Settings.Channel = freq;
    freq = (uint32_t)((double)freq / (double)FREQ_STEP);
    MULTIRFM96WWrite(radio, REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
    MULTIRFM96WWrite(radio, REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
    MULTIRFM96WWrite(radio, REG_FRFLSB, (uint8_t)(freq & 0xFF));
}

bool MULTIRFM96WIsChannelFree(RadioIndex_t radio, RadioModems_t modem, uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime)
{
    bool status = true;
    int16_t rssi = 0;
    uint32_t carrierSenseTime = 0;

    MULTIRFM96WSetSleep(radio);

    MULTIRFM96WSetModem(radio, modem);

    MULTIRFM96WSetChannel(radio, freq);

    MULTIRFM96WSetOpMode(radio, RF_OPMODE_RECEIVER);

    DelayMs(1);

    carrierSenseTime = TimerGetCurrentTime();

    // Perform carrier sense for maxCarrierSenseTime
    while (TimerGetElapsedTime(carrierSenseTime) < maxCarrierSenseTime)
    {
        rssi = MULTIRFM96WReadRssi(radio, modem);

        if (rssi > rssiThresh)
        {
            status = false;
            break;
        }
    }
    MULTIRFM96WSetSleep(radio);
    return status;
}

uint32_t MULTIRFM96WRandom(RadioIndex_t radio)
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    MULTIRFM96WSetModem(radio, MODEM_LORA);

    // Disable LoRa modem interrupts
    MULTIRFM96WWrite(radio, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED);

    // Set radio in continuous reception
    MULTIRFM96WSetOpMode(radio, RF_OPMODE_RECEIVER);

    for (i = 0; i < 32; i++)
    {
        DelayMs(1);
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ((uint32_t)MULTIRFM96WRead(radio, REG_LR_RSSIWIDEBAND) & 0x01) << i;
    }

    MULTIRFM96WSetSleep(radio);

    return rnd;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration(RadioIndex_t radio)
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = MULTIRFM96WRead(radio, REG_PACONFIG);
    initialFreq = (double)(((uint32_t)MULTIRFM96WRead(radio, REG_FRFMSB) << 16) |
                           ((uint32_t)MULTIRFM96WRead(radio, REG_FRFMID) << 8) |
                           ((uint32_t)MULTIRFM96WRead(radio, REG_FRFLSB))) *
                  (double)FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    MULTIRFM96WWrite(radio, REG_PACONFIG, 0x00);

    // Launch Rx chain calibration for LF band
    MULTIRFM96WWrite(radio, REG_IMAGECAL, (MULTIRFM96WRead(radio, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while ((MULTIRFM96WRead(radio, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
    {
    }

    // Sets a Frequency in HF band
    MULTIRFM96WSetChannel(radio, 868000000);

    // Launch Rx chain calibration for HF band
    MULTIRFM96WWrite(radio, REG_IMAGECAL, (MULTIRFM96WRead(radio, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while ((MULTIRFM96WRead(radio, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
    {
    }

    // Restore context
    MULTIRFM96WWrite(radio, REG_PACONFIG, regPaConfigInitVal);
    MULTIRFM96WSetChannel(radio, initialFreq);
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t GetFskBandwidthRegValue(RadioIndex_t radio, uint32_t bandwidth)
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

void MULTIRFM96WSetRxConfig(RadioIndex_t radio, RadioModems_t modem, uint32_t bandwidth,
                            uint32_t datarate, uint8_t coderate,
                            uint32_t bandwidthAfc, uint16_t preambleLen,
                            uint16_t symbTimeout, bool fixLen,
                            uint8_t payloadLen,
                            bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                            bool iqInverted, bool rxContinuous)
{
    MULTIRFM96WSetModem(radio, modem);

    switch (modem)
    {
    case MODEM_FSK:
    {
        MULTIRFM96W[radio].Settings.Fsk.Bandwidth = bandwidth;
        MULTIRFM96W[radio].Settings.Fsk.Datarate = datarate;
        MULTIRFM96W[radio].Settings.Fsk.BandwidthAfc = bandwidthAfc;
        MULTIRFM96W[radio].Settings.Fsk.FixLen = fixLen;
        MULTIRFM96W[radio].Settings.Fsk.PayloadLen = payloadLen;
        MULTIRFM96W[radio].Settings.Fsk.CrcOn = crcOn;
        MULTIRFM96W[radio].Settings.Fsk.IqInverted = iqInverted;
        MULTIRFM96W[radio].Settings.Fsk.RxContinuous = rxContinuous;
        MULTIRFM96W[radio].Settings.Fsk.PreambleLen = preambleLen;
        MULTIRFM96W[radio].Settings.Fsk.RxSingleTimeout = (uint32_t)(symbTimeout * ((1.0 / (double)datarate) * 8.0) * 1000);

        datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);
        MULTIRFM96WWrite(radio, REG_BITRATEMSB, (uint8_t)(datarate >> 8));
        MULTIRFM96WWrite(radio, REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

        MULTIRFM96WWrite(radio, REG_RXBW, GetFskBandwidthRegValue(radio, bandwidth));
        MULTIRFM96WWrite(radio, REG_AFCBW, GetFskBandwidthRegValue(radio, bandwidthAfc));

        MULTIRFM96WWrite(radio, REG_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
        MULTIRFM96WWrite(radio, REG_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

        if (fixLen == 1)
        {
            MULTIRFM96WWrite(radio, REG_PAYLOADLENGTH, payloadLen);
        }
        else
        {
            MULTIRFM96WWrite(radio, REG_PAYLOADLENGTH, 0xFF); // Set payload length to the maximum
        }

        MULTIRFM96WWrite(radio, REG_PACKETCONFIG1,
                         (MULTIRFM96WRead(radio, REG_PACKETCONFIG1) &
                          RF_PACKETCONFIG1_CRC_MASK &
                          RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                             ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                             (crcOn << 4));
        MULTIRFM96WWrite(radio, REG_PACKETCONFIG2, (MULTIRFM96WRead(radio, REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
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
        MULTIRFM96W[radio].Settings.LoRa.Bandwidth = bandwidth;
        MULTIRFM96W[radio].Settings.LoRa.Datarate = datarate;
        MULTIRFM96W[radio].Settings.LoRa.Coderate = coderate;
        MULTIRFM96W[radio].Settings.LoRa.PreambleLen = preambleLen;
        MULTIRFM96W[radio].Settings.LoRa.FixLen = fixLen;
        MULTIRFM96W[radio].Settings.LoRa.PayloadLen = payloadLen;
        MULTIRFM96W[radio].Settings.LoRa.CrcOn = crcOn;
        MULTIRFM96W[radio].Settings.LoRa.FreqHopOn = freqHopOn;
        MULTIRFM96W[radio].Settings.LoRa.HopPeriod = hopPeriod;
        MULTIRFM96W[radio].Settings.LoRa.IqInverted = iqInverted;
        MULTIRFM96W[radio].Settings.LoRa.RxContinuous = rxContinuous;

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
            MULTIRFM96W[radio].Settings.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            MULTIRFM96W[radio].Settings.LoRa.LowDatarateOptimize = 0x00;
        }

        MULTIRFM96WWrite(radio, REG_LR_MODEMCONFIG1,
                         (MULTIRFM96WRead(radio, REG_LR_MODEMCONFIG1) &
                          RFLR_MODEMCONFIG1_BW_MASK &
                          RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                          RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                             (bandwidth << 4) | (coderate << 1) |
                             fixLen);

        MULTIRFM96WWrite(radio, REG_LR_MODEMCONFIG2,
                         (MULTIRFM96WRead(radio, REG_LR_MODEMCONFIG2) &
                          RFLR_MODEMCONFIG2_SF_MASK &
                          RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                          RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) |
                             (datarate << 4) | (crcOn << 2) |
                             ((symbTimeout >> 8) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

        MULTIRFM96WWrite(radio, REG_LR_MODEMCONFIG3,
                         (MULTIRFM96WRead(radio, REG_LR_MODEMCONFIG3) &
                          RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                             (MULTIRFM96W[radio].Settings.LoRa.LowDatarateOptimize << 3));

        MULTIRFM96WWrite(radio, REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symbTimeout & 0xFF));

        MULTIRFM96WWrite(radio, REG_LR_PREAMBLEMSB, (uint8_t)((preambleLen >> 8) & 0xFF));
        MULTIRFM96WWrite(radio, REG_LR_PREAMBLELSB, (uint8_t)(preambleLen & 0xFF));

        if (fixLen == 1)
        {
            MULTIRFM96WWrite(radio, REG_LR_PAYLOADLENGTH, payloadLen);
        }

        if (MULTIRFM96W[radio].Settings.LoRa.FreqHopOn == true)
        {
            MULTIRFM96WWrite(radio, REG_LR_PLLHOP, (MULTIRFM96WRead(radio, REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
            MULTIRFM96WWrite(radio, REG_LR_HOPPERIOD, MULTIRFM96W[radio].Settings.LoRa.HopPeriod);
        }

        if ((bandwidth == 9) && (MULTIRFM96W[radio].Settings.Channel > RF_MID_BAND_THRESH))
        {
            // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
            MULTIRFM96WWrite(radio, REG_LR_HIGHBWOPTIMIZE1, 0x02);
            MULTIRFM96WWrite(radio, REG_LR_HIGHBWOPTIMIZE2, 0x64);
        }
        else if (bandwidth == 9)
        {
            // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
            MULTIRFM96WWrite(radio, REG_LR_HIGHBWOPTIMIZE1, 0x02);
            MULTIRFM96WWrite(radio, REG_LR_HIGHBWOPTIMIZE2, 0x7F);
        }
        else
        {
            // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
            MULTIRFM96WWrite(radio, REG_LR_HIGHBWOPTIMIZE1, 0x03);
        }

        if (datarate == 6)
        {
            MULTIRFM96WWrite(radio, REG_LR_DETECTOPTIMIZE,
                             (MULTIRFM96WRead(radio, REG_LR_DETECTOPTIMIZE) &
                              RFLR_DETECTIONOPTIMIZE_MASK) |
                                 RFLR_DETECTIONOPTIMIZE_SF6);
            MULTIRFM96WWrite(radio, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6);
        }
        else
        {
            MULTIRFM96WWrite(radio, REG_LR_DETECTOPTIMIZE,
                             (MULTIRFM96WRead(radio, REG_LR_DETECTOPTIMIZE) &
                              RFLR_DETECTIONOPTIMIZE_MASK) |
                                 RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
            MULTIRFM96WWrite(radio, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12);
        }
    }
    break;
    }
}

void MULTIRFM96WSetTxConfig(RadioIndex_t radio, RadioModems_t modem, int8_t power, uint32_t fdev,
                            uint32_t bandwidth, uint32_t datarate,
                            uint8_t coderate, uint16_t preambleLen,
                            bool fixLen, bool crcOn, bool freqHopOn,
                            uint8_t hopPeriod, bool iqInverted, uint32_t timeout)
{
    MULTIRFM96WSetModem(radio, modem);

    MULTIRFM96WSetRfTxPower(radio, power);

    switch (modem)
    {
    case MODEM_FSK:
    {
        MULTIRFM96W[radio].Settings.Fsk.Power = power;
        MULTIRFM96W[radio].Settings.Fsk.Fdev = fdev;
        MULTIRFM96W[radio].Settings.Fsk.Bandwidth = bandwidth;
        MULTIRFM96W[radio].Settings.Fsk.Datarate = datarate;
        MULTIRFM96W[radio].Settings.Fsk.PreambleLen = preambleLen;
        MULTIRFM96W[radio].Settings.Fsk.FixLen = fixLen;
        MULTIRFM96W[radio].Settings.Fsk.CrcOn = crcOn;
        MULTIRFM96W[radio].Settings.Fsk.IqInverted = iqInverted;
        MULTIRFM96W[radio].Settings.Fsk.TxTimeout = timeout;

        fdev = (uint16_t)((double)fdev / (double)FREQ_STEP);
        MULTIRFM96WWrite(radio, REG_FDEVMSB, (uint8_t)(fdev >> 8));
        MULTIRFM96WWrite(radio, REG_FDEVLSB, (uint8_t)(fdev & 0xFF));

        datarate = (uint16_t)((double)XTAL_FREQ / (double)datarate);
        MULTIRFM96WWrite(radio, REG_BITRATEMSB, (uint8_t)(datarate >> 8));
        MULTIRFM96WWrite(radio, REG_BITRATELSB, (uint8_t)(datarate & 0xFF));

        MULTIRFM96WWrite(radio, REG_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
        MULTIRFM96WWrite(radio, REG_PREAMBLELSB, preambleLen & 0xFF);

        MULTIRFM96WWrite(radio, REG_PACKETCONFIG1,
                         (MULTIRFM96WRead(radio, REG_PACKETCONFIG1) &
                          RF_PACKETCONFIG1_CRC_MASK &
                          RF_PACKETCONFIG1_PACKETFORMAT_MASK) |
                             ((fixLen == 1) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE) |
                             (crcOn << 4));
        MULTIRFM96WWrite(radio, REG_PACKETCONFIG2, (MULTIRFM96WRead(radio, REG_PACKETCONFIG2) | RF_PACKETCONFIG2_DATAMODE_PACKET));
    }
    break;
    case MODEM_LORA:
    {
        MULTIRFM96W[radio].Settings.LoRa.Power = power;
        if (bandwidth > 2)
        {
            // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            while (1)
                ;
        }
        bandwidth += 7;
        MULTIRFM96W[radio].Settings.LoRa.Bandwidth = bandwidth;
        MULTIRFM96W[radio].Settings.LoRa.Datarate = datarate;
        MULTIRFM96W[radio].Settings.LoRa.Coderate = coderate;
        MULTIRFM96W[radio].Settings.LoRa.PreambleLen = preambleLen;
        MULTIRFM96W[radio].Settings.LoRa.FixLen = fixLen;
        MULTIRFM96W[radio].Settings.LoRa.FreqHopOn = freqHopOn;
        MULTIRFM96W[radio].Settings.LoRa.HopPeriod = hopPeriod;
        MULTIRFM96W[radio].Settings.LoRa.CrcOn = crcOn;
        MULTIRFM96W[radio].Settings.LoRa.IqInverted = iqInverted;
        MULTIRFM96W[radio].Settings.LoRa.TxTimeout = timeout;

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
            MULTIRFM96W[radio].Settings.LoRa.LowDatarateOptimize = 0x01;
        }
        else
        {
            MULTIRFM96W[radio].Settings.LoRa.LowDatarateOptimize = 0x00;
        }

        if (MULTIRFM96W[radio].Settings.LoRa.FreqHopOn == true)
        {
            MULTIRFM96WWrite(radio, REG_LR_PLLHOP, (MULTIRFM96WRead(radio, REG_LR_PLLHOP) & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
            MULTIRFM96WWrite(radio, REG_LR_HOPPERIOD, MULTIRFM96W[radio].Settings.LoRa.HopPeriod);
        }

        MULTIRFM96WWrite(radio, REG_LR_MODEMCONFIG1,
                         (MULTIRFM96WRead(radio, REG_LR_MODEMCONFIG1) &
                          RFLR_MODEMCONFIG1_BW_MASK &
                          RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                          RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) |
                             (bandwidth << 4) | (coderate << 1) |
                             fixLen);

        MULTIRFM96WWrite(radio, REG_LR_MODEMCONFIG2,
                         (MULTIRFM96WRead(radio, REG_LR_MODEMCONFIG2) &
                          RFLR_MODEMCONFIG2_SF_MASK &
                          RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) |
                             (datarate << 4) | (crcOn << 2));

        MULTIRFM96WWrite(radio, REG_LR_MODEMCONFIG3,
                         (MULTIRFM96WRead(radio, REG_LR_MODEMCONFIG3) &
                          RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK) |
                             (MULTIRFM96W[radio].Settings.LoRa.LowDatarateOptimize << 3));

        MULTIRFM96WWrite(radio, REG_LR_PREAMBLEMSB, (preambleLen >> 8) & 0x00FF);
        MULTIRFM96WWrite(radio, REG_LR_PREAMBLELSB, preambleLen & 0xFF);

        if (datarate == 6)
        {
            MULTIRFM96WWrite(radio, REG_LR_DETECTOPTIMIZE,
                             (MULTIRFM96WRead(radio, REG_LR_DETECTOPTIMIZE) &
                              RFLR_DETECTIONOPTIMIZE_MASK) |
                                 RFLR_DETECTIONOPTIMIZE_SF6);
            MULTIRFM96WWrite(radio, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6);
        }
        else
        {
            MULTIRFM96WWrite(radio, REG_LR_DETECTOPTIMIZE,
                             (MULTIRFM96WRead(radio, REG_LR_DETECTOPTIMIZE) &
                              RFLR_DETECTIONOPTIMIZE_MASK) |
                                 RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
            MULTIRFM96WWrite(radio, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12);
        }
    }
    break;
    }
}

uint32_t MULTIRFM96WGetTimeOnAir(RadioIndex_t radio, RadioModems_t modem, uint8_t pktLen)
{
    uint32_t airTime = 0;

    switch (modem)
    {
    case MODEM_FSK:
    {
        airTime = round((8 * (MULTIRFM96W[radio].Settings.Fsk.PreambleLen + ((MULTIRFM96WRead(radio, REG_SYNCCONFIG) & ~RF_SYNCCONFIG_SYNCSIZE_MASK) + 1) + ((MULTIRFM96W[radio].Settings.Fsk.FixLen == 0x01) ? 0.0 : 1.0) + (((MULTIRFM96WRead(radio, REG_PACKETCONFIG1) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK) != 0x00) ? 1.0 : 0) + pktLen + ((MULTIRFM96W[radio].Settings.Fsk.CrcOn == 0x01) ? 2.0 : 0)) /
                         MULTIRFM96W[radio].Settings.Fsk.Datarate) *
                        1000);
    }
    break;
    case MODEM_LORA:
    {
        double bw = 0.0;
        // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
        switch (MULTIRFM96W[radio].Settings.LoRa.Bandwidth)
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
        double rs = bw / (1 << MULTIRFM96W[radio].Settings.LoRa.Datarate);
        double ts = 1 / rs;
        // time of preamble
        double tPreamble = (MULTIRFM96W[radio].Settings.LoRa.PreambleLen + 4.25) * ts;
        // Symbol length of payload and time
        double tmp = ceil((8 * pktLen - 4 * MULTIRFM96W[radio].Settings.LoRa.Datarate +
                           28 + 16 * MULTIRFM96W[radio].Settings.LoRa.CrcOn -
                           (MULTIRFM96W[radio].Settings.LoRa.FixLen ? 20 : 0)) /
                          (double)(4 * (MULTIRFM96W[radio].Settings.LoRa.Datarate -
                                        ((MULTIRFM96W[radio].Settings.LoRa.LowDatarateOptimize > 0) ? 2 : 0)))) *
                     (MULTIRFM96W[radio].Settings.LoRa.Coderate + 4);
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

void MULTIRFM96WSend(RadioIndex_t radio, uint8_t *buffer, uint8_t size)
{
    uint32_t txTimeout = 0;

    switch (MULTIRFM96W[radio].Settings.Modem)
    {
    case MODEM_FSK:
    {
        MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes = 0;
        MULTIRFM96W[radio].Settings.FskPacketHandler.Size = size;

        if (MULTIRFM96W[radio].Settings.Fsk.FixLen == false)
        {
            MULTIRFM96WWriteFifo(radio, (uint8_t *)&size, 1);
        }
        else
        {
            MULTIRFM96WWrite(radio, REG_PAYLOADLENGTH, size);
        }

        if ((size > 0) && (size <= 64))
        {
            MULTIRFM96W[radio].Settings.FskPacketHandler.ChunkSize = size;
        }
        else
        {
            memcpy1(RxTxBuffer[radio], buffer, size);
            MULTIRFM96W[radio].Settings.FskPacketHandler.ChunkSize = 32;
        }

        // Write payload buffer
        MULTIRFM96WWriteFifo(radio, buffer, MULTIRFM96W[radio].Settings.FskPacketHandler.ChunkSize);
        MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes += MULTIRFM96W[radio].Settings.FskPacketHandler.ChunkSize;
        txTimeout = MULTIRFM96W[radio].Settings.Fsk.TxTimeout;
    }
    break;
    case MODEM_LORA:
    {
        if (MULTIRFM96W[radio].Settings.LoRa.IqInverted == true)
        {
            MULTIRFM96WWrite(radio, REG_LR_INVERTIQ, ((MULTIRFM96WRead(radio, REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON));
            MULTIRFM96WWrite(radio, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
        }
        else
        {
            MULTIRFM96WWrite(radio, REG_LR_INVERTIQ, ((MULTIRFM96WRead(radio, REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
            MULTIRFM96WWrite(radio, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
        }

        MULTIRFM96W[radio].Settings.LoRaPacketHandler.Size = size;

        // Initializes the payload size
        MULTIRFM96WWrite(radio, REG_LR_PAYLOADLENGTH, size);

        // Full buffer used for Tx
        MULTIRFM96WWrite(radio, REG_LR_FIFOTXBASEADDR, 0);
        MULTIRFM96WWrite(radio, REG_LR_FIFOADDRPTR, 0);

        // FIFO operations can not take place in Sleep mode
        if ((MULTIRFM96WRead(radio, REG_OPMODE) & ~RF_OPMODE_MASK) == RF_OPMODE_SLEEP)
        {
            MULTIRFM96WSetStby(radio);
            DelayMs(1);
        }
        // Write payload buffer
        MULTIRFM96WWriteFifo(radio, buffer, size);
        txTimeout = MULTIRFM96W[radio].Settings.LoRa.TxTimeout;
    }
    break;
    }

    MULTIRFM96WSetTx(radio, txTimeout);
}

void MULTIRFM96WSetSleep(RadioIndex_t radio)
{
    TimerStop(&RxTimeoutTimers[radio]);
    TimerStop(&TxTimeoutTimers[radio]);
    TimerStop(&RxTimeoutSyncWords[radio]);

    MULTIRFM96WSetOpMode(radio, RF_OPMODE_SLEEP);

    // Disable TCXO radio is in SLEEP mode
    MULTIRFM96WSetBoardTcxo(radio, false);

    MULTIRFM96W[radio].Settings.State = RF_IDLE;
}

void MULTIRFM96WSetStby(RadioIndex_t radio)
{
    TimerStop(&RxTimeoutTimers[radio]);
    TimerStop(&TxTimeoutTimers[radio]);
    TimerStop(&RxTimeoutSyncWords[radio]);

    MULTIRFM96WSetOpMode(radio, RF_OPMODE_STANDBY);
    MULTIRFM96W[radio].Settings.State = RF_IDLE;
}

void MULTIRFM96WSetRx(RadioIndex_t radio, uint32_t timeout)
{
    bool rxContinuous = false;
    TimerStop(&TxTimeoutTimers[radio]);

    switch (MULTIRFM96W[radio].Settings.Modem)
    {
    case MODEM_FSK:
    {
        rxContinuous = MULTIRFM96W[radio].Settings.Fsk.RxContinuous;

        // DIO0=PayloadReady
        // DIO1=FifoLevel
        // DIO2=SyncAddr
        // DIO3=FifoEmpty
        // DIO4=Preamble
        // DIO5=ModeReady
        MULTIRFM96WWrite(radio, REG_DIOMAPPING1, (MULTIRFM96WRead(radio, REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK) | RF_DIOMAPPING1_DIO0_00 | RF_DIOMAPPING1_DIO1_00 | RF_DIOMAPPING1_DIO2_11);

        MULTIRFM96WWrite(radio, REG_DIOMAPPING2, (MULTIRFM96WRead(radio, REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK) | RF_DIOMAPPING2_DIO4_11 | RF_DIOMAPPING2_MAP_PREAMBLEDETECT);

        MULTIRFM96W[radio].Settings.FskPacketHandler.FifoThresh = MULTIRFM96WRead(radio, REG_FIFOTHRESH) & 0x3F;

        MULTIRFM96WWrite(radio, REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT);

        MULTIRFM96W[radio].Settings.FskPacketHandler.PreambleDetected = false;
        MULTIRFM96W[radio].Settings.FskPacketHandler.SyncWordDetected = false;
        MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes = 0;
        MULTIRFM96W[radio].Settings.FskPacketHandler.Size = 0;
    }
    break;
    case MODEM_LORA:
    {
        if (MULTIRFM96W[radio].Settings.LoRa.IqInverted == true)
        {
            MULTIRFM96WWrite(radio, REG_LR_INVERTIQ, ((MULTIRFM96WRead(radio, REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF));
            MULTIRFM96WWrite(radio, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
        }
        else
        {
            MULTIRFM96WWrite(radio, REG_LR_INVERTIQ, ((MULTIRFM96WRead(radio, REG_LR_INVERTIQ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
            MULTIRFM96WWrite(radio, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
        }

        // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
        if (MULTIRFM96W[radio].Settings.LoRa.Bandwidth < 9)
        {
            MULTIRFM96WWrite(radio, REG_LR_DETECTOPTIMIZE, MULTIRFM96WRead(radio, REG_LR_DETECTOPTIMIZE) & 0x7F);
            MULTIRFM96WWrite(radio, REG_LR_IFFREQ2, 0x00);
            switch (MULTIRFM96W[radio].Settings.LoRa.Bandwidth)
            {
            case 0: // 7.8 kHz
                MULTIRFM96WWrite(radio, REG_LR_IFFREQ1, 0x48);
                MULTIRFM96WSetChannel(radio, MULTIRFM96W[radio].Settings.Channel + 7810);
                break;
            case 1: // 10.4 kHz
                MULTIRFM96WWrite(radio, REG_LR_IFFREQ1, 0x44);
                MULTIRFM96WSetChannel(radio, MULTIRFM96W[radio].Settings.Channel + 10420);
                break;
            case 2: // 15.6 kHz
                MULTIRFM96WWrite(radio, REG_LR_IFFREQ1, 0x44);
                MULTIRFM96WSetChannel(radio, MULTIRFM96W[radio].Settings.Channel + 15620);
                break;
            case 3: // 20.8 kHz
                MULTIRFM96WWrite(radio, REG_LR_IFFREQ1, 0x44);
                MULTIRFM96WSetChannel(radio, MULTIRFM96W[radio].Settings.Channel + 20830);
                break;
            case 4: // 31.2 kHz
                MULTIRFM96WWrite(radio, REG_LR_IFFREQ1, 0x44);
                MULTIRFM96WSetChannel(radio, MULTIRFM96W[radio].Settings.Channel + 31250);
                break;
            case 5: // 41.4 kHz
                MULTIRFM96WWrite(radio, REG_LR_IFFREQ1, 0x44);
                MULTIRFM96WSetChannel(radio, MULTIRFM96W[radio].Settings.Channel + 41670);
                break;
            case 6: // 62.5 kHz
                MULTIRFM96WWrite(radio, REG_LR_IFFREQ1, 0x40);
                break;
            case 7: // 125 kHz
                MULTIRFM96WWrite(radio, REG_LR_IFFREQ1, 0x40);
                break;
            case 8: // 250 kHz
                MULTIRFM96WWrite(radio, REG_LR_IFFREQ1, 0x40);
                break;
            }
        }
        else
        {
            MULTIRFM96WWrite(radio, REG_LR_DETECTOPTIMIZE, MULTIRFM96WRead(radio, REG_LR_DETECTOPTIMIZE) | 0x80);
        }

        rxContinuous = MULTIRFM96W[radio].Settings.LoRa.RxContinuous;

        if (MULTIRFM96W[radio].Settings.LoRa.FreqHopOn == true)
        {
            MULTIRFM96WWrite(radio, REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                             //RFLR_IRQFLAGS_RXDONE |
                             //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                             RFLR_IRQFLAGS_VALIDHEADER |
                                 RFLR_IRQFLAGS_TXDONE |
                                 RFLR_IRQFLAGS_CADDONE |
                                 //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                 RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=RxDone, DIO2=FhssChangeChannel
            MULTIRFM96WWrite(radio, REG_DIOMAPPING1, (MULTIRFM96WRead(radio, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00);
        }
        else
        {
            //uncommented RFLR_IRQFLAGS_RXTIMEOUT
            MULTIRFM96WWrite(radio, REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                             //RFLR_IRQFLAGS_RXDONE |
                             //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                             RFLR_IRQFLAGS_VALIDHEADER |
                                 RFLR_IRQFLAGS_TXDONE |
                                 RFLR_IRQFLAGS_CADDONE |
                                 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                 RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=RxDone
            MULTIRFM96WWrite(radio, REG_DIOMAPPING1, (MULTIRFM96WRead(radio, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);
        }
        MULTIRFM96WWrite(radio, REG_LR_FIFORXBASEADDR, 0);
        MULTIRFM96WWrite(radio, REG_LR_FIFOADDRPTR, 0);
    }
    break;
    }

    memset(RxTxBuffer[radio], 0, (size_t)RX_BUFFER_SIZE);

    MULTIRFM96W[radio].Settings.State = RF_RX_RUNNING;
    if (timeout != 0)
    {
        TimerSetValue(&RxTimeoutTimers[radio], timeout);
        TimerStart(&RxTimeoutTimers[radio]);
    }

    if (MULTIRFM96W[radio].Settings.Modem == MODEM_FSK)
    {
        MULTIRFM96WSetOpMode(radio, RF_OPMODE_RECEIVER);

        TimerSetValue(&RxTimeoutSyncWords[radio], MULTIRFM96W[radio].Settings.Fsk.RxSingleTimeout);
        TimerStart(&RxTimeoutSyncWords[radio]);
    }
    else
    {
        if (rxContinuous == true)
        {
            MULTIRFM96WSetOpMode(radio, RFLR_OPMODE_RECEIVER);
        }
        else
        {
            MULTIRFM96WSetOpMode(radio, RFLR_OPMODE_RECEIVER_SINGLE);
        }
    }
}

void MULTIRFM96WSetTx(RadioIndex_t radio, uint32_t timeout)
{
    TimerStop(&RxTimeoutTimers[radio]);

    TimerSetValue(&TxTimeoutTimers[radio], timeout);

    switch (MULTIRFM96W[radio].Settings.Modem)
    {
    case MODEM_FSK:
    {
        // DIO0=PacketSent
        // DIO1=FifoEmpty
        // DIO2=FifoFull
        // DIO3=FifoEmpty
        // DIO4=LowBat
        // DIO5=ModeReady
        MULTIRFM96WWrite(radio, REG_DIOMAPPING1, (MULTIRFM96WRead(radio, REG_DIOMAPPING1) & RF_DIOMAPPING1_DIO0_MASK & RF_DIOMAPPING1_DIO1_MASK & RF_DIOMAPPING1_DIO2_MASK) | RF_DIOMAPPING1_DIO1_01);

        MULTIRFM96WWrite(radio, REG_DIOMAPPING2, (MULTIRFM96WRead(radio, REG_DIOMAPPING2) & RF_DIOMAPPING2_DIO4_MASK & RF_DIOMAPPING2_MAP_MASK));
        MULTIRFM96W[radio].Settings.FskPacketHandler.FifoThresh = MULTIRFM96WRead(radio, REG_FIFOTHRESH) & 0x3F;
    }
    break;
    case MODEM_LORA:
    {
        if (MULTIRFM96W[radio].Settings.LoRa.FreqHopOn == true)
        {
            MULTIRFM96WWrite(radio, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER |
                                                             //RFLR_IRQFLAGS_TXDONE |
                                                             RFLR_IRQFLAGS_CADDONE |
                                                             //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                             RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=TxDone, DIO2=FhssChangeChannel
            MULTIRFM96WWrite(radio, REG_DIOMAPPING1, (MULTIRFM96WRead(radio, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00);
        }
        else
        {
            MULTIRFM96WWrite(radio, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER |
                                                             //RFLR_IRQFLAGS_TXDONE |
                                                             RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED);

            // DIO0=TxDone
            MULTIRFM96WWrite(radio, REG_DIOMAPPING1, (MULTIRFM96WRead(radio, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_01);
        }
    }
    break;
    }

    MULTIRFM96W[radio].Settings.State = RF_TX_RUNNING;
    TimerStart(&TxTimeoutTimers[radio]);
    MULTIRFM96WSetOpMode(radio, RF_OPMODE_TRANSMITTER);
}

void MULTIRFM96WStartCad(RadioIndex_t radio)
{
    switch (MULTIRFM96W[radio].Settings.Modem)
    {
    case MODEM_FSK:
    {
    }
    break;
    case MODEM_LORA:
    {
        MULTIRFM96WWrite(radio, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER | RFLR_IRQFLAGS_TXDONE |
                                                         //RFLR_IRQFLAGS_CADDONE |
                                                         RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                                                                          //RFLR_IRQFLAGS_CADDETECTED
        );

        // DIO3=CADDone
        MULTIRFM96WWrite(radio, REG_DIOMAPPING1, (MULTIRFM96WRead(radio, REG_DIOMAPPING1) & RFLR_DIOMAPPING1_DIO3_MASK) | RFLR_DIOMAPPING1_DIO3_00);

        MULTIRFM96W[radio].Settings.State = RF_CAD;
        MULTIRFM96WSetOpMode(radio, RFLR_OPMODE_CAD);
    }
    break;
    default:
        break;
    }
}

void MULTIRFM96WSetTxContinuousWave(RadioIndex_t radio, uint32_t freq, int8_t power, uint16_t time)
{
    uint32_t timeout = (uint32_t)(time * 1000);

    MULTIRFM96WSetChannel(radio, freq);

    MULTIRFM96WSetTxConfig(radio, MODEM_FSK, power, 0, 0, 4800, 0, 5, false, false, 0, 0, 0, timeout);

    MULTIRFM96WWrite(radio, REG_PACKETCONFIG2, (MULTIRFM96WRead(radio, REG_PACKETCONFIG2) & RF_PACKETCONFIG2_DATAMODE_MASK));
    // Disable radio interrupts
    MULTIRFM96WWrite(radio, REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_11 | RF_DIOMAPPING1_DIO1_11);
    MULTIRFM96WWrite(radio, REG_DIOMAPPING2, RF_DIOMAPPING2_DIO4_10 | RF_DIOMAPPING2_DIO5_10);

    TimerSetValue(&TxTimeoutTimers[radio], timeout);

    MULTIRFM96W[radio].Settings.State = RF_TX_RUNNING;
    TimerStart(&TxTimeoutTimers[radio]);
    MULTIRFM96WSetOpMode(radio, RF_OPMODE_TRANSMITTER);
}

int16_t MULTIRFM96WReadRssi(RadioIndex_t radio, RadioModems_t modem)
{
    int16_t rssi = 0;

    switch (modem)
    {
    case MODEM_FSK:
        rssi = -(MULTIRFM96WRead(radio, REG_RSSIVALUE) >> 1);
        break;
    case MODEM_LORA:
        if (MULTIRFM96W[radio].Settings.Channel > RF_MID_BAND_THRESH)
        {
            rssi = RSSI_OFFSET_HF + MULTIRFM96WRead(radio, REG_LR_RSSIVALUE);
        }
        else
        {
            rssi = RSSI_OFFSET_LF + MULTIRFM96WRead(radio, REG_LR_RSSIVALUE);
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

void MULTIRFM96WSetOpMode(RadioIndex_t radio, uint8_t opMode)
{

    if (opMode != RF_OPMODE_SLEEP)
    {
        MULTIRFM96WSetBoardTcxo(radio, true);
    }
    MULTIRFM96WWrite(radio, REG_OPMODE, (MULTIRFM96WRead(radio, REG_OPMODE) & RF_OPMODE_MASK) | opMode);
}

void MULTIRFM96WSetModem(RadioIndex_t radio, RadioModems_t modem)
{
    if ((MULTIRFM96WRead(radio, REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_ON) != 0)
    {
        MULTIRFM96W[radio].Settings.Modem = MODEM_LORA;
    }
    else
    {
        MULTIRFM96W[radio].Settings.Modem = MODEM_FSK;
    }

    if (MULTIRFM96W[radio].Settings.Modem == modem)
    {
        return;
    }

    MULTIRFM96W[radio].Settings.Modem = modem;
    switch (MULTIRFM96W[radio].Settings.Modem)
    {
    default:
    case MODEM_FSK:
        MULTIRFM96WSetOpMode(radio, RF_OPMODE_SLEEP);
        MULTIRFM96WWrite(radio, REG_OPMODE, (MULTIRFM96WRead(radio, REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_OFF);

        MULTIRFM96WWrite(radio, REG_DIOMAPPING1, 0x00);
        MULTIRFM96WWrite(radio, REG_DIOMAPPING2, 0x30); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        MULTIRFM96WSetOpMode(radio, RF_OPMODE_SLEEP);
        MULTIRFM96WWrite(radio, REG_OPMODE, (MULTIRFM96WRead(radio, REG_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_MASK) | RFLR_OPMODE_LONGRANGEMODE_ON);

        MULTIRFM96WWrite(radio, REG_DIOMAPPING1, 0x00);
        MULTIRFM96WWrite(radio, REG_DIOMAPPING2, 0x00);
        break;
    }
}

void MULTIRFM96WWrite(RadioIndex_t radio, uint16_t addr, uint8_t data)
{
    MULTIRFM96WWriteBuffer(radio, addr, &data, 1);
}

uint8_t MULTIRFM96WRead(RadioIndex_t radio, uint16_t addr)
{
    uint8_t data;
    MULTIRFM96WReadBuffer(radio, addr, &data, 1);
    return data;
}

void MULTIRFM96WWriteBuffer(RadioIndex_t radio, uint16_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    //NSS = 0;
    GpioWrite(&MULTIRFM96W[radio].Spi.Nss, 0);

    SpiInOut(&MULTIRFM96W[radio].Spi, addr | 0x80);
    for (i = 0; i < size; i++)
    {
        SpiInOut(&MULTIRFM96W[radio].Spi, buffer[i]);
    }

    //NSS = 1;
    GpioWrite(&MULTIRFM96W[radio].Spi.Nss, 1);
}

void MULTIRFM96WReadBuffer(volatile RadioIndex_t radio, uint16_t addr, uint8_t *buffer, uint8_t size)
{
    uint8_t i;
    //NSS = 0;
    GpioWrite(&MULTIRFM96W[radio].Spi.Nss, 0);

    SpiInOut(&MULTIRFM96W[radio].Spi, addr & 0x7F);

    for (i = 0; i < size; i++)
    {
        buffer[i] = SpiInOut(&MULTIRFM96W[radio].Spi, 0);
    }

    //NSS = 1;
    GpioWrite(&MULTIRFM96W[radio].Spi.Nss, 1);
}

void MULTIRFM96WWriteFifo(RadioIndex_t radio, uint8_t *buffer, uint8_t size)
{
    MULTIRFM96WWriteBuffer(radio, 0, buffer, size);
}

void MULTIRFM96WReadFifo(RadioIndex_t radio, uint8_t *buffer, uint8_t size)
{
    MULTIRFM96WReadBuffer(radio, 0, buffer, size);
}

void MULTIRFM96WSetMaxPayloadLength(RadioIndex_t radio, RadioModems_t modem, uint8_t max)
{
    MULTIRFM96WSetModem(radio, modem);

    switch (modem)
    {
    case MODEM_FSK:
        if (MULTIRFM96W[radio].Settings.Fsk.FixLen == false)
        {
            MULTIRFM96WWrite(radio, REG_PAYLOADLENGTH, max);
        }
        break;
    case MODEM_LORA:
        MULTIRFM96WWrite(radio, REG_LR_PAYLOADMAXLENGTH, max);
        break;
    }
}

void MULTIRFM96WSetPublicNetwork(RadioIndex_t radio, bool enable)
{
    MULTIRFM96WSetModem(radio, MODEM_LORA);
    MULTIRFM96W[radio].Settings.LoRa.PublicNetwork = enable;
    if (enable == true)
    {
        // Change LoRa modem SyncWord
        MULTIRFM96WWrite(radio, REG_LR_SYNCWORD, LORA_MAC_PUBLIC_SYNCWORD);
    }
    else
    {
        // Change LoRa modem SyncWord
        MULTIRFM96WWrite(radio, REG_LR_SYNCWORD, LORA_MAC_PRIVATE_SYNCWORD);
    }
}

uint32_t MULTIRFM96WGetWakeupTime(RadioIndex_t radio)
{
    return MULTIRFM96WGetBoardTcxoWakeupTime(radio) + RADIO_WAKEUP_TIME;
}

void MULTIRFM96WOnTimeoutIrq(RadioIndex_t radio, void *context)
{
    switch (MULTIRFM96W[radio].Settings.State)
    {
    case RF_RX_RUNNING:
        if (MULTIRFM96W[radio].Settings.Modem == MODEM_FSK)
        {
            MULTIRFM96W[radio].Settings.FskPacketHandler.PreambleDetected = false;
            MULTIRFM96W[radio].Settings.FskPacketHandler.SyncWordDetected = false;
            MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes = 0;
            MULTIRFM96W[radio].Settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            MULTIRFM96WWrite(radio, REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI | RF_IRQFLAGS1_PREAMBLEDETECT | RF_IRQFLAGS1_SYNCADDRESSMATCH);
            MULTIRFM96WWrite(radio, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

            if (MULTIRFM96W[radio].Settings.Fsk.RxContinuous == true)
            {
                // Continuous mode restart Rx chain
                MULTIRFM96WWrite(radio, REG_RXCONFIG, MULTIRFM96WRead(radio, REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                TimerStart(&RxTimeoutSyncWords[radio]);
            }
            else
            {
                MULTIRFM96W[radio].Settings.State = RF_IDLE;
                TimerStop(&RxTimeoutSyncWords[radio]);
            }
        }
        if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
        {
            RadioEvents->RxTimeout(radio);
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
        MULTIRFM96WReset(radio);

        // Calibrate Rx chain
        RxChainCalibration(radio);

        // Initialize radio default values
        MULTIRFM96WSetOpMode(radio, RF_OPMODE_SLEEP);

        for (uint8_t i = 0; i < sizeof(RadioRegsInit) / sizeof(RadioRegisters_t); i++)
        {
            MULTIRFM96WSetModem(radio, RadioRegsInit[i].Modem);
            MULTIRFM96WWrite(radio, RadioRegsInit[i].Addr, RadioRegsInit[i].Value);
        }
        MULTIRFM96WSetModem(radio, MODEM_FSK);

        // Restore previous network type setting.
        MULTIRFM96WSetPublicNetwork(radio, MULTIRFM96W[radio].Settings.LoRa.PublicNetwork);
        // END WORKAROUND

        MULTIRFM96W[radio].Settings.State = RF_IDLE;
        if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL))
        {
            RadioEvents->TxTimeout(radio);
        }
        break;
    default:
        break;
    }
}

void MULTIRFM96WOnDio0Irq(RadioIndex_t radio, void *context)
{
    volatile uint8_t irqFlags = 0;
    switch (MULTIRFM96W[radio].Settings.State)
    {
    case RF_RX_RUNNING:
        //TimerStop( &RxTimeoutTimers[radio] );
        // RxDone interrupt
        switch (MULTIRFM96W[radio].Settings.Modem)
        {
        case MODEM_FSK:
            if (MULTIRFM96W[radio].Settings.Fsk.CrcOn == true)
            {
                irqFlags = MULTIRFM96WRead(radio, REG_IRQFLAGS2);
                if ((irqFlags & RF_IRQFLAGS2_CRCOK) != RF_IRQFLAGS2_CRCOK)
                {
                    // Clear Irqs
                    MULTIRFM96WWrite(radio, REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI | RF_IRQFLAGS1_PREAMBLEDETECT | RF_IRQFLAGS1_SYNCADDRESSMATCH);
                    MULTIRFM96WWrite(radio, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);

                    TimerStop(&RxTimeoutTimers[radio]);

                    if (MULTIRFM96W[radio].Settings.Fsk.RxContinuous == false)
                    {
                        TimerStop(&RxTimeoutSyncWords[radio]);
                        MULTIRFM96W[radio].Settings.State = RF_IDLE;
                    }
                    else
                    {
                        // Continuous mode restart Rx chain
                        MULTIRFM96WWrite(radio, REG_RXCONFIG, MULTIRFM96WRead(radio, REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                        TimerStart(&RxTimeoutSyncWords[radio]);
                    }

                    if ((RadioEvents != NULL) && (RadioEvents->RxError != NULL))
                    {
                        RadioEvents->RxError(radio);
                    }
                    MULTIRFM96W[radio].Settings.FskPacketHandler.PreambleDetected = false;
                    MULTIRFM96W[radio].Settings.FskPacketHandler.SyncWordDetected = false;
                    MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes = 0;
                    MULTIRFM96W[radio].Settings.FskPacketHandler.Size = 0;
                    break;
                }
            }

            // Read received packet size
            if ((MULTIRFM96W[radio].Settings.FskPacketHandler.Size == 0) && (MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes == 0))
            {
                if (MULTIRFM96W[radio].Settings.Fsk.FixLen == false)
                {
                    MULTIRFM96WReadFifo(radio, (uint8_t *)&MULTIRFM96W[radio].Settings.FskPacketHandler.Size, 1);
                }
                else
                {
                    MULTIRFM96W[radio].Settings.FskPacketHandler.Size = MULTIRFM96WRead(radio, REG_PAYLOADLENGTH);
                }
                MULTIRFM96WReadFifo(radio, RxTxBuffer[radio] + MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes, MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes);
                MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes += (MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes);
            }
            else
            {
                MULTIRFM96WReadFifo(radio, RxTxBuffer[radio] + MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes, MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes);
                MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes += (MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes);
            }

            TimerStop(&RxTimeoutTimers[radio]);

            if (MULTIRFM96W[radio].Settings.Fsk.RxContinuous == false)
            {
                MULTIRFM96W[radio].Settings.State = RF_IDLE;
                TimerStop(&RxTimeoutSyncWords[radio]);
            }
            else
            {
                // Continuous mode restart Rx chain
                MULTIRFM96WWrite(radio, REG_RXCONFIG, MULTIRFM96WRead(radio, REG_RXCONFIG) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK);
                TimerStart(&RxTimeoutSyncWords[radio]);
            }

            if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
            {
                RadioEvents->RxDone(radio, RxTxBuffer[radio], MULTIRFM96W[radio].Settings.FskPacketHandler.Size, MULTIRFM96W[radio].Settings.FskPacketHandler.RssiValue, 0);
            }
            MULTIRFM96W[radio].Settings.FskPacketHandler.PreambleDetected = false;
            MULTIRFM96W[radio].Settings.FskPacketHandler.SyncWordDetected = false;
            MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes = 0;
            MULTIRFM96W[radio].Settings.FskPacketHandler.Size = 0;
            break;
        case MODEM_LORA:
        {
            // Clear Irq
            MULTIRFM96WWrite(radio, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

            irqFlags = MULTIRFM96WRead(radio, REG_LR_IRQFLAGS);
            if ((irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR)
            {
                // Clear Irq
                MULTIRFM96WWrite(radio, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR);

                if (MULTIRFM96W[radio].Settings.LoRa.RxContinuous == false)
                {
                    MULTIRFM96W[radio].Settings.State = RF_IDLE;
                }
                TimerStop(&RxTimeoutTimers[radio]);

                if ((RadioEvents != NULL) && (RadioEvents->RxError != NULL))
                {
                    RadioEvents->RxError(radio);
                }
                break;
            }

            // Returns SNR value [dB] rounded to the nearest integer value
            MULTIRFM96W[radio].Settings.LoRaPacketHandler.SnrValue = (((int8_t)MULTIRFM96WRead(radio, REG_LR_PKTSNRVALUE)) + 2) >> 2;

            int16_t rssi = MULTIRFM96WRead(radio, REG_LR_PKTRSSIVALUE);
            if (MULTIRFM96W[radio].Settings.LoRaPacketHandler.SnrValue < 0)
            {
                if (MULTIRFM96W[radio].Settings.Channel > RF_MID_BAND_THRESH)
                {
                    MULTIRFM96W[radio].Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4) +
                                                                              MULTIRFM96W[radio].Settings.LoRaPacketHandler.SnrValue;
                }
                else
                {
                    MULTIRFM96W[radio].Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4) +
                                                                              MULTIRFM96W[radio].Settings.LoRaPacketHandler.SnrValue;
                }
            }
            else
            {
                if (MULTIRFM96W[radio].Settings.Channel > RF_MID_BAND_THRESH)
                {
                    MULTIRFM96W[radio].Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + (rssi >> 4);
                }
                else
                {
                    MULTIRFM96W[radio].Settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + (rssi >> 4);
                }
            }

            MULTIRFM96W[radio].Settings.LoRaPacketHandler.Size = MULTIRFM96WRead(radio, REG_LR_RXNBBYTES);
            MULTIRFM96WWrite(radio, REG_LR_FIFOADDRPTR, MULTIRFM96WRead(radio, REG_LR_FIFORXCURRENTADDR));
            MULTIRFM96WReadFifo(radio, RxTxBuffer[radio], MULTIRFM96W[radio].Settings.LoRaPacketHandler.Size);

            if (MULTIRFM96W[radio].Settings.LoRa.RxContinuous == false)
            {
                MULTIRFM96W[radio].Settings.State = RF_IDLE;
            }
            TimerStop(&RxTimeoutTimers[radio]);

            if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
            {
                RadioEvents->RxDone(radio, RxTxBuffer[radio], MULTIRFM96W[radio].Settings.LoRaPacketHandler.Size, MULTIRFM96W[radio].Settings.LoRaPacketHandler.RssiValue, MULTIRFM96W[radio].Settings.LoRaPacketHandler.SnrValue);
            }
        }
        break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        TimerStop(&TxTimeoutTimers[radio]);
        // TxDone interrupt
        switch (MULTIRFM96W[radio].Settings.Modem)
        {
        case MODEM_LORA:
            // Clear Irq
            MULTIRFM96WWrite(radio, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
            // Intentional fall through
        case MODEM_FSK:
        default:
            MULTIRFM96W[radio].Settings.State = RF_IDLE;
            if ((RadioEvents != NULL) && (RadioEvents->TxDone != NULL))
            {
                RadioEvents->TxDone(radio);
            }
            break;
        }
        break;
    default:
        break;
    }
}

void MULTIRFM96WOnDio1Irq(RadioIndex_t radio, void *context)
{
    switch (MULTIRFM96W[radio].Settings.State)
    {
    case RF_RX_RUNNING:
        switch (MULTIRFM96W[radio].Settings.Modem)
        {
        case MODEM_FSK:
            // Stop timer
            TimerStop(&RxTimeoutSyncWords[radio]);

            // FifoLevel interrupt
            // Read received packet size
            if ((MULTIRFM96W[radio].Settings.FskPacketHandler.Size == 0) && (MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes == 0))
            {
                if (MULTIRFM96W[radio].Settings.Fsk.FixLen == false)
                {
                    MULTIRFM96WReadFifo(radio, (uint8_t *)&MULTIRFM96W[radio].Settings.FskPacketHandler.Size, 1);
                }
                else
                {
                    MULTIRFM96W[radio].Settings.FskPacketHandler.Size = MULTIRFM96WRead(radio, REG_PAYLOADLENGTH);
                }
            }

            // ERRATA 3.1 - PayloadReady Set for 31.25ns if FIFO is Empty
            //
            //              When FifoLevel interrupt is used to offload the
            //              FIFO, the microcontroller should  monitor  both
            //              PayloadReady  and FifoLevel interrupts, and
            //              read only (FifoThreshold-1) bytes off the FIFO
            //              when FifoLevel fires
            if ((MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes) >= MULTIRFM96W[radio].Settings.FskPacketHandler.FifoThresh)
            {
                MULTIRFM96WReadFifo(radio, (RxTxBuffer[radio] + MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes), MULTIRFM96W[radio].Settings.FskPacketHandler.FifoThresh - 1);
                MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes += MULTIRFM96W[radio].Settings.FskPacketHandler.FifoThresh - 1;
            }
            else
            {
                MULTIRFM96WReadFifo(radio, (RxTxBuffer[radio] + MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes), MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes);
                MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes += (MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes);
            }
            break;
        case MODEM_LORA:
            // Sync time out
            TimerStop(&RxTimeoutTimers[radio]);
            // Clear Irq
            MULTIRFM96WWrite(radio, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT);

            MULTIRFM96W[radio].Settings.State = RF_IDLE;
            if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
            {
                RadioEvents->RxTimeout(radio);
            }
            break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        switch (MULTIRFM96W[radio].Settings.Modem)
        {
        case MODEM_FSK:
            // FifoEmpty interrupt
            if ((MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes) > MULTIRFM96W[radio].Settings.FskPacketHandler.ChunkSize)
            {
                MULTIRFM96WWriteFifo(radio, (RxTxBuffer[radio] + MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes), MULTIRFM96W[radio].Settings.FskPacketHandler.ChunkSize);
                MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes += MULTIRFM96W[radio].Settings.FskPacketHandler.ChunkSize;
            }
            else
            {
                // Write the last chunk of data
                MULTIRFM96WWriteFifo(radio, RxTxBuffer[radio] + MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes, MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes);
                MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes += MULTIRFM96W[radio].Settings.FskPacketHandler.Size - MULTIRFM96W[radio].Settings.FskPacketHandler.NbBytes;
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

void MULTIRFM96WOnDio2Irq(RadioIndex_t radio, void *context)
{
    switch (MULTIRFM96W[radio].Settings.State)
    {
    case RF_RX_RUNNING:
        switch (MULTIRFM96W[radio].Settings.Modem)
        {
        case MODEM_FSK:
            // Checks if DIO4 is connected. If it is not PreambleDetected is set to true.
            if (MULTIRFM96W[radio].DIO4.port == NULL)
            {
                MULTIRFM96W[radio].Settings.FskPacketHandler.PreambleDetected = true;
            }

            if ((MULTIRFM96W[radio].Settings.FskPacketHandler.PreambleDetected == true) && (MULTIRFM96W[radio].Settings.FskPacketHandler.SyncWordDetected == false))
            {
                TimerStop(&RxTimeoutSyncWords[radio]);

                MULTIRFM96W[radio].Settings.FskPacketHandler.SyncWordDetected = true;

                MULTIRFM96W[radio].Settings.FskPacketHandler.RssiValue = -(MULTIRFM96WRead(radio, REG_RSSIVALUE) >> 1);

                MULTIRFM96W[radio].Settings.FskPacketHandler.AfcValue = (int32_t)(double)(((uint16_t)MULTIRFM96WRead(radio, REG_AFCMSB) << 8) |
                                                                                          (uint16_t)MULTIRFM96WRead(radio, REG_AFCLSB)) *
                                                                        (double)FREQ_STEP;
                MULTIRFM96W[radio].Settings.FskPacketHandler.RxGain = (MULTIRFM96WRead(radio, REG_LNA) >> 5) & 0x07;
            }
            break;
        case MODEM_LORA:
            if (MULTIRFM96W[radio].Settings.LoRa.FreqHopOn == true)
            {
                // Clear Irq
                MULTIRFM96WWrite(radio, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                if ((RadioEvents != NULL) && (RadioEvents->FhssChangeChannel != NULL))
                {
                    RadioEvents->FhssChangeChannel(radio, (MULTIRFM96WRead(radio, REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
                }
            }
            break;
        default:
            break;
        }
        break;
    case RF_TX_RUNNING:
        switch (MULTIRFM96W[radio].Settings.Modem)
        {
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            if (MULTIRFM96W[radio].Settings.LoRa.FreqHopOn == true)
            {
                // Clear Irq
                MULTIRFM96WWrite(radio, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                if ((RadioEvents != NULL) && (RadioEvents->FhssChangeChannel != NULL))
                {
                    RadioEvents->FhssChangeChannel(radio, (MULTIRFM96WRead(radio, REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK));
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

void MULTIRFM96WOnDio3Irq(RadioIndex_t radio, void *context)
{
    switch (MULTIRFM96W[radio].Settings.Modem)
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if ((MULTIRFM96WRead(radio, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED) == RFLR_IRQFLAGS_CADDETECTED)
        {
            // Clear Irq
            MULTIRFM96WWrite(radio, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);
            if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL))
            {
                RadioEvents->CadDone(radio, true);
            }
        }
        else
        {
            // Clear Irq
            MULTIRFM96WWrite(radio, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE);
            if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL))
            {
                RadioEvents->CadDone(radio, false);
            }
        }
        break;
    default:
        break;
    }
}

void MULTIRFM96WOnDio4Irq(RadioIndex_t radio, void *context)
{
    switch (MULTIRFM96W[radio].Settings.Modem)
    {
    case MODEM_FSK:
    {
        if (MULTIRFM96W[radio].Settings.FskPacketHandler.PreambleDetected == false)
        {
            MULTIRFM96W[radio].Settings.FskPacketHandler.PreambleDetected = true;
        }
    }
    break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

void MULTIRFM96WOnDio5Irq(RadioIndex_t radio, void *context)
{
    switch (MULTIRFM96W[radio].Settings.Modem)
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

static void InitTimers(RadioIndex_t radio)
{
    switch (radio)
    {
    case RADIO0:
        TimerInit(&TxTimeoutTimers[0], Radio0TimeoutIrq);
        TimerInit(&RxTimeoutTimers[0], Radio0TimeoutIrq);
        TimerInit(&RxTimeoutSyncWords[0], Radio0TimeoutIrq);
        break;
    case RADIO1:
        TimerInit(&TxTimeoutTimers[1], Radio1TimeoutIrq);
        TimerInit(&RxTimeoutTimers[1], Radio1TimeoutIrq);
        TimerInit(&RxTimeoutSyncWords[1], Radio1TimeoutIrq);
        break;
    case RADIO2:
        TimerInit(&TxTimeoutTimers[2], Radio2TimeoutIrq);
        TimerInit(&RxTimeoutTimers[2], Radio2TimeoutIrq);
        TimerInit(&RxTimeoutSyncWords[2], Radio2TimeoutIrq);
        break;
    case RADIO3:
        TimerInit(&TxTimeoutTimers[3], Radio3TimeoutIrq);
        TimerInit(&RxTimeoutTimers[3], Radio3TimeoutIrq);
        TimerInit(&RxTimeoutSyncWords[3], Radio3TimeoutIrq);
        break;
    case RADIO4:
        TimerInit(&TxTimeoutTimers[4], Radio4TimeoutIrq);
        TimerInit(&RxTimeoutTimers[4], Radio4TimeoutIrq);
        TimerInit(&RxTimeoutSyncWords[4], Radio4TimeoutIrq);
        break;
    case RADIO5:
        TimerInit(&TxTimeoutTimers[5], Radio5TimeoutIrq);
        TimerInit(&RxTimeoutTimers[5], Radio5TimeoutIrq);
        TimerInit(&RxTimeoutSyncWords[5], Radio5TimeoutIrq);
        break;
    case RADIO6:
        TimerInit(&TxTimeoutTimers[6], Radio6TimeoutIrq);
        TimerInit(&RxTimeoutTimers[6], Radio6TimeoutIrq);
        TimerInit(&RxTimeoutSyncWords[6], Radio6TimeoutIrq);
        break;
    case RADIO7:
        TimerInit(&TxTimeoutTimers[7], Radio7TimeoutIrq);
        TimerInit(&RxTimeoutTimers[7], Radio7TimeoutIrq);
        TimerInit(&RxTimeoutSyncWords[7], Radio7TimeoutIrq);
        break;
    }
}

static void Radio0TimeoutIrq(void *context) { MULTIRFM96WOnTimeoutIrq(RADIO0, context); }
static void Radio1TimeoutIrq(void *context) { MULTIRFM96WOnTimeoutIrq(RADIO1, context); }
static void Radio2TimeoutIrq(void *context) { MULTIRFM96WOnTimeoutIrq(RADIO2, context); }
static void Radio3TimeoutIrq(void *context) { MULTIRFM96WOnTimeoutIrq(RADIO3, context); }
static void Radio4TimeoutIrq(void *context) { MULTIRFM96WOnTimeoutIrq(RADIO4, context); }
static void Radio5TimeoutIrq(void *context) { MULTIRFM96WOnTimeoutIrq(RADIO5, context); }
static void Radio6TimeoutIrq(void *context) { MULTIRFM96WOnTimeoutIrq(RADIO6, context); }
static void Radio7TimeoutIrq(void *context) { MULTIRFM96WOnTimeoutIrq(RADIO7, context); }

static void Radio0OnDio0Irq(void *context) { MULTIRFM96WOnDio0Irq(RADIO0, context); }
static void Radio1OnDio0Irq(void *context) { MULTIRFM96WOnDio0Irq(RADIO1, context); }
static void Radio2OnDio0Irq(void *context) { MULTIRFM96WOnDio0Irq(RADIO2, context); }
static void Radio3OnDio0Irq(void *context) { MULTIRFM96WOnDio0Irq(RADIO3, context); }
static void Radio4OnDio0Irq(void *context) { MULTIRFM96WOnDio0Irq(RADIO4, context); }
static void Radio5OnDio0Irq(void *context) { MULTIRFM96WOnDio0Irq(RADIO5, context); }
static void Radio6OnDio0Irq(void *context) { MULTIRFM96WOnDio0Irq(RADIO6, context); }
static void Radio7OnDio0Irq(void *context) { MULTIRFM96WOnDio0Irq(RADIO7, context); }

static void Radio0OnDio1Irq(void *context) { MULTIRFM96WOnDio1Irq(RADIO0, context); }
static void Radio1OnDio1Irq(void *context) { MULTIRFM96WOnDio1Irq(RADIO1, context); }
static void Radio2OnDio1Irq(void *context) { MULTIRFM96WOnDio1Irq(RADIO2, context); }
static void Radio3OnDio1Irq(void *context) { MULTIRFM96WOnDio1Irq(RADIO3, context); }
static void Radio4OnDio1Irq(void *context) { MULTIRFM96WOnDio1Irq(RADIO4, context); }
static void Radio5OnDio1Irq(void *context) { MULTIRFM96WOnDio1Irq(RADIO5, context); }
static void Radio6OnDio1Irq(void *context) { MULTIRFM96WOnDio1Irq(RADIO6, context); }
static void Radio7OnDio1Irq(void *context) { MULTIRFM96WOnDio1Irq(RADIO7, context); }

static void Radio0OnDio2Irq(void *context) { MULTIRFM96WOnDio2Irq(RADIO0, context); }
static void Radio1OnDio2Irq(void *context) { MULTIRFM96WOnDio2Irq(RADIO1, context); }
static void Radio2OnDio2Irq(void *context) { MULTIRFM96WOnDio2Irq(RADIO2, context); }
static void Radio3OnDio2Irq(void *context) { MULTIRFM96WOnDio2Irq(RADIO3, context); }
static void Radio4OnDio2Irq(void *context) { MULTIRFM96WOnDio2Irq(RADIO4, context); }
static void Radio5OnDio2Irq(void *context) { MULTIRFM96WOnDio2Irq(RADIO5, context); }
static void Radio6OnDio2Irq(void *context) { MULTIRFM96WOnDio2Irq(RADIO6, context); }
static void Radio7OnDio2Irq(void *context) { MULTIRFM96WOnDio2Irq(RADIO7, context); }

static void Radio0OnDio3Irq(void *context) { MULTIRFM96WOnDio3Irq(RADIO0, context); }
static void Radio1OnDio3Irq(void *context) { MULTIRFM96WOnDio3Irq(RADIO1, context); }
static void Radio2OnDio3Irq(void *context) { MULTIRFM96WOnDio3Irq(RADIO2, context); }
static void Radio3OnDio3Irq(void *context) { MULTIRFM96WOnDio3Irq(RADIO3, context); }
static void Radio4OnDio3Irq(void *context) { MULTIRFM96WOnDio3Irq(RADIO4, context); }
static void Radio5OnDio3Irq(void *context) { MULTIRFM96WOnDio3Irq(RADIO5, context); }
static void Radio6OnDio3Irq(void *context) { MULTIRFM96WOnDio3Irq(RADIO6, context); }
static void Radio7OnDio3Irq(void *context) { MULTIRFM96WOnDio3Irq(RADIO7, context); }

static void Radio0OnDio4Irq(void *context) { MULTIRFM96WOnDio4Irq(RADIO0, context); }
static void Radio1OnDio4Irq(void *context) { MULTIRFM96WOnDio4Irq(RADIO1, context); }
static void Radio2OnDio4Irq(void *context) { MULTIRFM96WOnDio4Irq(RADIO2, context); }
static void Radio3OnDio4Irq(void *context) { MULTIRFM96WOnDio4Irq(RADIO3, context); }
static void Radio4OnDio4Irq(void *context) { MULTIRFM96WOnDio4Irq(RADIO4, context); }
static void Radio5OnDio4Irq(void *context) { MULTIRFM96WOnDio4Irq(RADIO5, context); }
static void Radio6OnDio4Irq(void *context) { MULTIRFM96WOnDio4Irq(RADIO6, context); }
static void Radio7OnDio4Irq(void *context) { MULTIRFM96WOnDio4Irq(RADIO7, context); }

static void Radio0OnDio5Irq(void *context) { MULTIRFM96WOnDio5Irq(RADIO0, context); }
static void Radio1OnDio5Irq(void *context) { MULTIRFM96WOnDio5Irq(RADIO1, context); }
static void Radio2OnDio5Irq(void *context) { MULTIRFM96WOnDio5Irq(RADIO2, context); }
static void Radio3OnDio5Irq(void *context) { MULTIRFM96WOnDio5Irq(RADIO3, context); }
static void Radio4OnDio5Irq(void *context) { MULTIRFM96WOnDio5Irq(RADIO4, context); }
static void Radio5OnDio5Irq(void *context) { MULTIRFM96WOnDio5Irq(RADIO5, context); }
static void Radio6OnDio5Irq(void *context) { MULTIRFM96WOnDio5Irq(RADIO6, context); }
static void Radio7OnDio5Irq(void *context) { MULTIRFM96WOnDio5Irq(RADIO7, context); }