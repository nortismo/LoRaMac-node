/*!
 * \file      rfm96w-board.c
 *
 * \brief     Target board RFM96W driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */
#include <stdlib.h>
#include "utilities.h"
#include "board-config.h"
#include "delay.h"
#include "radio.h"
#include "rfm96w-board.h"

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] power Selects the right PA according to the wanted power.
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t RFM96WGetPaSelect(int8_t power);

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
    {
        RFM96WInit,
        RFM96WGetStatus,
        RFM96WSetModem,
        RFM96WSetChannel,
        RFM96WIsChannelFree,
        RFM96WRandom,
        RFM96WSetRxConfig,
        RFM96WSetTxConfig,
        RFM96WCheckRfFrequency,
        RFM96WGetTimeOnAir,
        RFM96WSend,
        RFM96WSetSleep,
        RFM96WSetStby,
        RFM96WSetRx,
        RFM96WStartCad,
        RFM96WSetTxContinuousWave,
        RFM96WReadRssi,
        RFM96WWrite,
        RFM96WRead,
        RFM96WWriteBuffer,
        RFM96WReadBuffer,
        RFM96WSetMaxPayloadLength,
        RFM96WSetPublicNetwork,
        RFM96WGetWakeupTime,
        NULL, // void ( *IrqProcess )( void )
        NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
        NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

/*!
 * TCXO power control pin
 */
Gpio_t TcxoPower;

void RFM96WIoInit(void)
{
    GpioInit(&RFM96W.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);

    GpioInit(&RFM96W.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&RFM96W.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&RFM96W.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&RFM96W.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&RFM96W.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&RFM96W.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
}

void RFM96WIoIrqInit(DioIrqHandler **irqHandlers)
{
    GpioSetInterrupt(&RFM96W.DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[0]);
    GpioSetInterrupt(&RFM96W.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[1]);
    GpioSetInterrupt(&RFM96W.DIO2, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[2]);
    GpioSetInterrupt(&RFM96W.DIO3, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[3]);
    GpioSetInterrupt(&RFM96W.DIO4, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[4]);
    GpioSetInterrupt(&RFM96W.DIO5, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[5]);
}

void RFM96WIoDeInit(void)
{
    GpioInit(&RFM96W.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);

    GpioInit(&RFM96W.DIO0, RADIO_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&RFM96W.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&RFM96W.DIO2, RADIO_DIO_2, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&RFM96W.DIO3, RADIO_DIO_3, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&RFM96W.DIO4, RADIO_DIO_4, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&RFM96W.DIO5, RADIO_DIO_5, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
}

void RFM96WIoTcxoInit(void)
{
    GpioInit(&TcxoPower, RADIO_TCXO_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
}

void RFM96WSetBoardTcxo(uint8_t state)
{
    if (state == true)
    {
        if (GpioRead(&TcxoPower) == 0)
        { // TCXO OFF power it up.
            // Power ON the TCXO
            GpioWrite(&TcxoPower, 1);
            DelayMs(BOARD_TCXO_WAKEUP_TIME);
        }
    }
    else
    {
        // Power OFF the TCXO
        GpioWrite(&TcxoPower, 0);
    }
}

uint32_t RFM96WGetBoardTcxoWakeupTime(void)
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void RFM96WReset(void)
{
    // Enables the TCXO if available on the board design
    RFM96WSetBoardTcxo(true);

    // Set RESET pin to 0
    GpioInit(&RFM96W.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);

    // Wait 1 ms
    DelayMs(1);

    // Configure RESET as input
    GpioInit(&RFM96W.Reset, RADIO_RESET, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);

    // Wait 6 ms
    DelayMs(6);
}

void RFM96WSetRfTxPower(int8_t power)
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = RFM96WRead(REG_PACONFIG);
    paDac = RFM96WRead(REG_PADAC);

    paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK) | RFM96WGetPaSelect(power);

    if ((paConfig & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST)
    {
        if (power > 17)
        {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }
        if ((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON)
        {
            if (power < 5)
            {
                power = 5;
            }
            if (power > 20)
            {
                power = 20;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 5) & 0x0F);
        }
        else
        {
            if (power < 2)
            {
                power = 2;
            }
            if (power > 17)
            {
                power = 17;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 2) & 0x0F);
        }
    }
    else
    {
        if (power > 0)
        {
            if (power > 15)
            {
                power = 15;
            }
            paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (7 << 4) | (power);
        }
        else
        {
            if (power < -4)
            {
                power = -4;
            }
            paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (0 << 4) | (power + 4);
        }
    }
    RFM96WWrite(REG_PACONFIG, paConfig);
    RFM96WWrite(REG_PADAC, paDac);
}

static uint8_t RFM96WGetPaSelect(int8_t power)
{
    if (power > 14)
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

bool RFM96WCheckRfFrequency(uint32_t frequency)
{
    // Implement check. Currently all frequencies are supported
    return true;
}
