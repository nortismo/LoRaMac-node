/*!
 * \file      multiRfm96w-board.c
 *
 * \brief     Target board RFM96W driver implementation for
 *            multiple transceivers in parallel.
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \author    Diego Bienz
 */
#include <stdlib.h>
#include "utilities.h"
#include "board-config.h"
#include "delay.h"
#include "multiRadio.h"
#include "multiRfm96w-board.h"

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] power Selects the right PA according to the wanted power.
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t MULTIRFM96WGetPaSelect(RadioIndex_t radio, int8_t power);

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
    {
        MULTIRFM96WInit,
        MULTIRFM96WGetStatus,
        MULTIRFM96WSetModem,
        MULTIRFM96WSetChannel,
        MULTIRFM96WIsChannelFree,
        MULTIRFM96WRandom,
        MULTIRFM96WSetRxConfig,
        MULTIRFM96WSetTxConfig,
        MULTIRFM96WCheckRfFrequency,
        MULTIRFM96WGetTimeOnAir,
        MULTIRFM96WSend,
        MULTIRFM96WSetSleep,
        MULTIRFM96WSetStby,
        MULTIRFM96WSetRx,
        MULTIRFM96WStartCad,
        MULTIRFM96WSetTxContinuousWave,
        MULTIRFM96WReadRssi,
        MULTIRFM96WWrite,
        MULTIRFM96WRead,
        MULTIRFM96WWriteBuffer,
        MULTIRFM96WReadBuffer,
        MULTIRFM96WSetMaxPayloadLength,
        MULTIRFM96WSetPublicNetwork,
        MULTIRFM96WGetWakeupTime,
        NULL, // void ( *IrqProcess )( void )
        NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
        NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

/*!
 * TCXO power control pin
 */
Gpio_t TcxoPower;

void MULTIRFM96WIoInit()
{
    GpioInit(&MULTIRFM96W[0].Spi.Nss, RADIO1_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
    GpioInit(&MULTIRFM96W[0].DIO0, RADIO1_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);

    GpioInit(&MULTIRFM96W[1].Spi.Nss, RADIO2_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
    GpioInit(&MULTIRFM96W[1].DIO0, RADIO2_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);

    GpioInit(&MULTIRFM96W[2].Spi.Nss, RADIO3_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
    GpioInit(&MULTIRFM96W[2].DIO0, RADIO3_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);

    /*
    SmartSilo Errata 1.0: Radio4 can't be used, since it is on the same pin as UART

    GpioInit(&MULTIRFM96W[3].Spi.Nss, RADIO4_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
    GpioInit(&MULTIRFM96W[3].DIO0, RADIO4_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
    */

    GpioInit(&MULTIRFM96W[4].Spi.Nss, RADIO5_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
    GpioInit(&MULTIRFM96W[4].DIO0, RADIO5_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);

    GpioInit(&MULTIRFM96W[5].Spi.Nss, RADIO6_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
    GpioInit(&MULTIRFM96W[5].DIO0, RADIO6_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);

    GpioInit(&MULTIRFM96W[6].Spi.Nss, RADIO7_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
    GpioInit(&MULTIRFM96W[6].DIO0, RADIO7_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);

    GpioInit(&MULTIRFM96W[7].Spi.Nss, RADIO8_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
    GpioInit(&MULTIRFM96W[7].DIO0, RADIO8_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
}

void MULTIRFM96WIoIrqInit(DioIrqHandler **irqHandlers)
{
    GpioSetInterrupt(&MULTIRFM96W[0].DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[0]);

    GpioSetInterrupt(&MULTIRFM96W[1].DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[6]);

    GpioSetInterrupt(&MULTIRFM96W[2].DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[12]);

    /*
    SmartSilo Errata 1.0: Radio4 can't be used, since it is on the same pin as UART

    GpioSetInterrupt(&MULTIRFM96W[3].DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[18]);
    */

    GpioSetInterrupt(&MULTIRFM96W[4].DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[24]);

    GpioSetInterrupt(&MULTIRFM96W[5].DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[30]);

    GpioSetInterrupt(&MULTIRFM96W[6].DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[36]);

    GpioSetInterrupt(&MULTIRFM96W[7].DIO0, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, irqHandlers[42]);
}

void MULTIRFM96WIoDeInit()
{
    GpioInit(&MULTIRFM96W[0].Spi.Nss, RADIO1_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioInit(&MULTIRFM96W[0].DIO0, RADIO1_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);

    GpioInit(&MULTIRFM96W[1].Spi.Nss, RADIO2_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioInit(&MULTIRFM96W[1].DIO0, RADIO2_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);

    GpioInit(&MULTIRFM96W[2].Spi.Nss, RADIO3_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioInit(&MULTIRFM96W[2].DIO0, RADIO3_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);

    /*
    SmartSilo Errata 1.0: Radio4 can't be used, since it is on the same pin as UART

    GpioInit(&MULTIRFM96W[3].Spi.Nss, RADIO4_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioInit(&MULTIRFM96W[3].DIO0, RADIO4_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    */

    GpioInit(&MULTIRFM96W[4].Spi.Nss, RADIO5_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioInit(&MULTIRFM96W[4].DIO0, RADIO5_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);

    GpioInit(&MULTIRFM96W[5].Spi.Nss, RADIO6_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioInit(&MULTIRFM96W[5].DIO0, RADIO6_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);

    GpioInit(&MULTIRFM96W[6].Spi.Nss, RADIO7_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioInit(&MULTIRFM96W[6].DIO0, RADIO7_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);

    GpioInit(&MULTIRFM96W[7].Spi.Nss, RADIO8_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
    GpioInit(&MULTIRFM96W[7].DIO0, RADIO8_DIO_0, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
}

void MULTIRFM96WIoTcxoInit(void)
{
    /* Not available on this board */
    return;
}

void MULTIRFM96WSetBoardTcxo(uint8_t state)
{
    /* Not available on this board */
    return;
}

uint32_t MULTIRFM96WGetBoardTcxoWakeupTime(RadioIndex_t radio)
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void MULTIRFM96WResetAll()
{
    // Enables the TCXO if available on the board design
    MULTIRFM96WSetBoardTcxo(true);

    // Set RESET pin to 0
    GpioInit(&MULTIRFM96W[0].Reset, RADIOS_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);

    // Wait 1 ms
    DelayMs(1);

    // Configure RESET as input
    GpioInit(&MULTIRFM96W[0].Reset, RADIOS_RESET, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);

    // Wait 6 ms
    DelayMs(6);
}

void MULTIRFM96WSetRfTxPower(RadioIndex_t radio, int8_t power)
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = MULTIRFM96WRead(radio, REG_PACONFIG);
    paDac = MULTIRFM96WRead(radio, REG_PADAC);

    paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK) | MULTIRFM96WGetPaSelect(radio, power);

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
    MULTIRFM96WWrite(radio, REG_PACONFIG, paConfig);
    MULTIRFM96WWrite(radio, REG_PADAC, paDac);
}

static uint8_t MULTIRFM96WGetPaSelect(RadioIndex_t radio, int8_t power)
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

bool MULTIRFM96WCheckRfFrequency(RadioIndex_t radio, uint32_t frequency)
{
    // Implement check. Currently all frequencies are supported
    return true;
}
