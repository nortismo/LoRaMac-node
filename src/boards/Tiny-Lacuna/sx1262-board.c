/*!
 * \file      sx1262-board.c
 *
 * \brief     Target board SX1262 driver implementation
 *
 * \author    Diego Bienz
 */

#include <stdlib.h>
#include "utilities.h"
#include "board-config.h"
#include "board.h"
#include "delay.h"
#include "radio.h"
#include "sx126x-board.h"

Gpio_t AntPow;

void SX126xIoInit(void) {
	GpioInit(&SX126x.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL,
			1);
	GpioInit(&SX126x.BUSY, RADIO_BUSY, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL,
			0);
	GpioInit(&SX126x.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL,
			0);
}

void SX126xIoIrqInit(DioIrqHandler dioIrq) {
	GpioSetInterrupt(&SX126x.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, dioIrq);
}

void SX126xIoDeInit(void) {
	GpioInit(&SX126x.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL,
			1);
	GpioInit(&SX126x.BUSY, RADIO_BUSY, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL,
			0);
	GpioInit(&SX126x.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL,
			0);
}

void SX126xIoDbgInit(void) {
	/* Nothing to do */
}

void SX126xIoTcxoInit(void) {
	CalibrationParams_t calibParam;

	SX126xSetDio3AsTcxoCtrl(TCXO_VOLTAGE_IN,
			SX126xGetBoardTcxoWakeupTime() << 6); // convert from ms to SX126x time base
	calibParam.Value = 0x7F;
	SX126xCalibrate(calibParam);
}

uint32_t SX126xGetBoardTcxoWakeupTime(void) {
	return TCXO_WAKEUP_TIME;
}

void SX126xReset(void) {
	DelayMs(10);
	GpioInit(&SX126x.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL,
			0);
	DelayMs(100);
	GpioInit(&SX126x.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL,
			1);
	DelayMs(10);
}

void SX126xWaitOnBusy(void) {
	while (GpioRead(&SX126x.BUSY) == 1) {
	}
}

void SX126xWakeup(void) {
	GpioWrite(&SX126x.Spi.Nss, 0);

	SpiInOut(&SX126x.Spi, RADIO_GET_STATUS);
	SpiInOut(&SX126x.Spi, 0x00);

	GpioWrite(&SX126x.Spi.Nss, 1);

// Wait for chip to be ready.
	SX126xWaitOnBusy();
}

void SX126xWriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size) {
	SX126xCheckDeviceReady();

	GpioWrite(&SX126x.Spi.Nss, 0);

	SpiInOut(&SX126x.Spi, (uint8_t) command);

	for (uint16_t i = 0; i < size; i++) {
		SpiInOut(&SX126x.Spi, buffer[i]);
	}

	GpioWrite(&SX126x.Spi.Nss, 1);

	if (command != RADIO_SET_SLEEP) {
		SX126xWaitOnBusy();
	}
}

uint8_t SX126xReadCommand(RadioCommands_t command, uint8_t *buffer,
		uint16_t size) {
	uint8_t status = 0;

	SX126xCheckDeviceReady();

	GpioWrite(&SX126x.Spi.Nss, 0);

	SpiInOut(&SX126x.Spi, (uint8_t) command);
	status = SpiInOut(&SX126x.Spi, 0x00);
	for (uint16_t i = 0; i < size; i++) {
		buffer[i] = SpiInOut(&SX126x.Spi, 0);
	}

	GpioWrite(&SX126x.Spi.Nss, 1);

	SX126xWaitOnBusy();

	return status;
}

void SX126xWriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size) {
	SX126xCheckDeviceReady();

	GpioWrite(&SX126x.Spi.Nss, 0);

	SpiInOut(&SX126x.Spi, RADIO_WRITE_REGISTER);
	SpiInOut(&SX126x.Spi, (address & 0xFF00) >> 8);
	SpiInOut(&SX126x.Spi, address & 0x00FF);

	for (uint16_t i = 0; i < size; i++) {
		SpiInOut(&SX126x.Spi, buffer[i]);
	}

	GpioWrite(&SX126x.Spi.Nss, 1);

	SX126xWaitOnBusy();
}

void SX126xWriteRegister(uint16_t address, uint8_t value) {
	SX126xWriteRegisters(address, &value, 1);
}

void SX126xReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size) {
	SX126xCheckDeviceReady();

	GpioWrite(&SX126x.Spi.Nss, 0);

	SpiInOut(&SX126x.Spi, RADIO_READ_REGISTER);
	SpiInOut(&SX126x.Spi, (address & 0xFF00) >> 8);
	SpiInOut(&SX126x.Spi, address & 0x00FF);
	SpiInOut(&SX126x.Spi, 0);
	for (uint16_t i = 0; i < size; i++) {
		buffer[i] = SpiInOut(&SX126x.Spi, 0);
	}
	GpioWrite(&SX126x.Spi.Nss, 1);

	SX126xWaitOnBusy();
}

uint8_t SX126xReadRegister(uint16_t address) {
	uint8_t data;
	SX126xReadRegisters(address, &data, 1);
	return data;
}

void SX126xWriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
	SX126xCheckDeviceReady();

	GpioWrite(&SX126x.Spi.Nss, 0);

	SpiInOut(&SX126x.Spi, RADIO_WRITE_BUFFER);
	SpiInOut(&SX126x.Spi, offset);
	for (uint16_t i = 0; i < size; i++) {
		SpiInOut(&SX126x.Spi, buffer[i]);
	}
	GpioWrite(&SX126x.Spi.Nss, 1);

	SX126xWaitOnBusy();
}

void SX126xReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
	SX126xCheckDeviceReady();

	GpioWrite(&SX126x.Spi.Nss, 0);

	SpiInOut(&SX126x.Spi, RADIO_READ_BUFFER);
	SpiInOut(&SX126x.Spi, offset);
	SpiInOut(&SX126x.Spi, 0);
	for (uint16_t i = 0; i < size; i++) {
		buffer[i] = SpiInOut(&SX126x.Spi, 0);
	}
	GpioWrite(&SX126x.Spi.Nss, 1);

	SX126xWaitOnBusy();
}

void SX126xSetRfTxPower(int8_t power) {
	SX126xSetTxParams(power, RADIO_RAMP_40_US);
}

uint8_t SX126xGetDeviceId(void) {
	return SX1262;
}

void SX126xAntSwOn(void) {
	GpioInit(&AntPow, RADIO_ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL,
			PIN_PULL_UP, 1);
}

void SX126xAntSwOff(void) {
	GpioInit(&AntPow, RADIO_ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL,
			PIN_NO_PULL, 0);
}

bool SX126xCheckRfFrequency(uint32_t frequency) {
// Implement check. Currently all frequencies are supported
	return true;
}
