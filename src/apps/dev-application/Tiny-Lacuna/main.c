/*!
 * \file      main.c
 *
 * \brief     Application for the development of the Tiny-Lacuna board layer.
 *            Mainly used for testing purposes of new functions.
 *
 * \author    Diego Bienz
 */

#include <stdio.h>
#include <string.h>
#include "board.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "gpio.h"
#include "sx126x.h"
#include "gps.h"

#define RF_FREQUENCY                                868000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        16         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        true

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here

typedef enum {
	LOWPOWER, RX_COMPLETE, RX_TIMEOUT, RX_ERROR, TX_COMPLETE, TX_TIMEOUT,
} States_t;

const uint8_t Msg[] = "Hello World!";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

TimerEvent_t timer_event;

/*!
 * Example for a GPIO interrupt
 * Variable is set as soon as GPIO
 * interrupt happens on PTD_5
 */
extern uint8_t testIrq;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1;
extern Gpio_t Led2;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

/**
 * Main application entry point.
 */

/*!
 * Example timer callback
 */
bool gps_initialized = true;
void test_callback(void *context) {
	gps_initialized = false;
}

/**
 * Main application entry point.
 */
int main(void) {
	volatile RadioStatus_t dev_status;

	// Target board initialization
	BoardInitMcu();
	BoardInitPeriph();

	// Radio initialization
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;

	Radio.Init(&RadioEvents);
	Radio.SetChannel( RF_FREQUENCY);

	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
	LORA_SPREADING_FACTOR, LORA_CODINGRATE,
	LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
	true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
	LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
	LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
	LORA_IQ_INVERSION_ON, true);

	Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);

	DelayMs(500);
	printf("DEBUG (before TX): ChipMode: 0x%1.0x, CmdStatus: 0x%1.0x\r\n",
			dev_status.Fields.ChipMode, dev_status.Fields.CmdStatus);
	BufferSize = sizeof(Msg);
	memcpy(Buffer, Msg, BufferSize);
	Radio.Send(Buffer, BufferSize);
	GpioToggle(&Led2);
	printf("DEBUG (after TX): ChipMode: 0x%1.0x, CmdStatus: 0x%1.0x\r\n",
			dev_status.Fields.ChipMode, dev_status.Fields.CmdStatus);

	/*!
	 * Example of how to init the timer and start it.
	 * We don't use that here.
	 */
	TimerInit(&timer_event, test_callback);
	TimerStart(&timer_event);
	/* Busy delay */
	DelayMs(200);

	printf("\r\n### Start of development application ###\r\n");
	printf("BOARD: Tiny-Lacuna\r\n");
	printf("see: https://github.com/nortismo/LoRaMac-node\r\n\r\n");

	while (1) {

		switch (State) {
		case RX_COMPLETE:
			GpioToggle(&Led1);
			printf("Received something: \"%s\"\r\n", Buffer);
			printf("Sending again...\r\n");

			BufferSize = sizeof(Msg);
			memcpy(Buffer, Msg, BufferSize);
			Radio.Send(Buffer, BufferSize);
			GpioToggle(&Led2);

			State = LOWPOWER;
			break;
		case TX_COMPLETE:
			GpioToggle(&Led2);
			printf("Sent successfully \"%s\"!", Msg);

			Radio.Rx( RX_TIMEOUT_VALUE);
			GpioToggle(&Led1);
			State = LOWPOWER;
			break;
		case RX_TIMEOUT:
			GpioToggle(&Led2);
			printf("RX Timeout.\r\n");
			printf(
					"DEBUG (before RX): ChipMode: 0x%1.0x, CmdStatus: 0x%1.0x\r\n",
					dev_status.Fields.ChipMode, dev_status.Fields.CmdStatus);
			Radio.Rx( RX_TIMEOUT_VALUE);
			dev_status = SX126xGetStatus();
			printf(
					"DEBUG (after RX): ChipMode: 0x%1.0x, CmdStatus: 0x%1.0x\r\n",
					dev_status.Fields.ChipMode, dev_status.Fields.CmdStatus);
			State = LOWPOWER;
			break;
		case RX_ERROR:
			printf("RX Error.\r\n");
			printf(
					"DEBUG (before RX): ChipMode: 0x%1.0x, CmdStatus: 0x%1.0x\r\n",
					dev_status.Fields.ChipMode, dev_status.Fields.CmdStatus);
			Radio.Rx( RX_TIMEOUT_VALUE);
			State = LOWPOWER;
			dev_status = SX126xGetStatus();
			printf(
					"DEBUG (after RX): ChipMode: 0x%1.0x, CmdStatus: 0x%1.0x\r\n",
					dev_status.Fields.ChipMode, dev_status.Fields.CmdStatus);
			break;
		case TX_TIMEOUT:
			GpioToggle(&Led2);
			printf(
					"DEBUG (before RX): ChipMode: 0x%1.0x, CmdStatus: 0x%1.0x\r\n",
					dev_status.Fields.ChipMode, dev_status.Fields.CmdStatus);
			printf("TX Timeout.\r\n");
			Radio.Rx( RX_TIMEOUT_VALUE);
			printf(
					"DEBUG (after RX): ChipMode: 0x%1.0x, CmdStatus: 0x%1.0x\r\n",
					dev_status.Fields.ChipMode, dev_status.Fields.CmdStatus);
			GpioToggle(&Led1);
			State = LOWPOWER;
			break;
		case LOWPOWER:
		default:
			// Set low power
			break;
		}

		// Process Radio IRQ
		if (Radio.IrqProcess != NULL) {
			Radio.IrqProcess();
		}

		if (!gps_initialized) {
			gps_initialized = true;
			/* TODO: Make sure in future that this function is called from time to time */
			GpsProcess();
		}
	}
}

void OnTxDone(void) {
	//Radio.Sleep( );
	State = TX_COMPLETE;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
	//Radio.Sleep( );
	BufferSize = size;
	memcpy(Buffer, payload, BufferSize);
	RssiValue = rssi;
	SnrValue = snr;
	State = RX_COMPLETE;
}

void OnTxTimeout(void) {
	//Radio.Sleep( );
	State = TX_TIMEOUT;
}

void OnRxTimeout(void) {
	//Radio.Sleep( );
	State = RX_TIMEOUT;
}

void OnRxError(void) {
	//Radio.Sleep( );
	State = RX_ERROR;
}
