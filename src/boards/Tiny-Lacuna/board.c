/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
 *
 * \author    Diego Bienz
 */

#include "utilities.h"

#include "uart.h"
#include "rtc-board.h"
#include "board-config.h"
#include "MK22F51212.h"
#include "board.h"
#include "clock_config.h"
#include "gpio.h"
#include "pin_mux.h"

/*!
 * LED GPIO pins objects
 */
Gpio_t Led1;
Gpio_t Led2;

/*!
 * Example of a GPIO interrupt
 * testIrq can be used externally to react to interrupts.
 */
Gpio_t testGpio;
uint8_t testIrq = 0;
void testFunction(void *context);

/*
 * MCU objects
 */
Uart_t Uart1;
volatile uint32_t sysTickCounter;
/* Countdown can be used for delays or timers */
volatile uint32_t sysTickCountDown;

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit(void);

/*!
 * System Clock Configuration
 */
static void SystemClockConfig(void);

/*!
 * Used to measure and calibrate the system wake-up time from STOP mode
 */
static void CalibrateSystemWakeupTime(void);

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig(void);

/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
static volatile bool SystemWakeupTimeCalibrated = false;

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
static void OnCalibrateSystemWakeupTimeTimerEvent(void *context) {
	//TODO: Check function
	RtcSetMcuWakeUpTime();
	SystemWakeupTimeCalibrated = true;
}

void BoardCriticalSectionBegin(uint32_t *mask) {
	*mask = __get_PRIMASK();
	__disable_irq();
}

void BoardCriticalSectionEnd(uint32_t *mask) {
	__set_PRIMASK(*mask);
}

void BoardInitPeriph(void) {

}

void BoardInitMcu(void) {
	if (McuInitialized == false) {

		/*!
		 * Initialize the pins and their functions from pin_mux.c
		 */
		BOARD_InitPins();
		SystemClockConfig();

		/*!
		 * Configure your terminal for 8 Bits data (7 data bit + 1 parity bit), no parity and no flow ctrl
		 */
		UartInit(&Uart1, UART_2, DEBUG_UART_FAKE_PIN, DEBUG_UART_FAKE_PIN);
		UartConfig(&Uart1, RX_TX, DEBUG_UART_BAUDRATE, UART_8_BIT,
				UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL);

		/*!
		 * Initialize the GPIOs and LEDs
		 */
		GpioInit(&Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
		GpioInit(&Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1);
		GpioWrite(&Led1, 0);
		GpioWrite(&Led2, 0);

		/*!
		 * Example of a GPIO initialized as interrupt. See also pin_mux.c
		 */
		GpioInit(&testGpio, PTD_5, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
		GpioSetInterrupt(&testGpio, IRQ_FALLING_EDGE, IRQ_HIGH_PRIORITY,
				&testFunction);

		/*!
		 * Initialize the GPIOs and LEDs
		 */
		RtcInit();

		BoardUnusedIoInit();
	} else {
		SystemClockReConfig();
	}

	//TODO: Integration of SPI
	//SpiInit( &SX126x.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS );

	//TODO: Integration of SX1262
	//SX126xIoInit( );

	if (McuInitialized == false) {
		McuInitialized = true;
		//TODO: Integration of SX1262
		//SX126xIoDbgInit( );
		//SX126xIoTcxoInit( );
	}
}

void BoardResetMcu(void) {
	CRITICAL_SECTION_BEGIN();

	//Restart system
	//TODO: Integrate the K22 CMSIS and reset the system here.
	//NVIC_SystemReset( );
}

void BoardDeInitMcu(void) {
	//TODO: Integration of SPI
	//SpiDeInit( &SX1276.Spi );
	//TODO: Integration of SX1262
	//SX126xIoDeInit( );
}

uint32_t BoardGetRandomSeed(void) {
	//TODO: Integration of random from SE050
	return 0;
}

void BoardGetUniqueId(uint8_t *id) {
	//TODO: Integration of id from SE050
}

static void BoardUnusedIoInit(void) {
	/*!
	 * Nothing to initialize as of yet
	 */
	return;
}

void SystemClockConfig(void) {
	BOARD_InitBootClocks();

	sysTickCounter = 0;
	/* Set systick reload value to generate 1ms interrupt */
	SysTick_Config(SystemCoreClock / 1000U);
}

void CalibrateSystemWakeupTime(void) {
	if (SystemWakeupTimeCalibrated == false) {
		TimerInit(&CalibrateSystemWakeupTimeTimer,
				OnCalibrateSystemWakeupTimeTimerEvent);
		TimerSetValue(&CalibrateSystemWakeupTimeTimer, 1000);
		TimerStart(&CalibrateSystemWakeupTimeTimer);
		while (SystemWakeupTimeCalibrated == false) {

		}
	}
}

void SystemClockReConfig(void) {
	//TODO: Check what to do here and if needed
}

void SysTick_Handler(void) {
	sysTickCounter++;
	if (sysTickCountDown != 0U) {
		sysTickCountDown--;
	}
}

/*
 * Function to be used by stdout for printf etc
 */
int _write(int fd, const void *buf, size_t count) {
	while (UartPutBuffer(&Uart1, (uint8_t*) buf, (uint16_t) count) != 0) {
	};
	return count;
}

/*
 * Function to be used by stdin for scanf etc
 */
int _read(int fd, const void *buf, size_t count) {
	size_t bytesRead = 0;
	while (UartGetBuffer(&Uart1, (uint8_t*) buf, count, (uint16_t*) &bytesRead)
			!= 0) {
	};
	// Echo back the character
	while (UartPutBuffer(&Uart1, (uint8_t*) buf, (uint16_t) bytesRead) != 0) {
	};
	return bytesRead;
}

/*!
 * Example of a GPIO interrupt
 * This is the callback method which sets testIrq to 1 then.
 * testIrq can be used externally to react to interrupts.
 */
void testFunction(void *context) {
	testIrq = 1;
}
