/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation
 *
 * \author    Diego Bienz
 */

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board-config.h"
#include "rtc-board.h"
#include "gpio-board.h"
#include "utilities.h"

/*!
 * Arrays for registering interrupts for every single GPIO
 */
static Gpio_t *GpioIrqA[20];
static Gpio_t *GpioIrqB[20];
static Gpio_t *GpioIrqC[20];
static Gpio_t *GpioIrqD[20];
static Gpio_t *GpioIrqE[20];

static void ClearGpioInterruptFlag(GPIO_Type *base, uint32_t mask);
static uint32_t PinNumberFromMask(uint32_t mask);

/*!
 * CAUTION: Configurations like Pin-Muxing, Pull-up/Pull-down and Clocks need to be preconfigured in BOARD_InitPins
 */
void GpioMcuInit(Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config,
		PinTypes type, uint32_t value) {

	if (pin >= PTA_0) {
		gpio_pin_config_t gpio_config;
		obj->pin = pin;

		obj->pinIndex = (obj->pin & 0x1F);

		if ((obj->pin & 0x0E0) == 0x00) {
			obj->port = GPIOA;
			/* Port A Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortA);
		} else if ((obj->pin & 0x0E0) == 0x20) {
			obj->port = GPIOB;
			/* Port B Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortB);
		} else if ((obj->pin & 0x0E0) == 0x40) {
			obj->port = GPIOC;
			/* Port C Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortC);
		} else if ((obj->pin & 0x0E0) == 0x60) {
			obj->port = GPIOD;
			/* Port D Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortD);
		} else if ((obj->pin & 0x0E0) == 0x80) {
			obj->port = GPIOE;
			/* Port E Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortE);
		} else {
			for (;;) {
				/*!
				 * You should not reach this state.
				 */
			}
		}

		gpio_config.pinDirection = mode;

		// Sets initial output value
		if (mode == PIN_OUTPUT) {
			GpioMcuWrite(obj, value);
		}

		GPIO_PinInit(obj->port, obj->pinIndex, &gpio_config);

	} else if (pin == NC) {
		return;
	}
}

void GpioMcuSetContext(Gpio_t *obj, void *context) {
	obj->Context = context;
}

void GpioMcuSetInterrupt(Gpio_t *obj, IrqModes irqMode,
		IrqPriorities irqPriority, GpioIrqHandler *irqHandler) {
	uint32_t irqConfig;
	gpio_pin_config_t gpio_config;

	gpio_config.pinDirection = 0;

	if (obj->pin >= PTA_0) {
		obj->IrqHandler = irqHandler;

		if (irqMode == NO_IRQ) {
			irqConfig = 0;
		} else {
			irqConfig = irqMode + 8;
		}

		if ((obj->pin & 0x0E0) == 0x00) {
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
			obj->port = GPIOA;
#else
			obj->port = PORTA;
#endif
			/* Port A Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortA);
		} else if ((obj->pin & 0x0E0) == 0x20) {
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
			obj->port = GPIOB;
#else
			obj->port = PORTB;
#endif			/* Port B Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortB);
		} else if ((obj->pin & 0x0E0) == 0x40) {
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
			obj->port = GPIOC;
#else
			obj->port = PORTC;
#endif			/* Port C Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortC);
		} else if ((obj->pin & 0x0E0) == 0x60) {
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
			obj->port = GPIOD;
#else
			obj->port = PORTD;
#endif			/* Port D Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortD);
		} else if ((obj->pin & 0x0E0) == 0x80) {
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
			obj->port = GPIOE;
#else
			obj->port = PORTE;
#endif			/* Port E Clock Gate Control: Clock enabled */
			CLOCK_EnableClock(kCLOCK_PortE);
		} else {
			for (;;) {
				/*!
				 * You should not reach this state.
				 */
			}
		}

#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
        GPIO_SetPinInterruptConfig(obj->port, obj->pinIndex, irqConfig);
#else
		PORT_SetPinInterruptConfig(obj->port, obj->pinIndex, irqConfig);
#endif

		GPIO_PinInit(obj->port, obj->pinIndex, &gpio_config);

		if (obj->port == GPIOA || obj->port == PORTA) {
			GpioIrqA[obj->pinIndex] = obj;
			EnableIRQ(PORTA_IRQn);
		} else if (obj->port == GPIOB || obj->port == PORTB) {
			GpioIrqB[obj->pinIndex] = obj;
			EnableIRQ(PORTB_IRQn);
		} else if (obj->port == GPIOC || obj->port == PORTC) {
			GpioIrqC[obj->pinIndex] = obj;
			EnableIRQ(PORTC_IRQn);
		} else if (obj->port == GPIOD || obj->port == PORTD) {
			GpioIrqD[obj->pinIndex] = obj;
			EnableIRQ(PORTD_IRQn);
		} else if (obj->port == GPIOE || obj->port == PORTE) {
			GpioIrqE[obj->pinIndex] = obj;
			EnableIRQ(PORTE_IRQn);
		}
	}
}

void GpioMcuRemoveInterrupt(Gpio_t *obj) {
	if (obj->pin >= PTA_0) {
		gpio_pin_config_t gpio_config;

		// Clear callback before changing pin mode
		if (obj->port == GPIOA || obj->port == PORTA) {
			GpioIrqA[obj->pinIndex] = 0x00;
		} else if (obj->port == GPIOB || obj->port == PORTB) {
			GpioIrqB[obj->pinIndex] = 0x00;
		} else if (obj->port == GPIOC || obj->port == PORTC) {
			GpioIrqC[obj->pinIndex] = 0x00;
		} else if (obj->port == GPIOD || obj->port == PORTD) {
			GpioIrqD[obj->pinIndex] = 0x00;
		} else if (obj->port == GPIOE || obj->port == PORTE) {
			GpioIrqE[obj->pinIndex] = 0x00;
		}

#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
        GPIO_SetPinInterruptConfig(obj->port, obj->pinIndex, kPORT_InterruptOrDMADisabled);
#else
		PORT_SetPinInterruptConfig(obj->port, obj->pinIndex,
				kPORT_InterruptOrDMADisabled);
#endif
		gpio_config.pinDirection = kGPIO_DigitalOutput;
		GPIO_PinInit(obj->port, obj->pinIndex, &gpio_config);
	}
}

void GpioMcuWrite(Gpio_t *obj, uint32_t value) {
	if (obj->pin >= PTA_0) {
		if (obj == NULL) {
			for (;;) {
				/*!
				 * You should not reach this state.
				 */
			}
		}
		// Check if pin is not connected
		if (obj->pin == NC) {
			return;
		}
		GPIO_PinWrite(obj->port, obj->pinIndex, (uint8_t) value);
	}
}

void GpioMcuToggle(Gpio_t *obj) {
	if (obj->pin >= PTA_0) {
		if (obj == NULL) {
			for (;;) {
				/*!
				 * You should not reach this state.
				 */
			}
		}
		// Check if pin is not connected
		if (obj->pin == NC) {
			return;
		}
		GPIO_PortToggle(obj->port, obj->pinIndex);
	}
}

uint32_t GpioMcuRead(Gpio_t *obj) {
	if (obj->pin >= PTA_0) {
		if (obj == NULL) {
			for (;;) {
				/*!
				 * You should not reach this state.
				 */
			}
		}
		// Check if pin is not connected
		if (obj->pin == NC) {
			return;
		}
		return GPIO_PinRead(obj->port, obj->pinIndex);
	}
}

/*!
 * Default IRQ callback methods for every GPIO bank.
 * The methods can be overriden in board-config.h
 *
 * There is an array with function pointers for every
 * GPIO bank where the pointers to the callback functions
 * are stored. The corresponding function pointer to the
 * raised interrupt is called in the methods below.
 *
 */
void BOARD_PORTA_IRQ_HANDLER(void) {
	uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOA);
	ClearGpioInterruptFlag(GPIOA, irqIndex);

	irqIndex = PinNumberFromMask(irqIndex);

	if ((GpioIrqA[irqIndex] != NULL)
			&& (GpioIrqA[irqIndex]->IrqHandler != NULL)) {
		GpioIrqA[irqIndex]->IrqHandler(GpioIrqA[irqIndex]->Context);
	}
}

void BOARD_PORTB_IRQ_HANDLER(void) {
	uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOB);
	ClearGpioInterruptFlag(GPIOB, irqIndex);

	irqIndex = PinNumberFromMask(irqIndex);

	if ((GpioIrqB[irqIndex] != NULL)
			&& (GpioIrqB[irqIndex]->IrqHandler != NULL)) {
		GpioIrqB[irqIndex]->IrqHandler(GpioIrqB[irqIndex]->Context);
	}
}

void BOARD_PORTC_IRQ_HANDLER(void) {
	uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOC);
	ClearGpioInterruptFlag(GPIOC, irqIndex);

	irqIndex = PinNumberFromMask(irqIndex);

	if ((GpioIrqC[irqIndex] != NULL)
			&& (GpioIrqC[irqIndex]->IrqHandler != NULL)) {
		GpioIrqC[irqIndex]->IrqHandler(GpioIrqC[irqIndex]->Context);
	}
}

void BOARD_PORTD_IRQ_HANDLER(void) {
	uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOD);
	ClearGpioInterruptFlag(GPIOD, irqIndex);

	irqIndex = PinNumberFromMask(irqIndex);

	if ((GpioIrqD[irqIndex] != NULL)
			&& (GpioIrqD[irqIndex]->IrqHandler != NULL)) {
		GpioIrqD[irqIndex]->IrqHandler(GpioIrqD[irqIndex]->Context);
	}
}

void BOARD_PORTE_IRQ_HANDLER(void) {
	uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOE);
	ClearGpioInterruptFlag(GPIOE, irqIndex);

	irqIndex = PinNumberFromMask(irqIndex);

	if ((GpioIrqE[irqIndex] != NULL)
			&& (GpioIrqE[irqIndex]->IrqHandler != NULL)) {
		GpioIrqE[irqIndex]->IrqHandler(GpioIrqE[irqIndex]->Context);
	}
}

static void ClearGpioInterruptFlag(GPIO_Type *base, uint32_t mask) {
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    GPIO_GpioClearInterruptFlags(base, mask);
#else
	GPIO_PortClearInterruptFlags(base, mask);
#endif
}

static uint32_t PinNumberFromMask(uint32_t mask) {
	uint32_t counter = -1;
	while (mask != 0) {
		mask = mask >> 1;
		counter++;
	}
	return counter;
}
