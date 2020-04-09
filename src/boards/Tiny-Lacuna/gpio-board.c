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

static Gpio_t *GpioIrqA[20];
static Gpio_t *GpioIrqB[20];
static Gpio_t *GpioIrqC[20];
static Gpio_t *GpioIrqD[20];
static Gpio_t *GpioIrqE[20];

static void ClearGpioInterruptFlag(GPIO_Type *base, uint32_t mask);

void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{

    if( pin >= PTA_0 ){
        gpio_pin_config_t gpio_config; 
        obj->pin = pin;

        obj->pinIndex = ( 0x01 << ( obj->pin & 0x1F ) );

        if( ( obj->pin & 0xE0 ) == 0x00 )
        {
            obj->port = GPIOA;
            /* Port A Clock Gate Control: Clock enabled */
            CLOCK_EnableClock(kCLOCK_PortA);
        }
        else if( ( obj->pin & 0xE0 ) == 0x10 )
        {
            obj->port = GPIOB;
            /* Port B Clock Gate Control: Clock enabled */
            CLOCK_EnableClock(kCLOCK_PortB);
        }
        else if( ( obj->pin & 0xE0 ) == 0x20 )
        {
            obj->port = GPIOC;
            /* Port C Clock Gate Control: Clock enabled */
            CLOCK_EnableClock(kCLOCK_PortC);
        }
        else if( ( obj->pin & 0xE0 ) == 0x30 )
        {
            obj->port = GPIOD;
            /* Port D Clock Gate Control: Clock enabled */
            CLOCK_EnableClock(kCLOCK_PortD);
        }
        else if( ( obj->pin & 0xE0 ) == 0x40 )
        {
            obj->port = GPIOE;
            /* Port E Clock Gate Control: Clock enabled */
            CLOCK_EnableClock(kCLOCK_PortE);
        }
        else
        {
            for(;;){
                /*!
                 * You should not reach this state.
                 */
            }
        }

        if( mode < PIN_OUTPUT){
            for(;;){
                /*!
                 * You should not reach this state.
                 */
            }
        }

        gpio_config.pinDirection = mode;

        // Sets initial output value
        if( mode == PIN_OUTPUT )
        {
            GpioMcuWrite( obj, value );
        }
        
        GPIO_PinInit(obj->port, 0x1 << obj->pinIndex, &gpio_config);

    } 
    else if( pin == NC )
    {
        return;
    }
}

void GpioMcuSetContext( Gpio_t *obj, void* context )
{
    obj->Context = context;
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    if( obj->pin >= PTA_0 )
    {   
        uint32_t irqConfig;
        gpio_pin_config_t gpio_config;

       

        obj->IrqHandler = irqHandler;

        if (irqMode == NO_IRQ){
            irqConfig = 0;
        }else
        {
            irqConfig = irqMode + 8;
        }
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
        GPIO_SetPinInterruptConfig(obj->port, 0x1 << obj->pinIndex, irqConfig);
        GPIO_GpioClearInterruptFlags(obj->port, 0x1 << obj->pinIndex);
#else
        PORT_SetPinInterruptConfig(obj->port, 0x1 << obj->pinIndex, irqConfig);
        PORT_ClearPinsInterruptFlags(obj->port, 0x1 << obj->pinIndex);
#endif
        
        gpio_config.pinDirection = kGPIO_DigitalInput;
        GPIO_PinInit(obj->port, 0x1 << obj->pinIndex, &gpio_config);

        if(obj->port == GPIOA)
        {
            GpioIrqA[obj->pinIndex] = obj;
            EnableIRQ(PORTA_IRQn);
        }
        else if( obj->port == GPIOB )
        {
            GpioIrqB[obj->pinIndex] = obj;
            EnableIRQ(PORTB_IRQn);
        }
        else if( obj->port == GPIOC )
        {
            GpioIrqC[obj->pinIndex] = obj;
            EnableIRQ(PORTC_IRQn);
        }
        else if( obj->port == GPIOD )
        {
            GpioIrqD[obj->pinIndex] = obj;
            EnableIRQ(PORTD_IRQn);
        }
        else if( obj->port == GPIOE )
        {
            GpioIrqE[obj->pinIndex] = obj;
            EnableIRQ(PORTE_IRQn);
        }
    }
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    if( obj->pin >= PTA_0 )
    {
        gpio_pin_config_t gpio_config;

        // Clear callback before changing pin mode
        if(obj->port == GPIOA)
        {
            GpioIrqA[obj->pinIndex] = 0x00;
        }
        else if( obj->port == GPIOB )
        {
            GpioIrqB[obj->pinIndex] = 0x00;
        }
        else if( obj->port == GPIOC )
        {
            GpioIrqC[obj->pinIndex] = 0x00;
        }
        else if( obj->port == GPIOD )
        {
            GpioIrqD[obj->pinIndex] = 0x00;
        }
        else if( obj->port == GPIOE )
        {
            GpioIrqE[obj->pinIndex] = 0x00;
        }

#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
        GPIO_SetPinInterruptConfig(obj->port, 0x1 << obj->pinIndex, kPORT_InterruptOrDMADisabled);
#else
        PORT_SetPinInterruptConfig(obj->port, 0x1 << obj->pinIndex, kPORT_InterruptOrDMADisabled);
#endif
        gpio_config.pinDirection = kGPIO_DigitalOutput;
        GPIO_PinInit(obj->port, 0x1 << obj->pinIndex, &gpio_config);
    }
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
    if( obj->pin >= PTA_0 )
    {
        if( obj == NULL )
        {
            for(;;){
                /*!
                 * You should not reach this state.
                 */
            }
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        GPIO_PinWrite(obj->port, 0x1 << obj->pinIndex, (uint8_t)value);
    }
}

void GpioMcuToggle( Gpio_t *obj )
{
    if( obj->pin >= PTA_0 )
    {
        if( obj == NULL )
        {
            for(;;){
                /*!
                 * You should not reach this state.
                 */
            }
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        GPIO_PortToggle(obj->port, 0x1 << obj->pinIndex);
    }
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
    if( obj->pin >= PTA_0 )
    {
        if( obj == NULL )
        {
            for(;;){
                /*!
                 * You should not reach this state.
                 */
            }
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        return GPIO_PinRead(obj->port, obj->pinIndex);
    }
}

void BOARD_PORTA_IRQ_HANDLER( void )
{
    uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOA);
    ClearGpioInterruptFlag(GPIOA, 0x1 << irqIndex);

    if( ( GpioIrqA[irqIndex] != NULL ) && ( GpioIrqA[irqIndex]->IrqHandler != NULL ) )
    {
        GpioIrqA[irqIndex]->IrqHandler( GpioIrqA[irqIndex]->Context );
    }
}

void BOARD_PORTB_IRQ_HANDLER( void )
{
    uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOB);
    ClearGpioInterruptFlag(GPIOB, 0x1 << irqIndex);

    if( ( GpioIrqB[irqIndex] != NULL ) && ( GpioIrqB[irqIndex]->IrqHandler != NULL ) )
    {
        GpioIrqB[irqIndex]->IrqHandler( GpioIrqB[irqIndex]->Context );
    }
}

void BOARD_PORTC_IRQ_HANDLER( void )
{
    uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOC);
    ClearGpioInterruptFlag(GPIOC, 0x1 << irqIndex);

    if( ( GpioIrqC[irqIndex] != NULL ) && ( GpioIrqC[irqIndex]->IrqHandler != NULL ) )
    {
        GpioIrqC[irqIndex]->IrqHandler( GpioIrqC[irqIndex]->Context );
    }
}

void BOARD_PORTD_IRQ_HANDLER( void )
{
    uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOD);
    ClearGpioInterruptFlag(GPIOD, 0x1 << irqIndex);

    if( ( GpioIrqD[irqIndex] != NULL ) && ( GpioIrqD[irqIndex]->IrqHandler != NULL ) )
    {
        GpioIrqD[irqIndex]->IrqHandler( GpioIrqD[irqIndex]->Context );
    }
}

void BOARD_PORTE_IRQ_HANDLER( void )
{
    uint32_t irqIndex = GPIO_PortGetInterruptFlags(GPIOE);
    ClearGpioInterruptFlag(GPIOE, 0x1 << irqIndex);

    if( ( GpioIrqE[irqIndex] != NULL ) && ( GpioIrqE[irqIndex]->IrqHandler != NULL ) )
    {
        GpioIrqE[irqIndex]->IrqHandler( GpioIrqE[irqIndex]->Context );
    }
}

static void ClearGpioInterruptFlag(GPIO_Type *base, uint32_t mask){
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    GPIO_GpioClearInterruptFlags(base, mask);
#else
    PORT_ClearPinsInterruptFlags(base, mask);
#endif
}
