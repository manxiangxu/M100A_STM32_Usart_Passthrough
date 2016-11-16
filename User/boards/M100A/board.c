/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian and Hellogz
*/
#include "board.h"

/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )


/*
 * MCU objects
 */
Uart_t Uart2;
#if defined( USE_USB_CDC )
Uart_t UartUsb;
#endif

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

void BoardInitPeriph( void )
{
	
}

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {
#if defined( USE_BOOTLOADER )
        // Set the Vector Table base location at 0x3000
        NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x3000 );
#endif
        // We use IRQ priority group 4 for the entire project
        // When setting the IRQ, only the preemption priority is used
        NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

        // Disable Systick
        SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk;    // Systick IRQ off 
        SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;            // Clear SysTick Exception pending flag
        SpiInit( &SX1276.Spi, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
        SX1276IoInit( );

#if defined( LOW_POWER_MODE_ENABLE )
        TimerSetLowPowerEnable( true );
#else
        TimerSetLowPowerEnable( false );
#endif
        BoardUnusedIoInit( );
								
        if( TimerGetLowPowerEnable( ) == true )
        {
            RtcInit( );
        }
        else
        {
            TimerHwInit( );
        }
				
        McuInitialized = true;
    }
}

void BoardDeInitMcu( void )
{
    Gpio_t ioPin;

    SpiDeInit( &SX1276.Spi );
    SX1276IoDeInit( );

    GpioInit( &ioPin, OSC_HSE_IN, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, OSC_HSE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, OSC_LSE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, OSC_LSE_OUT, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	
    McuInitialized = false;
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

uint8_t BoardGetBatteryLevel( void )
{
	return 255;	
}

static void BoardUnusedIoInit( void )
{
    Gpio_t ioPin;
  
    /* USB */
#if !defined( USE_USB_CDC )
    GpioInit( &ioPin, USB_DM, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, USB_DP, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
    GpioInit( &ioPin, WKUP2, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 ); 
    GpioInit( &ioPin, IO_0, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		GpioInit( &ioPin, I2C_SCL, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		GpioInit( &ioPin, I2C_SDA, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		GpioInit( &ioPin, STM8_CTS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		GpioInit( &ioPin, SPI1_MOSI, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		GpioInit( &ioPin, SPI1_MISO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		GpioInit( &ioPin, SPI1_SCLK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	  GpioInit( &ioPin, SPI1_NSS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#if defined( USE_DEBUGGER )
    DBGMCU_Config( DBGMCU_SLEEP, ENABLE );
    DBGMCU_Config( DBGMCU_STOP, ENABLE);
    DBGMCU_Config( DBGMCU_STANDBY, ENABLE);
#else
    DBGMCU_Config( DBGMCU_SLEEP, DISABLE );
    DBGMCU_Config( DBGMCU_STOP, DISABLE );
    DBGMCU_Config( DBGMCU_STANDBY, DISABLE );
    
    GpioInit( &ioPin, SWDIO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SWCLK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif    
}
