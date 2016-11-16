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
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32l1xx.h"
#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "gpio.h"
#include "spi.h"
#include "uart.h"
#include "radio.h"
#include "sx1276/sx1276.h"
#include "rtc-board.h"
#include "timer-board.h"
#include "sx1276-board.h"
#include "uart-board.h"
#include "m100a-board.h"
#include "loramac.h"

#if defined( USE_USB_CDC )
#include "usb-cdc-board.h"
#endif



/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0  
#endif


/*!
 * Board MCU pins definitions
 */
//       Define                                     GPIO
#define STM8_CTS																		PA_0
#define STM8_RTS																		PA_1
#define USART1_TX																		PA_2
#define USART1_RX																		PA_3

#define SPI1_NSS                                    PA_4
#define SPI1_SCLK                                   PA_5
#define SPI1_MISO                                   PA_6
#define SPI1_MOSI                                   PA_7

#define IO_0                                        PA_8
#define IO_1                                        PA_9	// RX_LED read
#define IO_2                                        PA_10 // TX_LED yellow

#define USB_DM                                      PA_11
#define USB_DP                                      PA_12

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14

#define RADIO_RESET                                 PA_15

#define RADIO_DIO_0                                 PB_0
#define RADIO_DIO_1                                 PB_1
#define RADIO_DIO_2                                 PB_2
#define RADIO_DIO_3                                 PB_3
#define RADIO_DIO_4                                 PB_4
#define RADIO_DIO_5                                 PB_5

#define I2C_SCL                                     PB_6
#define I2C_SDA                                     PB_7 

#define RADIO_ANT_SWITCH_LF                         PB_8

#define MODE_SET																		PB_9

#define USART3_TX																		PB_10
#define USART3_RX																		PB_11

#define RADIO_NSS                                   PB_12
#define RADIO_SCLK                                  PB_13
#define RADIO_MISO                                  PB_14
#define RADIO_MOSI                                  PB_15
                                                                
#define WKUP2                                       PC_13    

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

/*!
 * MCU objects
 */
extern Uart_t Uart2;
#if defined( USE_USB_CDC )
extern Uart_t UartUsb;
#endif

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t BoardGetBatteryLevel( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * \brief Gets the board 64 bits unique ID 
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

#endif // __BOARD_H__
