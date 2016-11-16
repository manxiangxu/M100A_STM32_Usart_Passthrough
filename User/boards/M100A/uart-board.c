/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Bleeper board UART driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "board.h"

#include "uart-board.h"

void UartMcuInit( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx )
{
    obj->UartId = uartId;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2, ENABLE );

    USART_DeInit( USART2 );

    GpioInit( &obj->Tx, tx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    GpioInit( &obj->Rx, rx, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );

    GPIO_PinAFConfig( obj->Tx.port, ( obj->Tx.pin & 0x0F ), GPIO_AF_USART2 );
    GPIO_PinAFConfig( obj->Rx.port, ( obj->Rx.pin & 0x0F ), GPIO_AF_USART2 );
}

void UartMcuConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    if( mode == TX_ONLY )
    {
        if( obj->FifoTx.Data == NULL )
        {
            while( 1 );
        }

        USART_InitStructure.USART_Mode = USART_Mode_Tx;
    }
    else if( mode == RX_ONLY )
    {
        if( obj->FifoRx.Data == NULL )
        {
            while( 1 );
        }

        USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );
        USART_InitStructure.USART_Mode = USART_Mode_Rx;
    }
    else
    {
        if( ( obj->FifoTx.Data == NULL ) || ( obj->FifoRx.Data == NULL ) )
        {
            while( 1 );
        }

        USART_ITConfig( USART2, USART_IT_RXNE, ENABLE );
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    }

    USART_InitStructure.USART_BaudRate = baudrate;

    if( wordLength == UART_8_BIT )
    {
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }
    else 
    {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }

    if( stopBits == UART_1_STOP_BIT )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    }
    else if( stopBits == UART_0_5_STOP_BIT )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_0_5;
    }
    else if( stopBits == UART_2_STOP_BIT )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    }
    else if( stopBits == UART_1_5_STOP_BIT )
    {
        USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
    }

    if( parity == NO_PARITY )
    {
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    else if( parity == EVEN_PARITY )
    {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    }
    else
    {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    }

    if( flowCtrl == NO_FLOW_CTRL )
    {
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    }
    else if( flowCtrl == RTS_FLOW_CTRL ) 
    {
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS;
    }
    else if( flowCtrl == CTS_FLOW_CTRL ) 
    {
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_CTS;
    }
    else if( flowCtrl == RTS_CTS_FLOW_CTRL ) 
    {
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
    }

    USART_Init( USART2, &USART_InitStructure );

    USART_Cmd( USART2, DISABLE );
}

void UartMcuDeInit( Uart_t *obj )
{
    USART_DeInit( USART2 );

    GpioInit( &obj->Tx, obj->Tx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Rx, obj->Rx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

uint8_t UartMcuPutChar( Uart_t *obj, uint8_t data )
{
    if( IsFifoFull( &obj->FifoTx ) == false )
    {
        __disable_irq( );
        FifoPush( &obj->FifoTx, data );
        __enable_irq( );
        // Enable the USART Transmit interrupt
        USART_ITConfig( USART2, USART_IT_TXE, ENABLE );
        return 0; // OK
    }
    return 1; // Busy
}

uint8_t UartMcuGetChar( Uart_t *obj, uint8_t *data )
{
    if( IsFifoEmpty( &obj->FifoRx ) == false )
    {
        __disable_irq( );
        *data = FifoPop( &obj->FifoRx );
        __enable_irq( );
        return 0;
    }
    return 1;
}

void USART2_IRQHandler( void )
{
    uint8_t data;

    if( USART_GetITStatus( USART2, USART_IT_TXE ) != RESET )
    {    
        if( IsFifoEmpty( &Uart2.FifoTx ) == false )
        {
            data = FifoPop( &Uart2.FifoTx );
            //  Write one byte to the transmit data register 
            USART_SendData( USART2, data );
        }
        else
        {
            // Disable the USART Transmit interrupt
            USART_ITConfig( USART2, USART_IT_TXE, DISABLE );
        }
        if( Uart2.IrqNotify != NULL )
        {
            Uart2.IrqNotify( UART_NOTIFY_TX );
        }
    }

    if( USART_GetITStatus( USART2, USART_IT_ORE_RX ) != RESET )
    {
        USART_ReceiveData( USART2 );
    }

    if( USART_GetITStatus( USART2, USART_IT_RXNE ) != RESET )
    {    
        data = USART_ReceiveData( USART2 );
        if( IsFifoFull( &Uart2.FifoRx ) == false )
        {
            // Read one byte from the receive data register
            FifoPush( &Uart2.FifoRx, data );
        }
        if( Uart2.IrqNotify != NULL )
        {
            Uart2.IrqNotify( UART_NOTIFY_RX );
        }
    }
}
