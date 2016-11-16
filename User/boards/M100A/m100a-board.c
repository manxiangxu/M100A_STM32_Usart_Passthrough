/*
2016 Winext

Description: M100A Usart Passthrough implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Hellogz
*/
#include "m100a-board.h"
#include "stm32_eeprom_lib.h"
#include "non-lorawan_passthrough.h"


#ifdef __GNUC__
	/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
	set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

// Usart config defined
#define COM                             USART3
#define COM_CLK                         RCC_APB1Periph_USART3

#define COM_TX_GPIO_CKL                 RCC_AHBPeriph_GPIOB
#define COM_TX_PIN                      GPIO_Pin_10
#define COM_TX_GPIO_PORT                GPIOB
#define COM_TX_SOURCE                   GPIO_PinSource10
#define COM_TX_AF                       GPIO_AF_USART3

#define COM_RX_GPIO_CKL                 RCC_AHBPeriph_GPIOB
#define COM_RX_PIN                      GPIO_Pin_11
#define COM_RX_GPIO_PORT                GPIOB
#define COM_RX_SROUCE                   GPIO_PinSource11
#define COM_RX_AF                       GPIO_AF_USART3

/* USARTx configured as follow:
- BaudRate = 115200 baud  
- Word Length = 8 Bits
- One Stop Bit
- No parity
- Hardware flow control disabled (RTS and CTS signals)
- Receive and transmit enabled
*/
#define USART_BAUDRATE                  115200
#define USART_WORDLENGTH                USART_WordLength_8b
#define USART_STOPBITS   								USART_StopBits_1
#define USART_PARITY                    USART_Parity_No
#define USART_HARDWAREFLOWCONTROL       USART_HardwareFlowControl_None
#define USART_MODE                      USART_Mode_Tx



static uint8_t tx_buffer[FIFO_TX_SIZE];
static uint8_t rx_buffer[FIFO_RX_SIZE];

static uint8_t buffer[255];
static uint8_t buffer_size = 0;

Gpio_t usart_rts;

uint8_t uart_buffer[SERIAL_PACKAGE_MAX_SIZE];
uint16_t uart_buffer_pkt_len = 0;

ConfigParam_t ConfigParam_malloc = {0};
ConfigParam_t *ConfigParam = &ConfigParam_malloc;

bool usart_received_data_flag = false;

enum ParseErrorCode parse_serial_frame(uint8_t *frame, uint8_t frame_length);

Gpio_t led_red;
Gpio_t led_yellow;

void led_blink(void)
{
	GpioInit(&led_red, IO_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);
	GpioInit(&led_yellow, IO_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1);

	Delay(1);

	GpioWrite(&led_red, 0);
	GpioWrite(&led_yellow, 0);
}

/**
  * @brief  Config the USART.
  * @param  None
  * @retval None
  */
void debug_printf_com_config(void)
{
	USART_InitTypeDef 	USART_InitStructure;
	GPIO_InitTypeDef 	GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(COM_TX_GPIO_CKL | COM_RX_GPIO_CKL, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(COM_TX_GPIO_PORT, COM_TX_SOURCE, COM_TX_AF);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(COM_RX_GPIO_PORT, COM_RX_SROUCE, COM_RX_AF);

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = COM_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(COM_TX_GPIO_PORT, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = COM_RX_PIN;
	GPIO_Init(COM_TX_GPIO_PORT, &GPIO_InitStructure);

	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(COM_CLK, ENABLE);

	USART_InitStructure.USART_BaudRate = USART_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WORDLENGTH;
	USART_InitStructure.USART_StopBits = USART_STOPBITS;
	USART_InitStructure.USART_Parity = USART_PARITY;
	USART_InitStructure.USART_HardwareFlowControl = USART_HARDWAREFLOWCONTROL;
	USART_InitStructure.USART_Mode = USART_MODE;

	/* USART configuration */
	USART_Init(COM, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(COM, ENABLE);
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	if(ConfigParam->EnableDebugPrintf)
	{
		/* Place your implementation of fputc here */
		/* e.g. write a character to the USART */
		USART_SendData(COM, (uint8_t) ch);

		/* Loop until transmit data register is empty */
		while (USART_GetFlagStatus(COM, USART_FLAG_TXE) == RESET)
		{}
	}
	return ch;
}

void print_message(const char *title, uint8_t *message, uint8_t length, uint8_t format)
{
	DBG(title);
	if(length == 0) 
	{ 
		DBG("[]\r\n");  
	}
	else
	{
		DBG("%dB\r\n[", length);
		for(uint8_t i = 0; i < (length - 1); i++)
		{
			if(format == 'X')
			{
				DBG("%02X ", message[i]);
			}
			else if(format == 'x')
			{
				DBG("%02x ", message[i]);
			}
			else if(format == 'd')
			{
				DBG("%d ", message[i]);
			}
			else
			{
				DBG("%d ", message[i]);
			}
		}
		if(format == 'X')
		{
			DBG("%02X]\r\n", message[length - 1]);
		}
		else if(format == 'x')
		{
			DBG("%02x]\r\n", message[length - 1]);
		}
		else if(format == 'd')
		{
			DBG("%d]\r\n", message[length - 1]);
		}
		else
		{
			DBG("%d]\r\n", message[length - 1]);
		}
	}
}

void lorawan_use_config_param(uint8_t *param)
{
	uint16_t index = 0;
	
	index = 0;
	memcpy1(ConfigParam->DevEui, param+index, 8);
	index += 8;
	ConfigParam->Activation = param[index];
	index += 1;
	memcpy1(ConfigParam->AppEui, param+index, 8);
	index += 8;
	memcpy1(ConfigParam->AppKey, param+index, 16);
	index += 16;
	memcpy1(ConfigParam->NwkSKey, param+index, 16);
	index += 16;
	memcpy1(ConfigParam->AppSKey, param+index, 16);
	index += 16;
	memcpyr((uint8_t *)&ConfigParam->DevAddr, param+index, 4);	//to little endian
	index += 4;
	memcpy1((uint8_t *)&ConfigParam->Channel_0, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_1, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_2, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_3, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_4, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_5, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_6, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_7, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_8, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_9, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_10, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_11, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_12, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_13, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_14, param+index, 6);
	index += 6;
	memcpy1((uint8_t *)&ConfigParam->Channel_15, param+index, 6);
	index += 6;
	ConfigParam->NetworkType = param[index];
	index += 1;
	memcpy1((uint8_t *)&ConfigParam->Rx2Channel, param+index, 5);
	index += 5;
	memcpy1((uint8_t *)&ConfigParam->ChannelsMask, param+index, 2);
	index += 2;
	ConfigParam->TransmissionRedundancy = param[index];
	index += 1;
	memcpy1((uint8_t *)&ConfigParam->ReceiveDelay1, param+index, 4);
	index += 4;
	memcpy1((uint8_t *)&ConfigParam->ReceiveDelay2, param+index, 4);
	index += 4;
	memcpy1((uint8_t *)&ConfigParam->JoinAcceptDelay1, param+index, 4);
	index += 4;
	memcpy1((uint8_t *)&ConfigParam->JoinAcceptDelay2, param+index, 4);
	index += 4;
	memcpy1((uint8_t *)&ConfigParam->AppTxDutycycle, param+index, 4);
	index += 4;
	memcpy1((uint8_t *)&ConfigParam->AppTxDutycycleRandom, param+index, 4);
	index += 4;
	ConfigParam->ChannelsDatarate = param[index];
	index += 1;
	ConfigParam->ChannelsTxPower = param[index];
	index += 1;
	ConfigParam->AdrEnable = param[index];
	index += 1;
	ConfigParam->ApplicationMode = param[index];
	index += 1;
	ConfigParam->EnableDebugPrintf = param[index];
	index += 1;
	ConfigParam->DeviceClass = param[index];
}

bool get_config_param(void)
{
	uint8_t buffer[CONFIG_PARAM_SIZE] = {0};
	
	if(*(__IO uint32_t*)DATA_EEPROM_FLAG_ADDR != SET_CONFIGURE_FLAG)
	{
		DBG("Get Config Flag:0x%08X=0x%08X\r\n", DATA_EEPROM_FLAG_ADDR, *(__IO uint32_t*)DATA_EEPROM_FLAG_ADDR);
		return false;
	}
	else
	{
		get_eeprom_data(buffer, CONFIG_PARAM_SIZE);
#ifdef USE_LORAWAN_PASSTHROUGH
 		lorawan_use_config_param(buffer);
		DBG("Get LoRaWAN Config Finish\r\n");
#else
		non_lorawan_use_config_param(buffer);
		DBG("Get non-LoRaWAN Config Finish\r\n");
#endif
		
		return true;
	}
}

uint8_t set_m100a_config(uint8_t *buffer, uint8_t size)
{  
	uint8_t result = 0;
	
	if(size == CONFIG_PARAM_SIZE)
	{
		if( !write_eeprom_data(buffer, size) )
		{
			result = 1;
			DBG("Save config Failed\r\n");
		}
		else
		{
			result = 0;
			DBG("Save config OK\r\n");
		}
	}
	else
	{
		result = 1;
		DBG("set all param size != 194\r\n");
	}
	
	return result;
}

void m100a_uart_standard_process(uint8_t *pdata, uint8_t size)
{ 
	print_message("Serial Data:", pdata, size, 'X');

#ifdef USE_LORAWAN_PASSTHROUGH
	LoRaMacTxInfo_t txInfo;
	
	if( LoRaMacQueryTxPossible( (size-3), &txInfo ) == LORAMAC_STATUS_LENGTH_ERROR )
	{
		DBG("Serail Data Length too long\r\n");
		rf_send_fail_ack();
		
		return;
	}
	else
	{
		uart_buffer_pkt_len = size-3;
		memcpy(uart_buffer, pdata+2, uart_buffer_pkt_len);
#else
	{
		set_rf_buffer(pdata+2, size-3);
#endif
		usart_received_data_flag = true;
		DBG("\r\nRecvied Standard Serial Data\r\n");
	}
}

void m100a_uart_config_process(uint8_t *pdata, uint8_t size)
{
	parse_serial_frame(pdata+1, size-2);
}

void m100a_uart_interrupt(UartNotifyId_t id)
{
	uint8_t data;
	static uint8_t start_flag = 0, frame_length = 0, frame_index = 0;
	
	if( id == UART_NOTIFY_RX )
	{
		if( UartGetChar( &Uart2, &data ) == 0 )
		{
			if( buffer_size >= 0xFF ) 
			{ 
				buffer_size = 0; 
				frame_length = 0;
				frame_index = 0;
				start_flag = 0;
				rf_send_fail_ack();
				DBG("Serial Data >= 255, set buffer_size = 0.\r\n");
			}

			if(( start_flag == 0) && ( data == 0xFE )) { start_flag = 1; }
			
			if( start_flag ) 
			{ 
				buffer[buffer_size++] = data;
				frame_index++;
			}
			if(get_work_mode() == WORK_MODE_STANDARD)
			{
				if(frame_index == 2) { frame_length = data + 3; }
			}
			else if(get_work_mode() == WORK_MODE_CONFIG)
			{
				if(frame_index == 3) { frame_length = data + 6; }
			}

			if((frame_length != 0) && (frame_length == frame_index))
			{
				if( data == 0xEF )
				{
					if(get_work_mode() == WORK_MODE_STANDARD)
					{
						m100a_uart_standard_process(buffer, buffer_size);
					}
					else if(get_work_mode() == WORK_MODE_CONFIG)
					{
						m100a_uart_config_process(buffer, buffer_size);
					}
					else
					{
							
					}   
				}
				buffer_size = 0;
				frame_length = 0;
				frame_index = 0;
				start_flag = 0;
			}
		}
	}
}

void set_com_to_default_config(void)
{
	FifoInit( &Uart2.FifoTx, tx_buffer, FIFO_TX_SIZE);
	FifoInit( &Uart2.FifoRx, rx_buffer, FIFO_RX_SIZE);

	Uart2.IrqNotify = m100a_uart_interrupt;

	UartInit( &Uart2, UART_2, USART1_TX, USART1_RX );
	UartConfig( &Uart2, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
	GpioInit( &usart_rts, STM8_RTS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void m100a_uart_init(void)
{
	uint32_t baudrate = 0;
	WordLength_t com_data_bits = UART_8_BIT;
	StopBits_t com_stop_bits = UART_1_STOP_BIT;
	Parity_t com_parity = NO_PARITY;
	
	FifoInit( &Uart2.FifoTx, tx_buffer, FIFO_TX_SIZE);
	FifoInit( &Uart2.FifoRx, rx_buffer, FIFO_RX_SIZE);

	Uart2.IrqNotify = m100a_uart_interrupt;

	UartInit( &Uart2, UART_2, USART1_TX, USART1_RX );
#ifdef USE_LORAWAN_PASSTHROUGH
	baudrate = 115200;
	com_data_bits = UART_8_BIT;
	com_stop_bits = UART_1_STOP_BIT;
	com_parity = NO_PARITY;
#else
	switch(get_com_baud())
	{
	case 0:  baudrate = 1200; DBG("UART Baud: 1200\r\n"); break;
	case 1:  baudrate = 2400; DBG("UART Baud: 2400\r\n"); break;
	case 2:  baudrate = 4800; DBG("UART Baud: 4800\r\n"); break;        
	case 3:  baudrate = 9600; DBG("UART Baud: 9600\r\n"); break;       
	case 4:  baudrate = 19200; DBG("UART Baud: 19200\r\n"); break;  
	case 5:  baudrate = 115200; DBG("UART Baud: 115200\r\n"); break;
	default: baudrate = 115200; DBG("UART Baud: 115200\r\n"); break;
	}
	com_data_bits = (WordLength_t)get_com_data_bits();
	com_stop_bits = (StopBits_t)get_com_stop_bits();
	com_parity = (Parity_t)get_com_parity();
	DBG("UART Data Bits: %d\r\n", com_data_bits);
	DBG("UART Stop Bits: %d\r\n", com_stop_bits);
	DBG("UART Parity: %d\r\n", com_parity);
#endif
	UartConfig( &Uart2, RX_TX, baudrate, com_data_bits, com_stop_bits, com_parity, NO_FLOW_CTRL );
	GpioInit( &usart_rts, STM8_RTS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void build_serial_frame(uint8_t msg_type, uint8_t *payload, uint8_t pyaload_length)
{
	uint16_t crc16 = 0;

	uart_buffer[0] = 0xFE;
	uart_buffer[1] = msg_type;
	uart_buffer[2] = pyaload_length;
	memcpy1(uart_buffer+3, payload, pyaload_length);
	crc16 = util_CRC16(uart_buffer+1, pyaload_length+2);
	uart_buffer[pyaload_length + 3] = (uint8_t)(crc16 & 0x00FF);
	uart_buffer[pyaload_length + 4] = (uint8_t)((crc16 >> 8) & 0x00FF);
	uart_buffer[pyaload_length + 5] = 0xEF;
	uart_buffer_pkt_len = pyaload_length + 6;
}

void respone_config(uint8_t msg_type)
{
	uint8_t buffer[CONFIG_PARAM_SIZE+1] = {0};

	buffer[0] = 0xAB;
	get_eeprom_data(buffer+1, CONFIG_PARAM_SIZE);

	build_serial_frame(msg_type, buffer, CONFIG_PARAM_SIZE + 1);
	
	if(UartPutBuffer( &Uart2, uart_buffer, uart_buffer_pkt_len ))
	{
		DBG("UART1 send Error\r\n");
	}
	else
	{
		DBG("UART1 send %dB OK\r\n", uart_buffer_pkt_len);
	}
}

enum ParseErrorCode parse_serial_frame(uint8_t *frame, uint8_t frame_length)
{
	uint8_t command = 0;

	serial_config_frame_t message_frame = {0};
	
	message_frame.crc16 = ((uint16_t)(frame[frame_length -1] << 8)) | frame[frame_length - 2];

	if(util_CRC16(frame, frame_length - 2) == message_frame.crc16)
	{
		message_frame.type = frame[0];
		message_frame.payload_length = frame[1];
		message_frame.pdata = frame + 2;
			
		switch(message_frame.type)
		{
		case SET_M100A: 
		case GET_M100A: 
			command = message_frame.pdata[0];
			switch(command)
			{
			case 0xAB:
				if(message_frame.type == SET_M100A)
				{
					DBG("Set All Param %dByte\r\n", message_frame.payload_length-1);
					if( set_m100a_config(message_frame.pdata+1, CONFIG_PARAM_SIZE) == 0 )
					{
					}
					respone_config(SET_M100A_ACK);
				}
				else
				{
					DBG("Get All Param %dByte\r\n", message_frame.payload_length);
					respone_config(GET_M100A_ACK);
				}
				break;
			default:
				DBG("No define Command:%d\r\n", command);
				DBG("Set failed\r\n");
				break;
			}
			break;
		default: 
			DBG("Error Serial Type:%d\r\n", message_frame.type); break;
		}
	}
	else
	{
		DBG("ERROR CRC:0x%04x->0x%04x\r\n", util_CRC16(frame, frame_length - 2), message_frame.crc16);
		return CRC_ERROR;
	}
	
	return FINISH;
}

void show_lorawan_config_param(void)
{
	DBG("======================================================================\r\n");					
	DBG("Activation:%d\r\n", ConfigParam->Activation);
	print_message("DevEui:", ConfigParam->DevEui, 8, 'X');
	print_message("AppEui:", ConfigParam->AppEui, 8, 'X');
	print_message("AppKey:", ConfigParam->AppKey, 16, 'X');
	print_message("NwkSKey:", ConfigParam->NwkSKey, 16, 'X');
	print_message("AppSKey:", ConfigParam->AppSKey, 16, 'X');
	DBG("DevAddress:0x%08X\r\n", ConfigParam->DevAddr);
	DBG("Channel 0:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_0.Frequency, 
	ConfigParam->Channel_0.DrRange.Fields.Max, 
	ConfigParam->Channel_0.DrRange.Fields.Min, 
	ConfigParam->Channel_0.Band);
	DBG("Channel 1:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_1.Frequency, 
	ConfigParam->Channel_1.DrRange.Fields.Max, 
	ConfigParam->Channel_1.DrRange.Fields.Min, 
	ConfigParam->Channel_1.Band);
	DBG("Channel 2:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_2.Frequency, 
	ConfigParam->Channel_2.DrRange.Fields.Max, 
	ConfigParam->Channel_2.DrRange.Fields.Min, 
	ConfigParam->Channel_2.Band);
	DBG("Channel 3:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_3.Frequency, 
	ConfigParam->Channel_3.DrRange.Fields.Max, 
	ConfigParam->Channel_3.DrRange.Fields.Min, 
	ConfigParam->Channel_3.Band);
	DBG("Channel 4:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_4.Frequency, 
	ConfigParam->Channel_4.DrRange.Fields.Max, 
	ConfigParam->Channel_4.DrRange.Fields.Min, 
	ConfigParam->Channel_4.Band);
	DBG("Channel 5:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_5.Frequency, 
	ConfigParam->Channel_5.DrRange.Fields.Max, 
	ConfigParam->Channel_5.DrRange.Fields.Min, 
	ConfigParam->Channel_5.Band);
	DBG("Channel 6:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_6.Frequency, 
	ConfigParam->Channel_6.DrRange.Fields.Max, 
	ConfigParam->Channel_6.DrRange.Fields.Min, 
	ConfigParam->Channel_6.Band);
	DBG("Channel 7:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_7.Frequency, 
	ConfigParam->Channel_7.DrRange.Fields.Max, 
	ConfigParam->Channel_7.DrRange.Fields.Min, 
	ConfigParam->Channel_7.Band);
	DBG("Channel 8:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_8.Frequency, 
	ConfigParam->Channel_8.DrRange.Fields.Max, 
	ConfigParam->Channel_8.DrRange.Fields.Min, 
	ConfigParam->Channel_8.Band);
	DBG("Channel 9:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_9.Frequency, 
	ConfigParam->Channel_9.DrRange.Fields.Max, 
	ConfigParam->Channel_9.DrRange.Fields.Min, 
	ConfigParam->Channel_9.Band);
	DBG("Channel 10:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_10.Frequency, 
	ConfigParam->Channel_10.DrRange.Fields.Max, 
	ConfigParam->Channel_10.DrRange.Fields.Min, 
	ConfigParam->Channel_10.Band);
	DBG("Channel 11:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_11.Frequency, 
	ConfigParam->Channel_11.DrRange.Fields.Max, 
	ConfigParam->Channel_11.DrRange.Fields.Min, 
	ConfigParam->Channel_11.Band);
	DBG("Channel 12:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_12.Frequency, 
	ConfigParam->Channel_12.DrRange.Fields.Max, 
	ConfigParam->Channel_12.DrRange.Fields.Min, 
	ConfigParam->Channel_12.Band);
	DBG("Channel 13:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_13.Frequency, 
	ConfigParam->Channel_13.DrRange.Fields.Max, 
	ConfigParam->Channel_13.DrRange.Fields.Min, 
	ConfigParam->Channel_13.Band);
	DBG("Channel 14:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_14.Frequency, 
	ConfigParam->Channel_14.DrRange.Fields.Max, 
	ConfigParam->Channel_14.DrRange.Fields.Min, 
	ConfigParam->Channel_14.Band);
	DBG("Channel 15:Freq:%dHz, DR:[%d-%d], Band:%d\r\n", 
	ConfigParam->Channel_15.Frequency, 
	ConfigParam->Channel_15.DrRange.Fields.Max, 
	ConfigParam->Channel_15.DrRange.Fields.Min, 
	ConfigParam->Channel_15.Band);
	DBG("Network Type:%d\r\n", ConfigParam->NetworkType);
	DBG("Rx2 Channel:Freq:%dHz, DR:%d\r\n",
	ConfigParam->Rx2Channel.Frequency,
	ConfigParam->Rx2Channel.Datarate);
	DBG("ChannelsMask:0x%04X\r\n", ConfigParam->ChannelsMask);
	DBG("Transmission Redundancy:%d\r\n", ConfigParam->TransmissionRedundancy);
	DBG("ReceiveDelay1:%dus\r\n", ConfigParam->ReceiveDelay1);
	DBG("ReceiveDelay2:%dus\r\n", ConfigParam->ReceiveDelay2);
	DBG("JoinAcceptDelay1:%dus\r\n", ConfigParam->JoinAcceptDelay1);
	DBG("JoinAcceptDelay2:%dus\r\n", ConfigParam->JoinAcceptDelay2);
	DBG("AppTxDutycycle:%dus\r\n", ConfigParam->AppTxDutycycle);
	DBG("AppTxDutycycleRandom:%dus\r\n", ConfigParam->AppTxDutycycleRandom);
	DBG("Channel Datarate Index:%d\r\n", ConfigParam->ChannelsDatarate);
	DBG("Channel TxPower Index:%d\r\n", ConfigParam->ChannelsTxPower);
	DBG("ADR Endble:%d\r\n", ConfigParam->AdrEnable);
	DBG("Application Mode:%d\r\n", ConfigParam->ApplicationMode);
	DBG("Device Class:%d\r\n", ConfigParam->DeviceClass);
	DBG("======================================================================\r\n");
}

void m100a_self_test(void)
{
	if(!get_config_param())
	{
		DBG("The M100A is not config, so stop work in config mode.\r\nPlease use com config: 115200 8E1 no parity config to set\r\n");
		set_com_to_default_config();
		set_work_mode(WORK_MODE_CONFIG);
		enable_usart_state(true);
		while(1);
	}
	else
	{
#ifdef USE_LORAWAN_PASSTHROUGH
		show_lorawan_config_param();
#else
		show_non_lorawan_config_param();
#endif
		DBG("Get EEPROM Config Param Success.\r\n");
	}
}

void passthrough_init(void)
{
	BoardInitMcu( );
	
	debug_printf_com_config();	// usart for printf

	enable_debug_printf();
	
	m100a_self_test();
	
	m100a_uart_init();
	
	printf("MCU Init ok\r\n");
	
	led_blink();
}

void config_initialization(void)
{
  MibRequestConfirm_t mibReq;
	LoRaMacStatus_t state;
	
	state = LoRaMacChannelAdd(0, ConfigParam->Channel_0);
	DBG("Add Channel_0 :%d\r\n", state);
	state = LoRaMacChannelAdd(1, ConfigParam->Channel_1);
	DBG("Add Channel_1 :%d\r\n", state);
	state = LoRaMacChannelAdd(2, ConfigParam->Channel_2);
	DBG("Add Channel_2 :%d\r\n", state);
	state = LoRaMacChannelAdd(3, ConfigParam->Channel_3);
	DBG("Add Channel_3 :%d\r\n", state);
	state = LoRaMacChannelAdd(4, ConfigParam->Channel_4);
	DBG("Add Channel_4 :%d\r\n", state);
	state = LoRaMacChannelAdd(5, ConfigParam->Channel_5);
	DBG("Add Channel_5 :%d\r\n", state);
	state = LoRaMacChannelAdd(6, ConfigParam->Channel_6);
	DBG("Add Channel_6 :%d\r\n", state);
	state = LoRaMacChannelAdd(7, ConfigParam->Channel_7);
	DBG("Add Channel_7 :%d\r\n", state);
	state = LoRaMacChannelAdd(8, ConfigParam->Channel_8);
	DBG("Add Channel_8 :%d\r\n", state);
	state = LoRaMacChannelAdd(9, ConfigParam->Channel_9);
	DBG("Add Channel_9 :%d\r\n", state);
	state = LoRaMacChannelAdd(10, ConfigParam->Channel_10);
	DBG("Add Channel_10 :%d\r\n", state);
	state = LoRaMacChannelAdd(11, ConfigParam->Channel_11);
	DBG("Add Channel_11 :%d\r\n", state);
	state = LoRaMacChannelAdd(12, ConfigParam->Channel_12);
	DBG("Add Channel_12 :%d\r\n", state);
	state = LoRaMacChannelAdd(13, ConfigParam->Channel_13);
	DBG("Add Channel_13 :%d\r\n", state);
	state = LoRaMacChannelAdd(14, ConfigParam->Channel_14);
	DBG("Add Channel_14 :%d\r\n", state);
	state = LoRaMacChannelAdd(15, ConfigParam->Channel_15);
	DBG("Add Channel_15 :%d\r\n", state);

	mibReq.Type = MIB_DEVICE_CLASS;
	mibReq.Param.Class = (DeviceClass_t)ConfigParam->DeviceClass;
	LoRaMacMibSetRequestConfirm( &mibReq );
	
	mibReq.Type = MIB_ADR;
	mibReq.Param.AdrEnable = ConfigParam->AdrEnable;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_PUBLIC_NETWORK;
	mibReq.Param.EnablePublicNetwork = ConfigParam->NetworkType;
	LoRaMacMibSetRequestConfirm( &mibReq );
	
	mibReq.Type = MIB_RX2_CHANNEL;
	mibReq.Param.Rx2Channel = ConfigParam->Rx2Channel;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_CHANNELS_MASK;
	mibReq.Param.ChannelsMask = &ConfigParam->ChannelsMask;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_CHANNELS_NB_REP;
	mibReq.Param.ChannelNbRep = ConfigParam->TransmissionRedundancy;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_RECEIVE_DELAY_1;
	mibReq.Param.ReceiveDelay1 = ConfigParam->ReceiveDelay1;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_RECEIVE_DELAY_2;
	mibReq.Param.ReceiveDelay2 = ConfigParam->ReceiveDelay2;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_JOIN_ACCEPT_DELAY_1;
	mibReq.Param.JoinAcceptDelay1 = ConfigParam->JoinAcceptDelay1;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
	mibReq.Param.JoinAcceptDelay2 = ConfigParam->JoinAcceptDelay2;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_CHANNELS_DATARATE;
	mibReq.Param.ChannelsDatarate = ConfigParam->ChannelsDatarate;
	LoRaMacMibSetRequestConfirm( &mibReq );

	mibReq.Type = MIB_CHANNELS_TX_POWER;
	mibReq.Param.ChannelsTxPower = ConfigParam->ChannelsTxPower;
	LoRaMacMibSetRequestConfirm( &mibReq );

	if(ConfigParam->Activation != 1)
	{
		mibReq.Type = MIB_NET_ID;
		mibReq.Param.NetID = LORAWAN_NETWORK_ID;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_DEV_ADDR;
		mibReq.Param.DevAddr = ConfigParam->DevAddr;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_NWK_SKEY;
		mibReq.Param.NwkSKey = ConfigParam->NwkSKey;
		LoRaMacMibSetRequestConfirm( &mibReq );

		mibReq.Type = MIB_APP_SKEY;
		mibReq.Param.AppSKey = ConfigParam->AppSKey;
		LoRaMacMibSetRequestConfirm( &mibReq );
		
		mibReq.Type = MIB_NETWORK_JOINED;
		mibReq.Param.IsNetworkJoined = true;
		LoRaMacMibSetRequestConfirm( &mibReq );
	}
	
	DBG("Use EEPROM Config Param Success.\r\n");
}

void ota_join_req(void)
{
	MlmeReq_t mlmeReq;
	
	mlmeReq.Type = MLME_JOIN;
	
	mlmeReq.Req.Join.DevEui = ConfigParam->DevEui;
	mlmeReq.Req.Join.AppEui = ConfigParam->AppEui;
	mlmeReq.Req.Join.AppKey = ConfigParam->AppKey;
	
	LoRaMacMlmeRequest( &mlmeReq );
}

bool check_activation(void)
{
	return ConfigParam->Activation;
}

uint32_t get_app_tx_dutycycle(void)
{
	return ConfigParam->AppTxDutycycle;
}

uint32_t get_app_tx_dutycycle_random(void)
{
	return ConfigParam->AppTxDutycycleRandom;
}

uint8_t get_application_mode(void)
{
	return ConfigParam->ApplicationMode;
}

uint32_t get_app_tx_dutycycle_time(void)
{
	return (ConfigParam->AppTxDutycycle + randr( -ConfigParam->AppTxDutycycleRandom, ConfigParam->AppTxDutycycleRandom ));
}

void set_app_port(uint8_t *port)
{
	*port = 222;
}

bool check_join_network(void)
{
	MibRequestConfirm_t mibReq;
	LoRaMacStatus_t status;
	
	mibReq.Type = MIB_NETWORK_JOINED;
	status = LoRaMacMibGetRequestConfirm( &mibReq );
	if( status == LORAMAC_STATUS_OK )
	{
		if( mibReq.Param.IsNetworkJoined == true )
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

void enable_debug_printf(void)
{
	ConfigParam->EnableDebugPrintf = true;
}

void clear_usart_received_data_flag(void)
{
	usart_received_data_flag = false;
}

bool check_usart_received_data_flag(void)
{
	return usart_received_data_flag;
}

uint8_t copy_usart_received_data(uint8_t *buffer)
{
	memcpy(buffer, uart_buffer, uart_buffer_pkt_len);
	return uart_buffer_pkt_len;
}

void set_work_mode(uint8_t mode)
{
	ConfigParam->WorkMode = mode;
}

uint8_t get_work_mode(void)
{
	return ConfigParam->WorkMode;
}

void enable_usart_state(bool state)
{
	if(state)
	{
		USART_Cmd(USART2, ENABLE);
		GpioWrite(&usart_rts, 0);
	}
	else
	{
		USART_Cmd(USART2, DISABLE);
		GpioWrite(&usart_rts, 1);
	}
}

void send_rf_data(uint8_t *data, uint8_t size)
{
    uint8_t buffer[255] = {0};
    
    buffer[0] = 0xFE;
    buffer[1] = size;
    memcpy1(buffer+2, data, size);
    buffer[2+size] = 0xEF;
    
    UartPutBuffer(&Uart2, buffer, size+3);
}

void rf_send_ok_ack(void)
{
	uint8_t buffer[4] = {0xFE, 0x01, SEND_OK_FLAG, 0xEF};

	UartPutBuffer( &Uart2, buffer, 4 );
}

void rf_send_fail_ack(void)
{
	uint8_t buffer[4] = {0xFE, 0x01, SEND_FAIL_FLAG, 0xEF};

	UartPutBuffer( &Uart2, buffer, 4 );
}

void check_run_work_mode(void)
{
	Gpio_t set_config_mode_chek_io;
	GpioInit(&set_config_mode_chek_io, MODE_SET, PIN_INPUT, PIN_OPEN_DRAIN, PIN_PULL_DOWN, 0);
	if(GpioRead(&set_config_mode_chek_io) != 0)
	{
		DBG("In Config Mode\r\n");
		set_work_mode(WORK_MODE_CONFIG);
		enable_usart_state(true);
		while(1);
	}
	else
	{
		DBG("Run Standard Mode\r\n");
		set_work_mode(WORK_MODE_STANDARD);
		enable_usart_state(false);
	}
}

void show_lora_mac_status(uint8_t *tite, LoRaMacEventInfoStatus_t Status)
{
	  DBG("%s->Status: ", tite);
    switch(Status)
    {
			case 0: DBG("LORAMAC_EVENT_INFO_STATUS_OK\r\n"); break;
			case 1: DBG("LORAMAC_EVENT_INFO_STATUS_ERROR\r\n"); break;
			case 2: DBG("LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT\r\n"); break;
			case 3: DBG("LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT\r\n"); break;
			case 4: DBG("LORAMAC_EVENT_INFO_STATUS_RX2_ERROR\r\n"); break;
			case 5: DBG("LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL\r\n"); break;
			case 6: DBG("LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED\r\n"); break;
			case 7: DBG("LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL\r\n"); break;
			case 8: DBG("LORAMAC_EVENT_INFO_STATUS_MIC_FAIL\r\n"); break;
			default: DBG("%d\r\n", Status); break;
    }
}

uint8_t get_channel_data_rate(void)
{
	return ConfigParam->ChannelsDatarate;;
}
