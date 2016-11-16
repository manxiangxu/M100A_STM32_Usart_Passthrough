/*
2016 Winext

Description: M100A Usart Passthrough implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Hellogz
*/
#ifndef __M100A_BOARD_H__
#define __M100A_BOARD_H__

#include "stm32l1xx.h"
#include "board.h"
#include <stdio.h>
#include "crc16.h"
#include "LoRaMac.h"

#define RF_BANDWIDTH_125K                           0   // 125k
#define LOFI_SYNCWORD				    										0x88

#define LORA_MAC_PUBLIC_SYNCWORD                    0x34
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12
 
#define NON_LORAWAN_MAX_RX_WINDOW                   0//1000000 // default 3s


#define LOFIMAC_MAXPAYLOAD                          255

/*********************************************************************************/
/*!
 * FIFO buffers size
 */
#define FIFO_TX_SIZE                    						256
#define FIFO_RX_SIZE                    						256

#define DBG																					printf

#ifdef USE_LORAWAN_PASSTHROUGH
#define CONFIG_PARAM_SIZE														204
#else
#define CONFIG_PARAM_SIZE               						27
#endif
#define LORAWAN_NETWORK_ID                          ( uint32_t )0

#define WORK_MODE_STANDARD													1
#define WORK_MODE_CONFIG														2

#define SEND_OK_FLAG																0xFF
#define SEND_FAIL_FLAG														  0x00

#define SERIAL_PACKAGE_MAX_SIZE                     255

enum SerialMessageType {
    SET_M100A = 0xF1,
    SET_M100A_ACK = 0xF2,
    GET_M100A = 0xF3,
    GET_M100A_ACK = 0xF4
};

enum ParseErrorCode {
    FINISH = 0,
    CRC_ERROR,
    MSG_TYPE_ERROR,
    PYLOAD_ERROR,
};

typedef enum e_lofi_mac_status
{
    LOFIMAC_STATUS_OK,
    LOFIMAC_STATUS_BUSY,
    LOFIMAC_STATUS_NO_NETWORK_JOINED
} lofi_mac_status_t;

typedef struct serial_frame_t_
{
    uint8_t type;
    uint8_t payload_length;
    uint8_t* pdata;
    uint16_t crc16;
} serial_config_frame_t;

/*!
 * LoRaMAC Config parameters
 */
typedef struct ConfigParam_t_
{
	uint8_t DevEui[8];

	uint8_t Activation;     // Personalization:0, Over the Air:1

	uint8_t AppEui[8];

	uint8_t AppKey[16];

	uint8_t NwkSKey[16];

	uint8_t AppSKey[16];

	uint32_t DevAddr;

	ChannelParams_t Channel_0;

	ChannelParams_t Channel_1;

	ChannelParams_t Channel_2;

	ChannelParams_t Channel_3;

	ChannelParams_t Channel_4;

	ChannelParams_t Channel_5;

	ChannelParams_t Channel_6;

	ChannelParams_t Channel_7;

	ChannelParams_t Channel_8;

	ChannelParams_t Channel_9;

	ChannelParams_t Channel_10;

	ChannelParams_t Channel_11;

	ChannelParams_t Channel_12;

	ChannelParams_t Channel_13;

	ChannelParams_t Channel_14;

	ChannelParams_t Channel_15;

	uint8_t NetworkType;    // Public:1, Private:0

	Rx2ChannelParams_t Rx2Channel;

	uint16_t ChannelsMask;

	uint8_t TransmissionRedundancy;

	uint32_t ReceiveDelay1;

	uint32_t ReceiveDelay2;

	uint32_t JoinAcceptDelay1;

	uint32_t JoinAcceptDelay2;

	uint32_t AppTxDutycycle;

	uint32_t AppTxDutycycleRandom;

	int8_t ChannelsDatarate;    // Datarate index

	int8_t ChannelsTxPower;     // Tx Power index

	bool AdrEnable;         // ON:1, OFF:0

	uint8_t ApplicationMode;

	bool EnableDebugPrintf;
	
	uint8_t WorkMode;
	
	uint8_t DeviceClass;
}ConfigParam_t;

struct prepare_frame_t_
{
    uint32_t rx_window1_delay;
    uint32_t rx_window2_delay;
    uint8_t lofi_mac_buffer[LOFIMAC_MAXPAYLOAD];
    uint16_t lofi_mac_buffer_pkt_len;
    uint8_t uart_buffer[LOFIMAC_MAXPAYLOAD];
    uint16_t uart_buffer_pkt_len;
    bool life_state_tx;
    bool sensor_state_tx;
    bool sensor_state_tx_ack;
    bool node_ack_requested;
    bool ack_timeout_retry;
};

struct m100a_config_t_
{
// RF Config
    uint32_t rf_frequency;
    uint8_t rf_syncword;
    uint8_t rf_preamble_len;
    uint8_t rf_tx_power;
    uint8_t rf_datarate;        // SF 
    uint8_t rf_bandwidth;
    uint8_t rf_header_mode;     //1: Implicit, 0: Explicit(??)      
    uint8_t rf_coderate;
    bool rf_crc_enable;
		bool rf_rx_iq_signal;
    bool rf_tx_iq_signal;

// Usart Config
    uint8_t com_baud;
    uint8_t com_parity;
    uint8_t com_data_bits;
    uint8_t com_stop_bits;

    uint32_t tx_time_on_air;
    uint8_t rx_slot;
    uint8_t work_mode;
    uint16_t breathe;
    uint8_t deveui[8];
    uint8_t security_code[6];
    uint32_t tx_duty_cycle_time;
    uint32_t tx_duty_cycle_random_time;
    uint8_t EnableDebugPrintf;
};

// typedef union uMsgHead
// {
//     uint8_t Value;

//     struct sFields
//     {
//         uint8_t PayloadLength : 6;
//         uint8_t Type : 2;
//     }Fields;
// }MsgHead_t;

typedef struct sLoFiFrame
{
// 	MsgHead_t MsgHead;
// 	uint8_t DevEui[8];
// 	uint8_t Payload[64];
//   uint8_t DecodePayload[64];
	uint8_t PayloadLength;
// 	uint32_t Mic;
// 	uint8_t NwsKey[16];
// 	uint8_t MicKey[16];
	uint8_t LoFiMacBuffer[76];
}LoFiFrame_t;

typedef struct prepare_frame_t_ *prepare_frame_t;
typedef struct m100a_config_t_ *m100a_config_t;

void led_blink(void);

void passthrough_init(void);

void debug_printf_com_config(void);

void print_message(const char *title, uint8_t *message, uint8_t length, uint8_t format);

void m100a_uart_init(void);

void ota_join_req(void);

void config_initialization(void);

void m100a_self_test(void);

bool check_activation(void);

uint32_t get_app_tx_dutycycle(void);

uint32_t get_app_tx_dutycycle_random(void);

uint32_t get_app_tx_dutycycle_time(void);

uint8_t get_application_mode(void);

void set_app_port(uint8_t *port);

bool check_join_network(void);

void enable_debug_printf(void);

void clear_usart_received_data_flag(void);

bool check_usart_received_data_flag(void);

uint8_t copy_usart_received_data(uint8_t *buffer);

void set_work_mode(uint8_t mode);

uint8_t get_work_mode(void);

void enable_usart_state(bool state);

void send_rf_data(uint8_t *data, uint8_t size);

void rf_send_ok_ack(void);

void rf_send_fail_ack(void);

void check_run_work_mode(void);

void respone_config(uint8_t msg_type);

void show_lora_mac_status(uint8_t *tite, LoRaMacEventInfoStatus_t Status);

uint8_t get_channel_data_rate(void);

#endif  // __M100A_BOARD_H__
