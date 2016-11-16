#ifndef __NON_LORAWAN_PASSTHROUGH_H__
#define __NON_LORAWAN_PASSTHROUGH_H__

#include "m100a-board.h"
#include "board.h"

uint8_t get_com_baud(void);
uint8_t get_com_parity(void);
uint8_t get_com_data_bits(void);
uint8_t get_com_stop_bits(void);
void show_non_lorawan_config_param(void);
void set_rf_buffer(uint8_t *buffer, uint8_t size);
void non_lorawan_use_config_param(uint8_t *param);
void non_lorawan_usart_passthrough_process(void);

#endif
