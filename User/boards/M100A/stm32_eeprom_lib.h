#ifndef __STM32_EEPROM_LIB__
#define __STM32_EEPROM_LIB__
#include "stm32l1xx.h"

#ifdef USE_LORAWAN_PASSTHROUGH
#define DATA_EEPROM_FLAG_ADDR      									0x08080000
#define SET_CONFIGURE_FLAG         									(uint32_t)0xAB
#define DATA_EEPROM_START_ADDR     									0x08080004
#else
#define DATA_EEPROM_FLAG_ADDR      									0x08080200
#define SET_CONFIGURE_FLAG         									(uint32_t)0xBB
#define DATA_EEPROM_START_ADDR     									0x08080204
#endif

#define DATA_EEPROM_END_ADDR       									0x080803FF


uint8_t write_eeprom_data(uint8_t *data, uint8_t size);
void get_eeprom_data(uint8_t *buffer, uint8_t size);

#endif
