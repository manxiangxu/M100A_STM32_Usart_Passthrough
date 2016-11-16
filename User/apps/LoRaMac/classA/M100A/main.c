/*
2016 Winext

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Hellogz
*/
#include "lorawan_passthrough.h"
#include "non-lorawan_passthrough.h"

/*
Note: the USE_LORAWAN_PASSTHROUGH if define, use LoRaWAN Usart Passthrough.
			the USE_LORAWAN_PASSTHROUGH if not define, use non-LoRaWAN Usart Passthrough.
*/

/**
 * Main application entry point.
 */
int main( void )
{
#ifdef USE_LORAWAN_PASSTHROUGH
  lorawan_usart_passthrough_process();
#else
	non_lorawan_usart_passthrough_process();
#endif
}
