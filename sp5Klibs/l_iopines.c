/*
 * l_iopines.c
 *
 *  Created on: 18 de jun. de 2017
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#include <l_iopines.h>
#include <sp5KV6_PZ.h>

//------------------------------------------------------------------------------------
uint8_t IO_read_D0(void)
{
	// El D0 se cablea a PD7.
	return ( ( D0_PIN & _BV(D0_BIT) ) >> D0_BIT );
}
//------------------------------------------------------------------------------------
uint8_t IO_read_D1(void)
{
	// El D0 se cablea a PD7.
	return ( ( D1_PIN & _BV(D1_BIT) ) >> D1_BIT );
}
//------------------------------------------------------------------------------------
uint8_t IO_read_UPULSE_WIDTH(void)
{

	return( ( UPULSE_WIDTH_PIN & _BV(2) ) >> 2 );
}
//------------------------------------------------------------------------------------
bool IO_read_dcd( uint8_t *pin)
{
	// MCP23008 logic
bool retS;
uint8_t regValue;

	// DCD es el bit1, mask = 0x02
	retS = MCP_read( MCP0_ADDR, MCP0_GPIO, &regValue);
//	*pin = ( regValue & 0x02) >> 1;
	*pin = ( regValue & _BV(1) ) >> 1;		// bit1, mask = 0x02
	return(retS);
}
//------------------------------------------------------------------------------------
