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
bool IO_read_pulseInputs( uint8_t *din0, uint8_t *din1 )
{

	// Las entradas de los pulsos ( latches ) corresponden a las entradas GPB5 y GPB6 del
	// MCP23018.

bool retS;
uint8_t regValue;

	retS = MCP_read( MCP1_ADDR, MCP1_GPIOB, &regValue);
	*din0 = ( regValue & 0x40) >> 6;
	*din1 = ( regValue & 0x20) >> 5;
	return(retS);
}
//------------------------------------------------------------------------------------
uint8_t IO_read_terminal_pin(void)
{
	// El TERMSW se cablea a PD7.
	return ( ( TERMSW_PIN & _BV(7) ) >> 7 );
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
