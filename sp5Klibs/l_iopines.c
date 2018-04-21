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
void IO_init_pines(void)
{
	// Configuracion de pines:

	// El pin de control de la terminal es entrada
	cbi(TERMSW_DDR, TERMSW_BIT);

	// El pin de DCD es entrada
	//cbi(DCD_DDR, DCD_BIT);

	// Leds
	//sbi(LED_KA_DDR, LED_KA_BIT);		// El pin del led de KA ( PD6 ) es una salida.
	//sbi(LED_MODEM_DDR, LED_MODEM_BIT);
	// inicialmente los led quedan en 0
	//sbi(LED_KA_PORT, LED_KA_BIT);
	//sbi(LED_MODEM_PORT, LED_MODEM_BIT);

	IO_config_TEST_PIN();
	IO_clear_TEST_PIN(); // Inicialmente en 0

	// RANGE
	IO_config_UPULSE_RUN();
	IO_config_UPULSE_WIDTH();

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
