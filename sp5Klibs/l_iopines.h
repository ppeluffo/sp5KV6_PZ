/*
 * l_iopines.h
 *
 *  Created on: 18 de jun. de 2017
 *      Author: pablo
 */

// --------------------------------------------------------------------------------
// SPV5 LIB
// --------------------------------------------------------------------------------

#ifndef SRC_SP5KLIBS_L_IOPINES_H_
#define SRC_SP5KLIBS_L_IOPINES_H_

#include <l_mcp.h>
#include <stdbool.h>
#include "sp5Klibs/avrlibdefs.h"

typedef enum { LOW = 0, HIGH } t_low_high;

//#define io_DEBUG

#ifdef io_DEBUG
	#define DEBUG_IO 4
#else
	#define DEBUG_IO 0
#endif

// Led de placa analogica ( PD6 )
#define TEST_PORT		PORTD
#define TEST_PIN		PIND
#define TEST_BIT		6
#define TEST_DDR		DDRD
#define TEST_MASK		0x40

// TERMINAL
// Pin de control de fuente de la terminal ( PD7)
#define TERMSW_PORT		PORTD
#define TERMSW_PIN		PIND
#define TERMSW_BIT		7
#define TERMSW_DDR		DDRD
#define TERMSW_MASK		0x80

// DCD
// Como el MCP23018 a veces no detecta el nivel del modem, cableamos
// el DCD a PB3
// Pin de control de fuente de la terminal ( PB3)
#define DCD_PORT		PORTB
#define DCD_PIN			PINB
#define DCD_BIT			3
#define DCD_DDR			DDRB
#define DCD_MASK		0x8

// --------------------------------------------------------------------------------

#define IO_set_led_KA_logicBoard() 		(MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OLED ))
#define IO_clear_led_KA_logicBoard() 	(MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OLED ))

#define IO_config_TEST_PIN()	(sbi(TEST_DDR, TEST_BIT))
#define IO_set_TEST_PIN() 		(sbi(TEST_PORT, TEST_BIT))
#define IO_clear_TEST_PIN() 	(cbi(TEST_PORT, TEST_BIT))

//#define IO_set_led_MODEM_analogBoard() 	(cbi(LED_MODEM_PORT, LED_MODEM_BIT))
//#define IO_clear_led_MODEM_analogBoard() (sbi(LED_MODEM_PORT, LED_MODEM_BIT))

#define IO_term_pwr_on() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OTERMPWR ) )
#define IO_term_pwr_off() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OTERMPWR ) )

#define IO_sensor_pwr_on() 	( MCP_modify( MCP1_ADDR, MCP1_OLATB, 1, MCP1_PWRSENSORS ) )
#define IO_sensor_pwr_off() ( MCP_modify( MCP1_ADDR, MCP1_OLATB, 0, MCP1_PWRSENSORS ) )

#define IO_analog_pwr_on() 	( MCP_modify( MCP1_ADDR, MCP1_OLATB, 1, MCP1_OANALOG ) )
#define IO_analog_pwr_off() ( MCP_modify( MCP1_ADDR, MCP1_OLATB, 0, MCP1_OANALOG ) )

#define IO_modem_hw_pwr_on() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OGPRSPWR ) )
#define IO_modem_hw_pwr_off() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OGPRSPWR ) )

// El modem_sw_switch es un transistor por lo tanto invierte la entrada en la salida. !!!
#define IO_modem_sw_switch_high() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 0, MCP0_GPIO_OGPRSSW ) )
#define IO_modem_sw_switch_low() 	( MCP_modify( MCP0_ADDR, MCP0_GPIO, 1, MCP0_GPIO_OGPRSSW ) )

bool IO_read_dcd( uint8_t *pin);

// RANGE
#define UPULSE_RUN_PORT		PORTB
#define UPULSE_RUN_BIT		1
#define UPULSE_RUN_DDR		DDRB
#define UPULSE_RUN_MASK		0x02

#define UPULSE_WIDTH_PORT		PORTB
#define UPULSE_WIDTH_PIN		PINB
#define UPULSE_WIDTH_BIT		2
#define UPULSE_WIDTH_DDR		DDRB
#define UPULSE_WIDTH_MASK		0x04

#define IO_config_UPULSE_RUN()	sbi(UPULSE_RUN_DDR, UPULSE_RUN_BIT);
#define IO_set_UPULSE_RUN()		sbi(UPULSE_RUN_PORT, UPULSE_RUN_BIT)
#define IO_clr_UPULSE_RUN()		cbi(UPULSE_RUN_PORT, UPULSE_RUN_BIT)

#define IO_config_UPULSE_WIDTH()	cbi(UPULSE_WIDTH_DDR, UPULSE_WIDTH_BIT);

uint8_t IO_read_UPULSE_WIDTH(void);

//------------------------------------------------------------------------------------

#endif /* SRC_SP5KLIBS_L_IOPINES_H_ */
