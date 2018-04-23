/*
 * sp5KV3_tkControl.c
 *
 *  Created on: 7/4/2015
 *      Author: pablo
 *
 *  Tareas de control generales del SP5K
 *  - Recibe un mensaje del timer del led para indicar si debe prender o apagarlo.
 */

#include <sp5KV6_PZ.h>
//#include "sp5KV6_PZ_tkGPRS/sp5KV6_PZ_tkGprs.h"

static char ctl_printfBuff[CHAR128];

static void pv_tkControl_init(void);
static void pv_init_show_reset_cause(void);

static void pv_check_leds(void);
static void pv_check_daily_reset(void);
static void pv_check_wdg(void);

//------------------------------------------------------------------------------------
void tkControl(void * pvParameters)
{

( void ) pvParameters;

	// Aqui solo controlo la terminal por lo que no me importa tanto el watchdog.
	// Lo controlo en LedTiltWdg

	pv_tkControl_init();
	pv_init_show_reset_cause();

	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"-----------------\r\n\0");
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"starting tkControl..\r\n\0");
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Loop
    for( ;; )
    {

    	u_kick_Wdg(WDG_CTL);

	   	// Espero 1 segundo para revisar todo.
        vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

        // Reviso los sistemas perifericos.
        pv_check_leds();
        pv_check_daily_reset();
        pv_check_wdg();

    }

}
//------------------------------------------------------------------------------------
static void pv_check_wdg(void)
{
	// Cada tarea periodicamente pone su wdg flag en 0. Esto hace que al chequearse c/3s
	// deban estar todas en 0 para asi resetear el wdg del micro.

static u08 l_timer = 20;

	// Espero 10s sin resetearme
	if (l_timer-- > 0 )
		wdt_reset();
		return;

	// Si algun WDG no se borro, me reeseteo
	if ( systemWdg != 0 ) {
		while(1);
	}

	// Reincializo el sistema
	systemWdg = WDG_CTL + WDG_CMD;
	l_timer = 20;
}
//------------------------------------------------------------------------------------
static void pv_check_leds(void)
{

static uint8_t count = 3;

	// Los leds flashean c/3s solo si la terminal esta prendida.
	// Siempre los apago para no correr riesgo que queden prendidos.

	if ( --count > 0 ) {
		return;
	}

	count = 3;

   	// Prendo.
   	IO_set_led_KA_logicBoard();				// Led de KA de la placa logica
  	// no es necesario ya que lo que demora las MCP son suficientes.
   	//vTaskDelay( 1 );

   	// Apago
   	IO_clear_led_KA_logicBoard();

 }
//------------------------------------------------------------------------------------
static void  pv_check_daily_reset(void)
{
	// Todos los dias debo resetearme para restaturar automaticamente posibles
	// problemas.

static uint32_t ticks_to_reset = 86400; // Segundos en 1 dia.

	while ( --ticks_to_reset > 0 ) {
		return;
	}

	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"%s CTL::reset: Daily Reset !!\r\n\0", u_now() );
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	wdt_enable(WDTO_30MS);
	while(1) {}

}
//------------------------------------------------------------------------------------
// FUNCIONES DE INIT
//------------------------------------------------------------------------------------
static void pv_tkControl_init(void)
{

uint8_t ffRcd;
StatBuffer_t pxFFStatBuffer;
uint16_t pos;
int8_t loadParamStatus = false;
uint16_t recSize;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	MCP_init(0);				// Esto prende la terminal.
	IO_term_pwr_on();
	u_uarts_ctl(0);

	// Load systemVars
	if  ( u_loadSystemParams() == true ) {
		loadParamStatus = true;
	} else {
		u_loadDefaults();
		u_saveSystemParams();
		loadParamStatus = false;
	}

	// Configuro el ID en el bluetooth: debe hacerse antes que nada
	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"AT+NAME%s\r\n",systemVars.dlgId);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

	// Mensaje de load Status.
	if ( loadParamStatus ) {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"Load config OK.\r\n\0");
	} else {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"Load config ERROR: defaults !!\r\n\0" );
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Inicializo la memoria EE ( fileSysyem)
	ffRcd = FF_fopen();
	FF_stat(&pxFFStatBuffer);
	if ( pxFFStatBuffer.errno != pdFF_ERRNO_NONE ) {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"FSInit ERROR (%d)[%d]\r\n\0",ffRcd, pxFFStatBuffer.errno);
	} else {
		FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"FSInit OK\r\nMEMsize=%d, wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d\r\n\0",FF_MAX_RCDS, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Tamanio de registro de memoria
	recSize = sizeof(frameData_t);
	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"RCD size %d bytes.\r\n\0",recSize );
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"Modules:: BASIC+RANGE\r\n\0");
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Habilito al resto de las tareas a arrancar.
	startTask = true;

}
//------------------------------------------------------------------------------------
static void pv_init_show_reset_cause(void)
{
uint8_t pos;

	// Muestro la razon del ultimo reseteo

	pos = FRTOS_snprintf( ctl_printfBuff,sizeof(ctl_printfBuff),"Init code (0x%X",wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," PORF");
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," EXTRF");
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," BORF");
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," WDRF");
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," JTRF");
	}
	pos += FRTOS_snprintf( &ctl_printfBuff[pos],sizeof(ctl_printfBuff)," )\r\n\0");
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

}
//------------------------------------------------------------------------------------
