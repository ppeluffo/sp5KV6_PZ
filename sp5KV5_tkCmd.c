/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include <sp5KV5.h>
#include "sp5KV5_tkGPRS/sp5KV5_tkGprs.h"

static char cmd_printfBuff[CHAR128];
char *argv[16];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static uint8_t pv_makeArgv(void);

bool pv_cmdWrDebugLevel(char *s);
bool pv_cmdWrkMode(char *s0, char *s1);
static void pv_readMemory(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdClearScreen(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdStatusFunction(void);
static void cmdReadFunction(void);
static void cmdWriteFunction(void);
static void cmdRedialFunction(void);
/*------------------------------------------------------------------------------------*/
void tkCmd(void * pvParameters)
{

uint8_t c;
uint8_t ticks;
( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	cmdlineInit();
	cmdlineSetOutputFunc(pvFreeRTOS_UART1_writeChar);

	cmdlineAddCommand((uint8_t *)("cls"), cmdClearScreen );
	cmdlineAddCommand((uint8_t *)("help"), cmdHelpFunction);
	cmdlineAddCommand((uint8_t *)("reset"), cmdResetFunction);
	cmdlineAddCommand((uint8_t *)("read"), cmdReadFunction);
	cmdlineAddCommand((uint8_t *)("write"), cmdWriteFunction);
	cmdlineAddCommand((uint8_t *)("status"), cmdStatusFunction);
	cmdlineAddCommand((uint8_t *)("redial"), cmdRedialFunction);

	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	ticks = 1;
	FreeRTOS_ioctl( &pdUART1,ioctlSET_TIMEOUT, &ticks, false );

	// loop
	for( ;; )
	{
		u_kick_Wdg(WDG_CMD);

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		while ( FreeRTOS_read( &pdUART1, &c, 1 ) == 1 ) {
			cmdlineInputFunc(c);
		}

		/* run the cmdline execution functions */
		cmdlineMainLoop();

	}

}
/*------------------------------------------------------------------------------------*/
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdHelpFunction(void)
{

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset {memory}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-redial\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc YYMMDDhhmm\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR( "  wrkmode [service | monitor {sqe|frame}]\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerpoll, dlgid, gsmband\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );


	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debuglevel +/-{none,basic,mem,data,gprs,all} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  loglevel (none, info, all)\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn, roaming {on|off}, port, ip, script, passwd\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  save\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) mcp {devId}{regAddr}{regValue}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) ee addr string\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) led {0|1},gprspwr {0|1},gprssw {0|1}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  mcp {0|1} regAddr\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc, adc {ch}, frame\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee {addr}{lenght}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  defaults \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  (SM) memory,gprs\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
/*------------------------------------------------------------------------------------*/
static void cmdResetFunction(void)
{
	pv_makeArgv();

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
		FF_rewind();
	}

	cmdClearScreen();
	// RESET
	u_reset();

}
/*------------------------------------------------------------------------------------*/
static void cmdRedialFunction(void)
{
	// Envio un mensaje a la tk_Gprs para que recargue la configuracion y disque al server
	// Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
	while ( xTaskNotify(xHandle_tkGprsRx,TK_REDIAL , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

}
/*------------------------------------------------------------------------------------*/
static void cmdStatusFunction(void)
{

RtcTimeType_t rtcDateTime;
uint16_t pos;
uint8_t channel;
frameData_t Cframe;
StatBuffer_t pxFFStatBuffer;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Last reset info
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Wdg (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" JTRF"));
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DlgId */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Fecha y Hora */
	RTC_read(&rtcDateTime);
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("rtc: %02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Server:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* APN */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn: %s\r\n\0"), systemVars.apn );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER IP:SERVER PORT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.server_ip_address,systemVars.server_tcp_port );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER SCRIPT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER PASSWD */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  passwd: %s\r\n\0"), systemVars.passwd );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MODEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Modem:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Modem band */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  band: "));
	switch ( systemVars.gsmBand) {
	case 0:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(900)"));
		break;
	case 1:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(1800)"));
		break;
	case 2:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (900/1800)"));
		break;
	case 3:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("pcs (1900)"));
		break;
	case 4:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("gsm (850)"));
		break;
	case 5:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (1900/850)"));
		break;
	case 6:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (900/1800/1900)"));
		break;
	case 7:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (850/1800/1900)"));
		break;
	case 8:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("cuatriband (850/900/1800/1900)"));
		break;
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* ROAMING */
	if ( systemVars.roaming == true ) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  roaming ON\r\n\0"));
	} else {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  roaming OFF\r\n\0"));
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DLGIP */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  dlg ip: %s\r\n\0"), systemVars.dlg_ip_address );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CSQ */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// GPRS STATE
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  state: "));
	switch (GPRS_stateVars.state) {
	case G_ESPERA_APAGADO:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("await_off\r\n"));
		break;
	case G_PRENDER:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("prendiendo\r\n"));
		break;
	case G_CONFIGURAR:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("configurando\r\n"));
		break;
	case G_MON_SQE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("mon_sqe\r\n"));
		break;
	case G_GET_IP:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ip\r\n"));
		break;
	case G_INIT_FRAME:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("init frame\r\n"));
		break;
	case G_DATA:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("data\r\n"));
		break;
	default:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SYSTEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">System:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Memoria */
	FF_stat(&pxFFStatBuffer);
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memory: wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d \r\n"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* WRK mode (NORMAL / SERVICE) */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  wrkmode: "));
	switch (systemVars.wrkMode) {
	case WK_NORMAL:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("normal\r\n"));
		break;
	case WK_MONITOR_SQE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("monitor_sqe\r\n"));
		break;
	default:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Timers */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerPoll: [%ds]\r\n\0"),systemVars.timerPoll );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DebugLevel */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debugLevel: "));
	if ( systemVars.debugLevel == D_NONE) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("none") );
	} else {
		if ( (systemVars.debugLevel & D_BASIC) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+basic")); }
		if ( (systemVars.debugLevel & D_DATA) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+data")); }
		if ( (systemVars.debugLevel & D_GPRS) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+gprs")); }
		if ( (systemVars.debugLevel & D_MEM) != 0)   { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+mem")); }
		if ( (systemVars.debugLevel & D_DEBUG) != 0)  { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+debug")); }
	}
	snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CONFIG */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Config:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Bateria
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  batt{0-15V}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* VALUES --------------------------------------------------------------------------------------- */

}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{
uint8_t argc;
char datetime[24];
bool retS = false;
uint16_t adcRetValue = 9999;
uint8_t regValue;
uint8_t pin;

	argc = pv_makeArgv();

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		retS = EE_test_read( argv[2], cmd_printfBuff, argv[3] );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
			FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		if ( RTC_date_to_str( datetime, sizeof(datetime) ) != -1 ) {
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n%s\r\n\0"), datetime );
		} else {
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
		}
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// MCP
	// read mcp 0|1|2 addr
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		switch( atoi(argv[2] )) {
		case 0:
			retS = MCP_read( MCP0_ADDR, atoi(argv[3]), &regValue );
			break;
		case 1:
			retS = MCP_read( MCP1_ADDR, atoi(argv[3]), &regValue );
			break;
		}

		if (retS ) {
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n[reg 0X%03x]=[0X%03x]\r\n\0"),atoi(argv[3]),regValue);
		} else {
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
		}
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

 	// DEFAULT (load default configuration)
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULTS\0"))) {
		u_loadDefaults();
		return;
	}

	// MEMORY
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
		pv_readMemory();
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{
bool retS = false;
uint8_t argc;

	argc = pv_makeArgv();

	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		retS = u_saveSystemParams();
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PASSWD
	if (!strcmp_P( strupr(argv[1]), PSTR("PASSWD\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.passwd, '\0', sizeof(systemVars.passwd));
			memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
			systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
			systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// APN
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.apn, '\0', sizeof(systemVars.apn));
			memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
			systemVars.apn[APN_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// ROAMING
	if (!strcmp_P( strupr(argv[1]), PSTR("ROAMING\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON"))) { systemVars.roaming = true; }
		if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) { systemVars.roaming = false; }
		pv_snprintfP_OK();
		return;
	}

	// SERVER PORT
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_tcp_port, '\0', sizeof(systemVars.server_tcp_port));
			memcpy(systemVars.server_tcp_port, argv[2], sizeof(systemVars.server_tcp_port));
			systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER IP
	if (!strcmp_P( strupr(argv[1]), PSTR("IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_ip_address, '\0', sizeof(systemVars.server_ip_address));
			memcpy(systemVars.server_ip_address, argv[2], sizeof(systemVars.server_ip_address));
			systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER SCRIPT
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.serverScript, '\0', sizeof(systemVars.serverScript));
			memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
			systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* DEBUGLEVEL */
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUGLEVEL\0"))) {
		retS = pv_cmdWrDebugLevel(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* WRKMODE */
	if (!strcmp_P( strupr(argv[1]), PSTR("WRKMODE\0"))) {
		retS = pv_cmdWrkMode(argv[2],argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// TIMERPOLL
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0"))) {
		retS = u_configTimerPoll(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		retS = RTC_str_to_date(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	//----------------------------------------------------------------------
	// COMANDOS USADOS PARA DIAGNOSTICO
	// DEBEMOS ESTAR EN MODO SERVICE
	//----------------------------------------------------------------------

	// GSMBAND:
	// Debo estar en modo service ya que para que tome el valor debe resetearse
	if (!strcmp_P( strupr(argv[1]), PSTR("GSMBAND\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			systemVars.gsmBand = atoi(argv[2]);
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// EE: write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) ) {
		retS = EE_test_write( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSPWR\0"))) {
		switch( atoi(argv[2]) ) {
		case 0:
			IO_modem_hw_pwr_off();
			retS = true;
			break;
		case 1:
			IO_modem_hw_pwr_on();
			retS = true;
			break;
		default:
			retS = false;
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsSW
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSSW\0"))) {
		switch( atoi(argv[2]) ) {
		case 0:
			IO_modem_sw_switch_low();
			retS = true;
			break;
		case 1:
			IO_modem_sw_switch_high();
			retS = true;
			break;
		default:
			retS = false;
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// MCP
	// write mcp 0|1|2 addr value
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		switch( atoi(argv[2] )) {
		case 0:
			retS = MCP_write( MCP0_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;
		case 1:
			retS = MCP_write( MCP1_ADDR, atoi(argv[3]), atoi(argv[4]) );
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
// FUNCIONES PRIVADAS
//-------------------------------------------------------------------------------------
bool pv_cmdWrkMode(char *s0, char *s1)
{
bool retS = false;

	if ((!strcmp_P(strupr(s0), PSTR("MONITOR")))) {

		if ((!strcmp_P( strupr(s1), PSTR("SQE")))) {
			systemVars.wrkMode = WK_MONITOR_SQE;
			retS = true;
			goto quit;
		}

	}

quit:

	return(retS);
}
/*------------------------------------------------------------------------------------*/
bool pv_cmdWrDebugLevel(char *s)
{

	if ((!strcmp_P( strupr(s), PSTR("NONE")))) {
		systemVars.debugLevel = D_NONE;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("+BASIC")))) {
		systemVars.debugLevel += D_BASIC;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-BASIC")))) {
		if ( ( systemVars.debugLevel & D_BASIC) != 0 ) {
			systemVars.debugLevel -= D_BASIC;
			return(true);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DATA")))) {
		systemVars.debugLevel += D_DATA;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DATA")))) {
		if ( ( systemVars.debugLevel & D_DATA) != 0 ) {
			systemVars.debugLevel -= D_DATA;
			return(true);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+MEM")))) {
		systemVars.debugLevel += D_MEM;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-MEM")))) {
		if ( ( systemVars.debugLevel & D_MEM) != 0 ) {
			systemVars.debugLevel -= D_MEM;
			return(true);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+GPRS")))) {
		systemVars.debugLevel += D_GPRS;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-GPRS")))) {
		if ( ( systemVars.debugLevel & D_GPRS) != 0 ) {
			systemVars.debugLevel -= D_GPRS;
			return(true);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DEBUG")))) {
		systemVars.debugLevel += D_DEBUG;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DEBUG")))) {
		if ( ( systemVars.debugLevel & D_DEBUG) != 0 ) {
			systemVars.debugLevel -= D_DEBUG;
			return(true);
		}
	}
	if ((!strcmp_P( strupr(s), PSTR("ALL")))) {
		systemVars.debugLevel = D_DATA + D_GPRS + D_MEM + D_DEBUG;
		return(true);
	}

	return(false);
}
/*------------------------------------------------------------------------------------*/
static uint8_t pv_makeArgv(void)
{
// A partir de la linea de comando, genera un array de punteros a c/token
//
char *token = NULL;
char parseDelimiters[] = " ";
int i = 0;

	// inicialmente todos los punteros deben apuntar a NULL.
	memset(argv, 0, sizeof(argv) );

	// Genero los tokens delimitados por ' '.
	token = strtok(SP5K_CmdlineBuffer, parseDelimiters);
	argv[i++] = token;
	while ( (token = strtok(NULL, parseDelimiters)) != NULL ) {
		argv[i++] = token;
		if (i == 16) break;
	}
	return(( i - 1));
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_OK(void )
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_ERR(void)
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_readMemory(void)
{
	// Leemos la memoria e imprimo los datos.
	// El problema es que si hay muchos datos puede excederse el tiempo de watchdog y
	// resetearse el dlg.
	// Para esto, cada 32 registros pateo el watchdog.

StatBuffer_t pxFFStatBuffer;
frameData_t Aframe;
size_t bRead;
uint8_t pos, channel;
uint16_t rcds = 0;

	FF_seek();
	while(1) {
		bRead = FF_fread( &Aframe, sizeof(Aframe));

		if ( bRead == 0) {
			break;
		}

		if ( ( rcds++ % 32) == 0 ) {
			u_kick_Wdg(WDG_CMD);
		}

		// imprimo
		FF_stat(&pxFFStatBuffer);
		pos = snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("RD:[%d/%d/%d][%d/%d] "), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("frame::{" ));
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR("%02d%02d%02d,"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );

		// Bateria
		pos += snprintf_P( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ), PSTR(",bt=%.2f}\r\n\0"),Aframe.batt );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
}
//------------------------------------------------------------------------------------
