/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include <sp5KV6_PZ.h>
//#include "sp5KV6_PZ_tkGPRS/sp5KV6_PZ_tkGprs.h"

static char cmd_printfBuff[CHAR128];
char *argv[16];

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static uint8_t pv_makeArgv(void);

bool pv_cmdWrDebugLevel(char *s);
static void pv_readMemory(void);
static void pv_cmd_pulse(void);
static void pv_cmd_testpin(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdClearScreen(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdStatusFunction(void);
static void cmdReadFunction(void);
static void cmdWriteFunction(void);
static void cmdKillFunction(void);
static void cmdConfigFunction(void);

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
	cmdlineAddCommand((uint8_t *)("kill"), cmdKillFunction);
	cmdlineAddCommand((uint8_t *)("config"), cmdConfigFunction);

	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"starting tkCmd..\r\n\0");
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
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"\x1B[2J\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void cmdHelpFunction(void)
{

	pv_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]),PSTR("WRITE\0"))) {
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"-write\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  mcp {devId}{regAddr}{regValue}\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  ee addr string\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  led {0|1},gprspwr {0|1},gprssw {0|1}\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  redial\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  pulse {on|off}\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ\0"))) {
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-read\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  mcp {0|1} regAddr\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  rtc, frame\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  ee {addr}{lenght}\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  memory,gprs\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-reset\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  memory\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// HELP CONFIG
	else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-config\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  rtc YYMMDDhhmm\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  timerpoll, dlgid, gsmband\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  debuglevel {none,gprs,range } \r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  apn, roaming {on|off}, port, ip, script, passwd\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  maxrange, C{0..2} chname\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  default\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  save\r\n\0");
	}

	// HELP KILL
	else if (!strcmp_P( strupr(argv[1]), PSTR("KILL\0"))) {
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-kill \r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  {gprstx,gprsrx,range}\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	} else {

		// HELP GENERAL
		memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "\r\nSpymovil %s %s %s %s\r\n\0", SP5K_MODELO, SP5K_VERSION, SP5K_REV, SP5K_DATE);
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "Available commands are:\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-cls\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-status\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-reset...\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-config...\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-read...\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-write...\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "-kill...\r\n\0");
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"\r\n\0");
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
static void cmdKillFunction(void)
{

	pv_makeArgv();

	// KILL GPRSTX
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSTX\0"))) {
		vTaskSuspend( xHandle_tkGprs );
		// Dejo la flag de modem prendido para poder leer comandos
//		GPRS_stateVars.modem_prendido = true;
		return;
	}

	// KILL GPRSRX
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSRX\0"))) {
		vTaskSuspend( xHandle_tkGprsRx );
		return;
	}

	// KILL RANGE
	if (!strcmp_P( strupr(argv[1]), PSTR("RANGE\0"))) {
		vTaskSuspend( xHandle_tkRange );
		return;
	}

	pv_snprintfP_OK();
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdStatusFunction(void)
{

RtcTimeType_t rtcDateTime;
uint16_t pos;
StatBuffer_t pxFFStatBuffer;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "\r\nSpymovil %s %s %s %s\r\n\0", SP5K_MODELO, SP5K_VERSION,SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Last reset info
	pos = FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "Wdg (0x%X",wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), " PORF");
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), " EXTRF");
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), " BORF");
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), " WDRF");
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), " JTRF");
	}
	pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), " )\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DlgId */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "dlgid: %s\r\n\0", systemVars.dlgId );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Fecha y Hora */
	RTC_read(&rtcDateTime);
	pos = FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "rtc: %02d/%02d/%04d ",rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
	pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "%02d:%02d:%02d\r\n\0",rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), ">Server:\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* APN */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  apn: %s\r\n\0", systemVars.apn );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER IP:SERVER PORT */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  server ip:port: %s:%s\r\n\0", systemVars.server_ip_address,systemVars.server_tcp_port );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER SCRIPT */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  server script: %s\r\n\0", systemVars.serverScript );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER PASSWD */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  passwd: %s\r\n\0", systemVars.passwd );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MODEM ---------------------------------------------------------------------------------------
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), ">Modem:\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Modem band */
	pos = FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  band: ");
	switch ( systemVars.gsmBand) {
	case 0:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "(900)");
		break;
	case 1:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "(1800)");
		break;
	case 2:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "dual band (900/1800)");
		break;
	case 3:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "pcs (1900)");
		break;
	case 4:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "gsm (850)");
		break;
	case 5:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "dual band (1900/850)");
		break;
	case 6:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "triband (900/1800/1900)");
		break;
	case 7:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "triband (850/1800/1900)");
		break;
	case 8:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "cuatriband (850/900/1800/1900)");
		break;
	}
	pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* ROAMING */
	if ( systemVars.roaming == true ) {
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  roaming ON\r\n\0");
	} else {
		FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  roaming OFF\r\n\0");
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DLGIP */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  dlg ip: %s\r\n\0", systemVars.dlg_ip_address );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CSQ */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  signalQ: csq=%d, dBm=%d\r\n\0", systemVars.csq, systemVars.dbm );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// GPRS STATE
/*	pos = FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  state: ");
	switch (GPRS_stateVars.state) {
	case G_ESPERA_APAGADO:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "await_off\r\n");
		break;
	case G_PRENDER:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "prendiendo\r\n");
		break;
	case G_CONFIGURAR:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "configurando\r\n");
		break;
	case G_MON_SQE:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "mon_sqe\r\n");
		break;
	case G_GET_IP:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "ip\r\n");
		break;
	case G_INIT_FRAME:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "init frame\r\n");
		break;
	case G_DATA:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "data\r\n");
		break;
	default:
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "ERROR\r\n");
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
*/
	// SYSTEM ---------------------------------------------------------------------------------------
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), ">System:\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Memoria */
	FF_stat(&pxFFStatBuffer);
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  memory: wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d \r\n", pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CONFIG */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), ">Config:\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Timers */
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  timerPoll: [%ds]\r\n\0",systemVars.timerPoll );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DebugLevel */
	pos = FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  debugLevel: ");
	if ( systemVars.debugLevel == D_NONE) {
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "none" );
	} else 	if ( systemVars.debugLevel == D_GPRS) {
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "gprs" );
	} else 	if ( systemVars.debugLevel == D_RANGE) {
		pos += FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "range" );
	}
	FRTOS_snprintf( &cmd_printfBuff[pos],sizeof(cmd_printfBuff), "\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MaxRange
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  maxRange=%d\r\n\0",systemVars.maxRange);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Channel Names
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  Ch0(range): %s\r\n\0",systemVars.chName[0] );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  Ch1(dig0):  %s\r\n\0",systemVars.chName[1] );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "  Ch2(dig1):  %s\r\n\0",systemVars.chName[2] );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );


}
/*------------------------------------------------------------------------------------*/
static void cmdReadFunction(void)
{
uint8_t argc;
char datetime[24];
bool retS = false;
uint8_t regValue;

	argc = pv_makeArgv();

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		retS = EE_test_read( argv[2], cmd_printfBuff, argv[3] );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			FRTOS_snprintf( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),  "\r\n\0");
			FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		if ( RTC_date_to_str( datetime, sizeof(datetime) ) != -1 ) {
			FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "OK\r\n%s\r\n\0", datetime );
		} else {
			FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "ERROR\r\n\0");
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
			FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "OK\r\n[reg 0X%03x]=[0X%03x]\r\n\0",atoi(argv[3]),regValue);
		} else {
			FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "ERROR\r\n\0");
		}
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// DEFAULT (load default configuration)
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULTS\0"))) {
		u_loadDefaults();
		return;
	}

	// FRAME

	// MEMORY
	if (!strcmp_P( strupr(argv[1]),  "MEMORY\0")) {
		pv_readMemory();
		return;
	}

	// CMD NOT FOUND
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "ERROR\r\nCMD NOT DEFINED\r\n");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdWriteFunction(void)
{
bool retS = false;

	pv_makeArgv();

	// PULSE
	if (!strcmp_P( strupr(argv[1]), PSTR("PULSE\0")) ) {
		pv_cmd_pulse();
		return;
	}

	// TESTPIN
	if (!strcmp_P( strupr(argv[1]), PSTR("TESTPIN\0")) ) {
		pv_cmd_testpin();
		return;
	}

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
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
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

	// REDIAL
	// Envio un mensaje a la tk_Gprs para que recargue la configuracion y disque al server
	// Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
	if (!strcmp_P( strupr(argv[1]), PSTR("REDIAL\0"))) {

		while ( xTaskNotify(xHandle_tkGprsRx,TK_REDIAL , eSetBits ) != pdPASS ) {
			vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		}

	}

	// CMD NOT FOUND
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "ERROR\r\nCMD NOT DEFINED\r\n");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
static void cmdConfigFunction(void)
{
bool retS = false;

	pv_makeArgv();

	if (!strcmp_P( strupr(argv[1]), PSTR("MAXRANGE\0"))) {
		retS = u_configMaxRange(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CANALES ANALOGICOS
	if (!strcmp_P( strupr(argv[1]), PSTR("C0\0"))) {
		retS = u_configChName( 0, argv[2] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("C1\0"))) {
		retS = u_configChName( 1, argv[2] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	if (!strcmp_P( strupr(argv[1]), PSTR("C2\0"))) {
		retS = u_configChName( 2, argv[2] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

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

	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT\0"))) {
		u_loadDefaults();
		pv_snprintfP_OK();
		return;
	}


	// CMD NOT FOUND
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "ERROR\r\nCMD NOT DEFINED\r\n");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
/*------------------------------------------------------------------------------------*/
// FUNCIONES PRIVADAS
//-------------------------------------------------------------------------------------
bool pv_cmdWrDebugLevel(char *s)
{

	if ((!strcmp_P( strupr(s), PSTR("NONE")))) {
		systemVars.debugLevel = D_NONE;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("GPRS")))) {
		systemVars.debugLevel = D_GPRS;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("RANGE")))) {
		systemVars.debugLevel = D_RANGE;
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
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "OK\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_snprintfP_ERR(void)
{
	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "ERROR\r\n\0");
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
uint16_t rcds = 0;
uint16_t pos;

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
		pos = FRTOS_snprintf( cmd_printfBuff, sizeof(cmd_printfBuff),  "RD:[%d/%d/%d][%d/%d] ", pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		pos += FRTOS_snprintf( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),  "frame::{" );
		pos += FRTOS_snprintf( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),  "%04d%02d%02d,",Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
		pos += FRTOS_snprintf( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),  "%02d%02d%02d",Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );

		// Datos
		pos += FRTOS_snprintf( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),  ",%s=%d", systemVars.chName[0], Aframe.range );
		pos += FRTOS_snprintf( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),  ",%s=%d", systemVars.chName[1], Aframe.dig0 );
		pos += FRTOS_snprintf( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),  ",%s=%d", systemVars.chName[2], Aframe.dig1 );
		pos += FRTOS_snprintf( &cmd_printfBuff[pos], ( sizeof(cmd_printfBuff) - pos ),  "}\r\n\0" );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}
}
//------------------------------------------------------------------------------------
static void pv_cmd_pulse(void)
{
	// pulse on|off
	// El pulso lo vemos del lado del sensor por lo que al pasar por un inversor
	// debemos cambiar el sentido
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_clr_UPULSE_RUN();
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_set_UPULSE_RUN();
		pv_snprintfP_OK();
		return;
	}

	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "cmd ERROR: ( write pulse on{off} )\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_testpin(void)
{
	// pulse on|off
	// El pulso lo vemos del lado del sensor por lo que al pasar por un inversor
	// debemos cambiar el sentido
	if (!strcmp_P( strupr(argv[2]), PSTR("ON\0")) ) {
		IO_set_TEST_PIN();
		pv_snprintfP_OK();
		return;
	}

	if (!strcmp_P( strupr(argv[2]), PSTR("OFF\0")) ) {
		IO_clear_TEST_PIN();
		pv_snprintfP_OK();
		return;
	}

	FRTOS_snprintf( cmd_printfBuff,sizeof(cmd_printfBuff), "cmd ERROR: ( write testpin on{off} )\r\n\0");
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
