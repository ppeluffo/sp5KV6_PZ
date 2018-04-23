/*
 * sp5KV5_tkGprs_init_frame.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "sp5KV6_PZ_tkGprs.h"

bool pv_send_init_frame(void);
bool pv_process_init_response(void);
static void pv_TX_init_frame(void);
static void pv_reconfigure_params(void);

static void pv_process_server_clock(void);
static uint8_t pv_process_timerPoll(void);
static uint8_t pv_process_maxRange(void);

//------------------------------------------------------------------------------------
bool gprs_init_frame(void)
{
	// Debo mandar el frame de init al server, esperar la respuesta, analizarla
	// y reconfigurarme.
	// Intento 3 veces antes de darme por vencido.
	// El socket puede estar abierto o cerrado. Lo debo determinar en c/caso y
	// si esta cerrado abrirlo.
	// Mientras espero la respuesta debo monitorear que el socket no se cierre

uint8_t intentos;
bool exit_flag = false;

// Entry:

	GPRS_stateVars.state = G_INIT_FRAME;

	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe:\r\n\0", u_now());
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Intenteo MAX_INIT_TRYES procesar correctamente el INIT
	for ( intentos = 0; intentos < MAX_INIT_TRYES; intentos++ ) {

		if ( ! pv_send_init_frame() ) {			// Intento madar el frame al servidor
			FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Send retry\r\n\0", u_now() );
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			continue;
		}

		if ( ! pv_process_init_response() ) {	// Intento procesar la respuesta
			FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Rsp retry\r\n\0", u_now() );
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			continue;
		}

		// Aqui es que anduvo todo bien y debo salir para pasar al modo DATA
		FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Init frame OK.\r\n\0", u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

		exit_flag = true;
		goto EXIT;

	}

	// Aqui es que no puede enviar/procesar el INIT correctamente
	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Init frame FAIL !!.\r\n\0", u_now() );
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

// Exit
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
bool pv_send_init_frame(void)
{
	// Intento enviar 1 SOLO frame de init.
	// El socket puede estar cerrado por lo que reintento abrirlo hasta 3 veces.
	// Una vez que envie el INIT, salgo.
	// Al entrar, veo que el socket este cerrado.

uint8_t i;
bool exit_flag = false;

	for ( i = 0; i < MAX_TRYES_OPEN_SOCKET; i++ ) {

		//g_sleep(5);			// Espero 5s
		if ( g_socket_is_open() ) {
			pv_TX_init_frame();		// Escribo en el socket el frame de INIT
			exit_flag = true;
			break;
		}

		g_open_socket();
	}

	return(exit_flag);
}
//------------------------------------------------------------------------------------
bool pv_process_init_response(void)
{
	// Espero la respuesta al frame de INIT.
	// Si la recibo la proceso.
	// Salgo por timeout 10s o por socket closed.

uint8_t timeout;
bool exit_flag = false;

	for ( timeout = 0; timeout < 10; timeout++) {

		g_sleep(1);				// Espero 1s

		if ( ! g_socket_is_open() ) {		// El socket se cerro
			exit_flag = false;
			goto EXIT;
		}

		if ( strstr( gprsRx.buffer, "ERROR") != NULL ) {	// Recibi un ERROR de respuesta
			g_printRxBuffer();
			exit_flag = false;
			goto EXIT;
		}

		if ( strstr( gprsRx.buffer, "INIT_OK") != NULL ) {	// Respuesta correcta del server
			g_printRxBuffer();
			pv_reconfigure_params();
			exit_flag = true;
			goto EXIT;
		}

	}

// Exit:
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_TX_init_frame(void)
{
	// Send Init Frame
	// GET /cgi-bin/sp5K/sp5K.pl?DLGID=SPY001&PASSWD=spymovil123&&INIT&ALARM&PWRM=CONT&TPOLL=23&TDIAL=234&PWRS=1,1230,2045&A0=pZ,1,20,3,10&D0=qE,3.24&CONS=1,1234,927,1,3 HTTP/1.1
	// Host: www.spymovil.com
	// Connection: close\r\r ( no mando el close )

uint16_t pos = 0;

	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Sent\r\n\0", u_now() );
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Trasmision: 1r.Parte.
	// HEADER:
	// Envio parcial ( no CR )
	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();

	pos = FRTOS_snprintf( gprs_printfBuff,CHAR256,"GET " );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"%s", systemVars.serverScript );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"?DLGID=%s", systemVars.dlgId );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"&PASSWD=%s", systemVars.passwd );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"&IMEI=%s", g_getImei() );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"&VER=%s\0", SP5K_REV );
	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( systemVars.debugLevel == D_GPRS ) {
		FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"\r\n\0" );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// BODY ( 1a parte) :
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = FRTOS_snprintf( gprs_printfBuff ,CHAR256,"&INIT");

	// timerpoll
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"&TPOLL=%d", systemVars.timerPoll);

	// maxRange
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"&MRANGE=%d", systemVars.maxRange);
	// csq
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"&CSQ=%d\0", systemVars.csq);

	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( systemVars.debugLevel == D_GPRS ) {
		FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"\r\n\0" );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// BODY ( 2a parte) :
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = 0;

	// Configuracion de canales analogicos

	// Reset status
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( CHAR256 - pos ),"&WDG=%d\0",wdgStatus.resetCause );

	// GPRS send
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( systemVars.debugLevel == D_GPRS ) {
		FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"\r\n\0");
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// TAIL ( No mando el close ya que espero la respuesta y no quiero que el socket se cierre )
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = FRTOS_snprintf( gprs_printfBuff, ( sizeof(gprs_printfBuff) - pos )," HTTP/1.1\n" );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),"Host: www.spymovil.com\n" );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos], sizeof(gprs_printfBuff),"\n\n\0");

	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( systemVars.debugLevel == D_GPRS ) {
		FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"\r\n\0" );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static void pv_reconfigure_params(void)
{

uint8_t saveFlag = 0;

	// Proceso la respuesta del INIT para reconfigurar los parametros
	pv_process_server_clock();
	saveFlag += pv_process_timerPoll();
	saveFlag += pv_process_maxRange();

	if ( saveFlag > 0 ) {

		if ( u_saveSystemParams() ) {

			// DEBUG & LOG
			if ( systemVars.debugLevel == D_GPRS ) {
				FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Save params OK\r\n\0", u_now());
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}

		}
	}

}
//------------------------------------------------------------------------------------
static void pv_process_server_clock(void)
{
/* Extraigo el srv clock del string mandado por el server y si el drift con la hora loca
 * es mayor a 5 minutos, ajusto la hora local
 * La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:PWRM=DISC:</h1>
 *
 */

char *p, *s;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char rtcStr[12];
uint8_t i;
char c;

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "CLOCK");
	if ( p == NULL ) {
		return;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);			// CLOCK

	token = strsep(&stringp,delim);			// rtc
	memset(rtcStr, '\0', sizeof(rtcStr));
//	memcpy(rtcStr,token, sizeof(rtcStr));	// token apunta al comienzo del string con la hora
	for ( i = 0; i<12; i++) {
		c = *token;
		rtcStr[i] = c;
		c = *(++token);
		if ( c == '\0' )
			break;

	}
	RTC_str_to_date(rtcStr);

	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Update rtc to: %s\r\n\0", u_now(), rtcStr );
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

}
//------------------------------------------------------------------------------------
static uint8_t pv_process_timerPoll(void)
{
//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:PWRM=DISC:</h1>

char *p, *s;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "TPOLL");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TPOLL

	token = strsep(&stringp,delim);	// timerPoll
	u_configTimerPoll(token);
	ret = 1;
	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Reconfig TPOLL\r\n\0", u_now());
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
static uint8_t pv_process_maxRange(void)
{
//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:PWRM=DISC:MRANGE=500:</h1>

char *p, *s;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "MRANGE");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// MRANGE

	token = strsep(&stringp,delim);	// maxRange
	u_configMaxRange(token);
	ret = 1;
	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Reconfig MRANGE\r\n\0", u_now());
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
void pv_process_chName(uint8_t channel)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=q0,1.00:D1=q1,1.00</h1>

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName;
char *s;

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	switch (channel) {
	case 0:
		stringp = strstr(s, "C0=");
		break;
	case 1:
		stringp = strstr(s, "C1=");
		break;
	case 2:
		stringp = strstr(s, "C2=");
		break;
	default:
		goto quit;
		break;
	}

	if ( stringp == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp,31);

	stringp = localStr;
	token = strsep(&stringp,delim);		//C0

	chName = strsep(&stringp,delim);	//name

	u_configChName( channel, chName );

	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::iniframe: Reconfig C%d\r\n\0", u_now(), channel);
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

quit:
	return;
}
//--------------------------------------------------------------------------------------
