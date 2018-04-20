/*
 * sp5KV3_utils.c
 *
 *  Created on: 27/10/2015
 *      Author: pablo
 */

#include <sp5KV6_PZ.h>

static uint8_t pv_paramLoad(uint8_t* data, uint8_t* addr, uint16_t sizebytes);
static uint8_t pv_paramStore(uint8_t* data, uint8_t* addr, uint16_t sizebytes);
static uint8_t pv_checkSum ( uint8_t *data,uint16_t sizebytes );

//----------------------------------------------------------------------------------------
void u_uarts_ctl(uint8_t cmd)
{

	cbi(UARTCTL_PORT, UARTCTL);	// Habilito el LM365

}
//----------------------------------------------------------------------------------------
bool u_configTimerPoll(char *s_tPoll)
{
	// Configura el tiempo de poleo.
	// El cambio puede ser desde tkCmd o tkGprs(init frame)
	// Le avisa a la tarea tkAnalog del cambio

uint16_t tpoll;

	tpoll = abs((uint16_t) ( atol(s_tPoll) ));
	if ( tpoll < 15 ) { tpoll = 15; }

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.timerPoll = tpoll;
	xSemaphoreGive( sem_SYSVars );

	return(true);
}
//------------------------------------------------------------------------------------
void u_kick_Wdg( uint8_t wdgId )
{
	// Pone el correspondiente bit del wdg en 0.
	systemWdg &= ~wdgId ;

}
//------------------------------------------------------------------------------------
bool u_saveSystemParams(void)
{
	// Salva el systemVars en la EE y verifica que halla quedado bien.
	// Hago hasta 3 intentos.

bool retS = false;
uint8_t storeChecksum = 0;
uint8_t loadChecksum = 0;
uint8_t i;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	for ( i=0; i<3; i++ ) {
		storeChecksum = pv_paramStore( (uint8_t *)&systemVars, (uint8_t *)EEADDR_SV, sizeof(systemVarsType));
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		pv_paramLoad( (uint8_t *)&tmpSV, (uint8_t *)EEADDR_SV, sizeof(systemVarsType));
		loadChecksum = pv_checkSum( (uint8_t *)&tmpSV,sizeof(systemVarsType));

		if ( loadChecksum == storeChecksum ) {
			retS = true;
			break;
		}
	}

	xSemaphoreGive( sem_SYSVars );
	return(retS);

}
//------------------------------------------------------------------------------------
bool u_loadSystemParams(void)
{
bool retS = false;
uint8_t i;

	// Leo la configuracion:  Intento leer hasta 3 veces.

	for ( i=0; i<3;i++) {
		retS =  pv_paramLoad( (uint8_t *)&systemVars, (uint8_t *)EEADDR_SV, sizeof(systemVarsType));
		if ( retS )
			break;
	}

	// Ajustes de inicio:
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	//systemVars.debugLevel = D_BASIC;

	// Cuando arranca si la EE no esta inicializada puede dar cualquier cosa.
	// De este modo controlo el largo de los strings.
	systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
	systemVars.apn[APN_LENGTH - 1] = '\0';
	systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
	systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
	systemVars.dlg_ip_address[IP_LENGTH - 1] = '\0';
	systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
	systemVars.passwd[PASSWD_LENGTH - 1] = '\0';

	return(retS);

}
//------------------------------------------------------------------------------------
void u_loadDefaults(void)
{

// Configura el systemVars con valores por defecto.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.initByte = 0x49;
	strncpy_P(systemVars.dlgId, PSTR("DEF400\0"),DLGID_LENGTH);
	strncpy_P(systemVars.server_tcp_port, PSTR("80\0"),PORT_LENGTH	);
	strncpy_P(systemVars.passwd, PSTR("spymovil123\0"),PASSWD_LENGTH);
	strncpy_P(systemVars.serverScript, PSTR("/cgi-bin/sp5K/sp5K.pl\0"),SCRIPT_LENGTH);

	systemVars.csq = 0;
	systemVars.dbm = 0;
	systemVars.gsmBand = 8;
	strncpy_P(systemVars.apn, PSTR("SPYMOVIL.VPNANTEL\0"),APN_LENGTH);
	systemVars.roaming = false;

	// DEBUG
	systemVars.debugLevel = D_BASIC;

	strncpy_P(systemVars.server_ip_address, PSTR("192.168.0.9\0"),IP_LENGTH);
	systemVars.timerPoll = 300;			// Poleo c/5 minutos

	xSemaphoreGive( sem_SYSVars );


}
//------------------------------------------------------------------------------------
char *u_now(void)
{

	// Devuelve un puntero a un string con la fecha y hora formateadas para usar en
	// los mensajes de log.

RtcTimeType_t rtcDateTime;

	RTC_read(&rtcDateTime);
	rtcDateTime.year -= 2000;
	snprintf_P( nowStr,sizeof(nowStr), PSTR("%02d/%02d/%02d %02d:%02d:%02d\0"),rtcDateTime.day,rtcDateTime.month,rtcDateTime.year,rtcDateTime.hour,rtcDateTime.min,rtcDateTime.sec );
	return(nowStr);
}
//------------------------------------------------------------------------------------
void u_debugPrint(uint8_t debugCode, char *msg, uint16_t size)
{

	if ( (systemVars.debugLevel & debugCode) != 0) {
		FreeRTOS_write( &pdUART1, msg, size );
	}
}
//------------------------------------------------------------------------------------
void u_reset(void)
{
	wdt_enable(WDTO_30MS);
	while(1) {}
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static uint8_t pv_paramLoad(uint8_t* data, uint8_t* addr, uint16_t sizebytes)
{
uint16_t i;
uint8_t checksum_stored=0;
uint8_t checksum=0;

	// load parameters
	eeprom_read_block(data, (uint8_t *)addr, sizebytes);
	// load checksum
	eeprom_read_block(&checksum_stored, (uint8_t *)(addr+sizebytes), sizeof(uint8_t));

	// calculate own checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;

	if(checksum == checksum_stored)
		return true;
	else
		return false;
}
//------------------------------------------------------------------------------------
static uint8_t pv_paramStore(uint8_t* data, uint8_t* addr, uint16_t sizebytes)
{
	// Almacena un string de bytes en la eeprom interna del micro

uint16_t i;
uint8_t checksum=0;

	// calculate checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;

	// store parameters
	 eeprom_write_block(data, (uint8_t *)addr, sizebytes);
	// store checksum
	eeprom_write_block(&checksum, (uint8_t *)(addr+sizebytes), sizeof(uint8_t));

	return(checksum);
}
//------------------------------------------------------------------------------------
static uint8_t pv_checkSum ( uint8_t *data,uint16_t sizebytes )
{
uint16_t i;
uint8_t checksum=0;

	// calculate checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;
	return(checksum);
}
//------------------------------------------------------------------------------------
