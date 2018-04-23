/*
 * sp5KV5_tkGprs_data.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */


#include "sp5KV6_PZ_tkGprs.h"

static bool pv_hay_datos_para_trasmitir(void);
static bool pv_trasmitir_paquete_de_datos(void );

static bool pv_check_socket_open(void);

static void pv_trasmitir_dataHeader( void );
static void pv_trasmitir_dataTail( void );
static void pv_trasmitir_dataRecord( void );

static uint8_t pv_procesar_respuesta(void);
static void pv_process_response_RESET(void);
static uint8_t pv_process_response_OK(void);
static bool pv_check_more_Rcds4Del ( void );

static bool pv_procesar_signals_data( bool *exit_flag );

//------------------------------------------------------------------------------------
bool gprs_data(void)
{
	// Me quedo en un loop permanente revisando si hay datos y si hay los trasmito
	// Solo salgo en caso que hubiesen problemas para trasmitir
	// Esto es que no puedo abrir un socket mas de 3 veces o que no puedo trasmitir
	// el mismo frame mas de 3 veces.
	// En estos casos salgo con false de modo de hacer un ciclo completo apagando el modem.
	// Si por algun problema no puedo trasmitir, salgo asi me apago y reinicio.
	// Si pude trasmitir o simplemente no hay datos, en modo continuo retorno TRUE.

	// Si hay datos los trasmito todos
	// Solo salgo en caso que hubiesen problemas para trasmitir
	// Esto es que no puedo abrir un socket mas de 3 veces o que no puedo trasmitir
	// el mismo frame mas de 3 veces.
	// En estos casos salgo con false de modo de hacer un ciclo completo apagando el modem.

	// Mientras estoy trasmitiendo datos no atiendo las señales; solo durante la espera en modo
	// continuo.

uint8_t sleep_time;
bool exit_flag = false;

	GPRS_stateVars.state = G_DATA;

	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::data: \r\n\0", u_now() );
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	//
	while ( 1 ) {

		// Si por algun problema no puedo trasmitir, salgo asi me apago y reinicio.
		// Si pude trasmitir o simplemente no hay datos, en modo continuo retorno TRUE.


		if ( pv_hay_datos_para_trasmitir() ) {			// Si hay datos, intento trasmitir

			if ( ! pv_trasmitir_paquete_de_datos() ) {	// Si tuve errores de trasmision, salgo
				exit_flag = bool_RESTART;
				goto EXIT;
			}

		} else {

			// No hay datos para trasmitir
			// Modo discreto, Salgo a apagarme y esperar

			// modo continuo: espero 90s antes de revisar si hay mas datos para trasmitir
			sleep_time = 90;
			while( sleep_time-- > 0 ) {

				// PROCESO LAS SEÑALES
				if ( pv_procesar_signals_data( &exit_flag )) {
					// Si recibi alguna senal, debo salir.
					goto EXIT;
				}

				vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );

			} // while


		} // else

	} // while

	// Exit area:
EXIT:
	// No espero mas y salgo del estado prender.
	return(exit_flag);
}
//------------------------------------------------------------------------------------
static bool pv_procesar_signals_data( bool *exit_flag )
{

bool ret_f = false;

	if ( GPRS_stateVars.signal_redial) {
		// Salgo a discar inmediatamente.
		*exit_flag = bool_RESTART;
		ret_f = true;
		goto EXIT;
	}

	if ( GPRS_stateVars.signal_frameReady) {
		goto EXIT;
	}

	ret_f = false;
EXIT:

	GPRS_stateVars.signal_redial = false;
	GPRS_stateVars.signal_frameReady = false;

	return(ret_f);

}
//------------------------------------------------------------------------------------
static bool pv_hay_datos_para_trasmitir(void)
{

	// Veo si hay datos en memoria para trasmitir

StatBuffer_t pxFFStatBuffer;
bool exit_flag;
uint16_t pos = 0;

	FF_stat(&pxFFStatBuffer);

	// 1 -La memoria esta vacia en absoluto ( Free = MAX )
	// o en lectura, es decir que lei todos los registros ocupados.
	//
	if ( pxFFStatBuffer.rcdsFree == FF_MAX_RCDS) {
		// Memoria vacia en absoluto: No trasmito
		exit_flag = false;
	} else if ( pxFFStatBuffer.rcdsFree == 0 ) {
		// Memoria llena: Trasmito
		exit_flag = true;
	} else if ( pxFFStatBuffer.RD == pxFFStatBuffer.HEAD ) {
		// Memoria con datos pero todos trasmitidos
		exit_flag = false;
	} else {
		// Hay datos para trasmitir
		exit_flag = true;
	}

	if ( systemVars.debugLevel == D_GPRS ) {
		pos = FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::data: [wrPtr=%d,rdPtr=%d,delPtr=%d][Free=%d,4del=%d]",u_now(), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		if ( exit_flag == false ) {
			pos += FRTOS_snprintf( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos )," EMPTY\r\n\0");
		} else {
			pos += FRTOS_snprintf( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),"\r\n\0" );
		}
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	return(exit_flag);

}
//------------------------------------------------------------------------------------
static bool pv_trasmitir_paquete_de_datos(void )
{
	// Hay datos que intento trasmitir.
	// Leo los mismos hasta completar un TX_WINDOW
	// El socket lo chequeo antes de comenzar a trasmitir. Una vez que comienzo
	// trasmito todo y paso a esperar la respuesta donde chequeo el socket nuevamente.
	// Intento enviar el mismo paquete de datos hasta 3 veces.
	// Entro luego de haber chequeado que hay registros para trasmitir o sea que se que los hay !!

uint8_t registros_trasmitidos = 0;
uint8_t registros_procesados = 0;
uint8_t win_tries;
bool exit_flag = false;

	// TRASMITO
	for ( win_tries = 0; win_tries < MAX_TX_WINDOW_TRYES; win_tries++ ) {

		// Paso 1: Me aseguro que el socket este abierto. Si esta cerrado intento
		//         abrirlo. Si al final no puedo, salgo con error.
		if ( ! pv_check_socket_open() ) {
			exit_flag = false;			// Error por no poder abrir el socket
			goto EXIT;
		}

		// Paso 2: Trasmito un paquete de datos
		registros_trasmitidos = 0;
		pv_trasmitir_dataHeader();

		while ( pv_hay_datos_para_trasmitir() && ( registros_trasmitidos < MAX_RCDS_WINDOW_SIZE ) ) {
			pv_trasmitir_dataRecord();
			registros_trasmitidos++;
		}

		pv_trasmitir_dataTail();

		// Paso 3: Espero la respuesta y la proceso
		registros_procesados = pv_procesar_respuesta();
		if ( registros_procesados == 0 ) {
			exit_flag = false;		// Error por no poder procesar registros transmitidos
		} else {
			exit_flag = true;		// OK al procesar los registros
			goto EXIT;
		}

	}

	// Expiraron los intenteos MAX_TX_WINDOW_TRYES
	exit_flag = false;			// No procese nada: Salgo

EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_trasmitir_dataHeader( void )
{

uint16_t pos;

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL,false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();

	// Armo el header en el buffer
	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"GET ");
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"%s", systemVars.serverScript );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"?DLGID=%s", systemVars.dlgId );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"&PASSWD=%s", systemVars.passwd );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"&VER=%s", SP5K_REV );

	// Trasmito
	FreeRTOS_write( &pdUART0, gprs_printfBuff, pos );

	// DebugMsg
	if ( systemVars.debugLevel == D_GPRS ) {
		g_print_debug_gprs_header("GPRS::data:");
		FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"\r\n\0" );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static void pv_trasmitir_dataTail( void )
{
	// TAIL : No mando el close ya que espero la respuesta del server

uint16_t pos = 0;

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();

	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));

	pos = FRTOS_snprintf( gprs_printfBuff, ( sizeof(gprs_printfBuff) - pos )," HTTP/1.1\n" );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),"Host: www.spymovil.com\n");
	pos += FRTOS_snprintf( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),"\n\n\0" );

	// Trasmito
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DebugMsg
	if ( systemVars.debugLevel == D_GPRS ) {
		g_print_debug_gprs_header("GPRS::data:");
		FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"\r\n\0");
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::data: Frame enviado\r\n\0", u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static void pv_trasmitir_dataRecord( void )
{

uint16_t pos;
frameData_t Aframe;
StatBuffer_t pxFFStatBuffer;

	// Paso1: Leo un registro de memoria
	FF_fread( &Aframe, sizeof(Aframe));
	FF_stat(&pxFFStatBuffer);

	// Paso2: Armo el frame
	// Siempre trasmito los datos aunque vengan papasfritas.
	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	// Indice de la linea
	pos = FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"&CTL=%d", pxFFStatBuffer.RD );
	//
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"&LINE=" );
	// Fecha y hora
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"%04d%02d%02d,",Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
	pos += FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"%02d%02d%02d",Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );

	// C0 ( Range )
	pos += FRTOS_snprintf( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), ",%s=%d", systemVars.chName[0], Aframe.range );
	// C1
	pos += FRTOS_snprintf( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), ",%s=%d", systemVars.chName[1], Aframe.dig0 );
	// C2
	pos += FRTOS_snprintf( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), ",%s=%d", systemVars.chName[2], Aframe.dig1 );

	// Paso 3: Trasmito por el modem sin el \r
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, gprs_printfBuff, pos );

	// Paso 4: Log
	if ( systemVars.debugLevel == D_GPRS ) {
		g_print_debug_gprs_header("GPRS::data:");
		//FRTOS_snprintf( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),"\r\n\0");
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		FreeRTOS_write( &pdUART1, "\r\n", sizeof("\r\n") );

		// Agrego mem.stats
		if (pxFFStatBuffer.errno > 0 ) {
			FRTOS_snprintf( gprs_printfBuff,  sizeof(gprs_printfBuff), "%s GPRS::data: ERROR (%d) MEM[%d/%d/%d][%d/%d]\r\n\0", u_now(), pxFFStatBuffer.errno, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		} else {
			FRTOS_snprintf( gprs_printfBuff, sizeof(gprs_printfBuff), "%s GPRS::data: sent OK. MEM[%d/%d/%d][%d/%d]\r\n\0", u_now(), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		}
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}
}
//------------------------------------------------------------------------------------
static bool pv_check_socket_open(void)
{
	// Debo salir con el socket abierto o con ERROR ( false ).
	// Si el socket esta abierto, salgo.
	// Si esta cerrado, intento hasta 3 veces abrirlo.

uint8_t i;
bool return_flag = false;

	for ( i = 0; i < MAX_TRYES_OPEN_SOCKET; i++ ) {

		if ( ! g_socket_is_open() ) {		// Si el socket no esta abierto
			g_open_socket();				// lo intento abrir.
		} else {
			return_flag = true;			// Si esta abierto
			break;						// me voy.
		}
	}

	return( return_flag);

}
//------------------------------------------------------------------------------------
static uint8_t pv_procesar_respuesta(void)
{
	// Me quedo hasta 10s esperando la respuesta del server al paquete de datos.
	// Salgo por timeout, socket cerrado, error del server o respuesta correcta
	// Si la respuesta es correcta, ejecuto la misma
	// RETURN: la cantidad de registros confirmados.
	// Si hubo error retorna 0.

uint8_t timeout;
uint8_t recds_procesados = 0;

	timeout = 10;
	while ( timeout-- > 0) {

		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

		// Si el socket se cerro salgo
		if ( ! g_socket_is_open() ) {
			return(0);
		}

		// Evaluo las respuestas del server

		if ( strstr( gprsRx.buffer, "RESET") != NULL ) {
			// El sever mando la orden de resetearse inmediatamente
			// Muestro mensaje de respuesta del server.
			g_printRxBuffer();
			pv_process_response_RESET();
			return(0);
		}

		if ( strstr( gprsRx.buffer, "RX_OK") != NULL ) {
			// Datos procesados por el server.
			// Muestro mensaje de respuesta del server.
			g_printRxBuffer();
			recds_procesados = pv_process_response_OK();
			return(recds_procesados);
		}

		if ( strstr( gprsRx.buffer, "ERROR") != NULL ) {
			// ERROR del server: salgo inmediatamente
			// Muestro mensaje de respuesta del server.
			g_printRxBuffer();
			return(0);
		}

	}

	// Sali por timeout
	return(0);

}
//------------------------------------------------------------------------------------
static void pv_process_response_RESET(void)
{
	// El server me pide que me resetee de modo de mandar un nuevo init y reconfigurarme

	FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::data: Config RESET...\r\n\0",u_now());
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	// RESET
	u_reset();
}
//------------------------------------------------------------------------------------
static uint8_t pv_process_response_OK(void)
{
	// Retorno la cantidad de registros procesados ( y borrados )
	// Recibi un OK del server y el ultimo ID de registro a borrar.
	// Los borro de a uno.

StatBuffer_t pxFFStatBuffer;
uint8_t recds_borrados = 0;

	// Borro los registros.
	while ( pv_check_more_Rcds4Del() ) {

		FF_del();
		recds_borrados++;
		FF_stat(&pxFFStatBuffer);

		if ( systemVars.debugLevel == D_GPRS ) {
			FRTOS_snprintf( gprs_printfBuff,sizeof(gprs_printfBuff),"%s GPRS::data: [wrPtr=%d,rdPtr=%d,delPtr=%d][Free=%d,4del=%d]\r\n\0",u_now(), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

	}

	return(recds_borrados);
}
//------------------------------------------------------------------------------------
static bool pv_check_more_Rcds4Del ( void )
{
	// Devuelve si aun quedan registros para borrar del FS

StatBuffer_t pxFFStatBuffer;

	FF_stat(&pxFFStatBuffer);

	if ( FCB.ff_stat.rcds4del > 0 ) {
		return(true);
	} else {
		return(false);
	}

}
//------------------------------------------------------------------------------------
