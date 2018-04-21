/*
 * spx_tkPulses.c
 *
 *  Created on: 11 de abr. de 2018
 *      Author: pablo
 */


#include <sp5KV6_PZ.h>

typedef enum { RISING_EDGE = 0, FALLING_EDGE } t_edge_sensing;

#define MAX_PULSE_STACK 32

static struct {
	uint8_t ptr;
	int32_t stack[MAX_PULSE_STACK];
} s_pulse_stack;

static char range_printfBuff[CHAR128];

void pv_rangeMeter_start(void);
void pv_rangeMeter_stop(void);
void pv_rangeMeter_init(void);
void pv_flush_stack_pulses(void);
void pv_push_stack_pulses(uint16_t counter);
int16_t pv_pulse_calcular_distancia(void);
void pv_pulsos_stats(uint16_t *avg, float *var);
void pv_rangeMeter_ping(int16_t *range);
void  pv_rangeMeter_process_frame(void);

static uint8_t USxTICK;
static bool midiendo;
uint32_t await_time_ms;
frameData_t data_record;

#define TIMER1_START ( TCCR1B |= ( 1 << CS11 ))	// Prescaler x8: cuento de a uS
#define TIMER1_STOP ( TCCR1B &= ~( 1 << CS11 ))

//--------------------------------------------------------------------------------------
 void tkRange(void * pvParameters)
{

( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("starting tkRange..\r\n\0"));
	FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );

	pv_rangeMeter_init();
	//
	vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
	//
	for( ;; )
	{

	    // Mido
	    pv_rangeMeter_ping(&data_record.range);

	    // Proceso el frame
	    pv_rangeMeter_process_frame();

		// Espero
		await_time_ms = ( systemVars.timerPoll - 5 ) * 1000;
	    vTaskDelay( ( TickType_t)( await_time_ms / portTICK_RATE_MS ) );

	}

}
//------------------------------------------------------------------------------------
void pv_rangeMeter_init(void)
{
	// Apagamos la medicion de pulsos.
	pv_rangeMeter_stop();

	// Calculo el prescaler del timer.
	USxTICK = 1;

	// Configuro el TIMER1 como base de tiempos
	TCCR1A = 0x00;

	// No uso prescaler por lo que cuento en uS.

	// Configuro la interrupcion 2 para que detecte ambos flancos
	// 1: Deshabilto la interrupcion
	EIMSK = 0x00;
	// 2: Configuro como detecto la interripcion:
	EICRA |= 1<<ISC20;   // Trigger INT2 on any edge
	// 3: Borro la flag
	EIFR = 0x00;
	// 4: Habilito la interrupcion
	//EIMSK = 1 << INT2;


}
//------------------------------------------------------------------------------------
void pv_rangeMeter_ping(int16_t *range)
{
	// Mido durante 5 segundos y luego promedio las medidas.

	// Prendo el sensor
	pv_rangeMeter_start();

	// Inicializo
	midiendo = false;
	pv_flush_stack_pulses();

	// Habilto la interrupcion externa conectada al pin PB2 (INT2)
	EIMSK = 1 << INT2;
	//
	// Espero
	vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
	//
	// Desbilto la interrupcion
	EIMSK = 0x00;
	//
	// Apago el sensor
	pv_rangeMeter_stop();
	//
	// Calculo valores y muestro resultados
	*range = pv_pulse_calcular_distancia();

	return;

}
//------------------------------------------------------------------------------------
void pv_rangeMeter_start(void)
{
	// Activo el sensor habilitando la señal UPULSE_RUN lo que hace
	// que el sensor comienze a medir con una frecuencia de 6hz.
	// En el pin 2 del sensor debo poner un 1 ( aunque tiene un pullup). Como paso
	// por un inversor, el micro debe poner un 0.

	IO_clr_UPULSE_RUN();

}
//------------------------------------------------------------------------------------
void pv_rangeMeter_stop(void)
{

	// El pin de RUN es el pin4 del sensor MAX-XL alimentado por medio de un inversor
	// En reposo el pin del sensor debe estar en 0 por lo tanto el micro pone un 1
	// antes del inversor.

	IO_set_UPULSE_RUN();

}
//------------------------------------------------------------------------------------
void pv_flush_stack_pulses(void)
{

	// Inicicalizo el stack de datos

uint8_t i;

	for (i=0; i < MAX_PULSE_STACK; i++) {
		s_pulse_stack.stack[i] = -1;
	}
	s_pulse_stack.ptr = 0;
}
//------------------------------------------------------------------------------------
void pv_push_stack_pulses(uint16_t counter)
{

	if( s_pulse_stack.ptr < MAX_PULSE_STACK ) {
		s_pulse_stack.stack[s_pulse_stack.ptr++] = counter;
	}

}
//------------------------------------------------------------------------------------
int16_t pv_pulse_calcular_distancia(void)
{

uint16_t avg;
float var;
float us;
uint16_t distancia;
int16_t ping;

	pv_pulsos_stats(&avg, &var);
	us = USxTICK * avg;						// Convierto a us.
	distancia = (uint16_t)( us / 58);		// Calculo la distancia ( 58us - 1cms )
	if ( (distancia > 0) && (distancia < 600) ) {
		ping = distancia;
	} else {
		ping = -1;
	}

	if (systemVars.debugLevel ==  D_RANGE ) {
		snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("pulse DEBUG: avg=%d, var=%.03f, us=%.1f, distancia=%d \r\n\0"),avg, var, us, distancia);
		FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );
	}

	return(ping);

}
//------------------------------------------------------------------------------------
void pv_pulsos_stats(uint16_t *avg, float *var)
{
	// Calculo el promedio de los datos del stack si sin validos.

uint8_t i, items;
float prom, std;

	prom = 0.0;
	std = 0.0;
	items = 0;

	for ( i = 0; i < MAX_PULSE_STACK; i++ ) {
		if ( s_pulse_stack.stack[i] > 0) {
			prom += s_pulse_stack.stack[i];
			std += ( s_pulse_stack.stack[i] * s_pulse_stack.stack[i] );
			items += 1;
		}

		if (systemVars.debugLevel ==  D_RANGE ) {
			snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("pulse DEBUG: [%02d][%02d] %.01f %.01f %.01f\r\n\0"), i,items,(float)s_pulse_stack.stack[i], prom, std );
			FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );
		}

	}
	// Promedio
	prom /= items;
	// Desviacion estandard
	std = sqrt (std / items - ( prom * prom ));

	*avg = (uint16_t) prom;
	*var = std;

}
//------------------------------------------------------------------------------------
void  pv_rangeMeter_process_frame(void)
{

size_t bytes_written;
StatBuffer_t pxFFStatBuffer;
uint16_t pos = 0;

	// Agrego el timeStamp
	RTC_read(&data_record.rtc);

    // Guardo en BD
	bytes_written = FF_fwrite( &data_record, sizeof(data_record));
	FF_stat(&pxFFStatBuffer);

	if ( bytes_written != sizeof(data_record) ) {
		// Error de escritura ??
		pos = snprintf_P( range_printfBuff,sizeof(range_printfBuff),PSTR("%s BD: WR ERROR: (%d),\0"),u_now(),pxFFStatBuffer.errno);
	} else {
		// Stats de memoria
		pos = snprintf_P( range_printfBuff, sizeof(range_printfBuff), PSTR("%s MEM [%d/%d/%d][%d/%d],\0"),u_now(), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}

    // Imprimo
	// HEADER
	pos += snprintf_P( &range_printfBuff[pos], (sizeof(range_printfBuff) - pos ), PSTR(" frame {" ) );
	// timeStamp.
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),data_record.rtc.year,data_record.rtc.month,data_record.rtc.day );
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("%02d%02d%02d"),data_record.rtc.hour,data_record.rtc.min, data_record.rtc.sec );
	// Range.
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR(",H=%d"), data_record.range );
	// TAIL
	pos += snprintf_P( &range_printfBuff[pos], ( sizeof(range_printfBuff) - pos ), PSTR("}\r\n\0") );
	FreeRTOS_write( &pdUART1, range_printfBuff, sizeof(range_printfBuff) );

    // Aviso que trasmita
	// tkGPRS
	while ( xTaskNotify(xHandle_tkGprsRx, TK_FRAME_READY , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

}
//------------------------------------------------------------------------------------
// ISR
// El ANCHO DEL PULSO SE MIDE EN EL PIN UPULSE_WIDTH.
// Tenemos una interrupcion de flanco.
//
ISR( INT2_vect )
{
	// Detectamos cual flanco disparo la interrupcion.
	// Como pasamos el pulso por un inversor los flancos quedan cambiados.
	// Lo que hacemos es tomar como referencias los pulsos medidos en el micro y
	// no los generados en el sensor.


	if ( IO_read_UPULSE_WIDTH() == 0 ) {
		// Flanco de bajada: Arranca el pulso. Arranco el timer.
		TCNT1 = 0x00;
		TIMER1_START;
		midiendo = true;
//		IO_clear_TEST_PIN();

	} else {
		// Flanco de subida: Termino el pulso. Paro el timer
		TIMER1_STOP;	// Apago el timer.
		if (midiendo) {
			midiendo = false;
			pv_push_stack_pulses(TCNT1);
		}
//		IO_set_TEST_PIN();
	}

	// Borro la interrupcion escribiendo un 1.
	EIFR = 0x04;

}
//------------------------------------------------------------------------------------
