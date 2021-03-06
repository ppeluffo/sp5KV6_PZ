/*
 * sp5K.h
 *
 * Created on: 27/12/2013
 *      Author: root
 */

#ifndef SP5K_H_
#define SP5K_H_

#include <avr/io.h>			/* include I/O definitions (port names, pin names, etc) */
//#include <avr/signal.h>		/* include "signal" names (interrupt names) */
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
//#include <stdio.h>
#include <avr/sleep.h>
#include <string.h>
#include <compat/deprecated.h>
#include <util/twi.h>
#include <util/delay.h>
#include <ctype.h>
#include <util/delay.h>
#include <avr/cpufunc.h>

#include <l_file.h>
#include <l_rtc.h>
#include <l_iopines.h>
#include <l_mcp.h>
#include <FRTOS_stdio.h>

#include "sp5Klibs/avrlibdefs.h"
#include "sp5Klibs/avrlibtypes.h"
#include "sp5Klibs/global.h"			// include our global settings
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "FRTOS-IO.h"

#include "cmdline.h"

// DEFINICION DEL TIPO DE SISTEMA
//----------------------------------------------------------------------------
#define SP5K_REV "5.0.2"
#define SP5K_DATE "@ 20180423"

#define SP5K_MODELO "sp5KV6_PZ HW:avr1284P R5.0"
#define SP5K_VERSION "FW:FRTOS8"

#define CHAR64		64
#define CHAR128	 	128
#define CHAR256	 	256

//----------------------------------------------------------------------------
// TASKS
/* Stack de las tareas */
#define tkCmd_STACK_SIZE		1024
#define tkControl_STACK_SIZE	1024
#define tkGprs_STACK_SIZE		1024
#define tkGprsRx_STACK_SIZE		1024
#define tkRange_STACK_SIZE		1024

/* Prioridades de las tareas */
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkControl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkGprs_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprsRx_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkRange_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )

/* Prototipos de tareas */
void tkCmd(void * pvParameters);
void tkControl(void * pvParameters);
void tkGprsTx(void * pvParameters);
void tkGprsRx(void * pvParameters);
void tkRange(void * pvParameters);

TaskHandle_t xHandle_tkCmd, xHandle_tkControl, xHandle_tkGprs, xHandle_tkGprsRx, xHandle_tkRange;

bool startTask;
typedef struct {
	uint8_t resetCause;
	uint8_t mcusr;
} wdgStatus_t;

wdgStatus_t wdgStatus;

// Mensajes entre tareas
#define TK_READ_FRAME			0x02	// to tkAnalogIN: (mode service) read a frame
#define TK_FRAME_READY			0x08	//
#define TK_REDIAL				0x10	//

//------------------------------------------------------------------------------------

xSemaphoreHandle sem_SYSVars;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

typedef enum { D_NONE = 0,D_GPRS, D_RANGE, D_DEBUG  } t_debug;

#define NRO_CHANNELS		3

#define DLGID_LENGTH		12
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define IP_LENGTH			24
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15
#define PARAMNAME_LENGTH	5

#define TIMERDIAL_FOR_CONTINUO	600

typedef struct {
	RtcTimeType_t rtc;
	int16_t range;
	uint8_t dig0;
	uint8_t dig1;
} frameData_t;

typedef struct {
	// Variables de trabajo.
	// Tamanio: 302 bytes para 3 canales.

	uint8_t dummyBytes;
	uint8_t initByte;

	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char server_tcp_port[PORT_LENGTH];
	char server_ip_address[IP_LENGTH];
	char dlg_ip_address[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char passwd[PASSWD_LENGTH];

	uint8_t csq;
	uint8_t dbm;

	uint16_t timerPoll;

	uint8_t debugLevel;		// Indica que funciones debugear.
	uint8_t gsmBand;

	uint16_t maxRange;

	char chName[NRO_CHANNELS][PARAMNAME_LENGTH];

	bool roaming;

} systemVarsType;

systemVarsType systemVars,tmpSV;

uint32_t ticks;

#define EEADDR_SV 32		// Direccion inicio de la EE de escritura del systemVars.

//------------------------------------------------------------------------------------
// FUNCIONES DE USO GENERAL.
//------------------------------------------------------------------------------------
// utils
void u_uarts_ctl(uint8_t cmd);
bool u_configTimerPoll(char *s_tPoll);
bool u_configMaxRange(char *s_maxRange);
bool u_configChName( uint8_t channel, char *chName );
void u_kick_Wdg( uint8_t wdgId );
bool u_saveSystemParams(void);
bool u_loadSystemParams(void);
void u_loadDefaults(void);
char *u_now(void);
void u_debugPrint(uint8_t debugCode, char *msg, uint16_t size);
void u_reset(void);

// tkGprs
bool u_modem_prendido(void);

char nowStr[32];
char debug_printfBuff[CHAR128];

//------------------------------------------------------------------------------------
// WATCHDOG
uint8_t systemWdg;

#define WDG_CTL			0x01
#define WDG_CMD			0x02
//#define WDG_GPRS		0x20
//#define WDG_GPRSRX	0x40

//------------------------------------------------------------------------------------

#endif /* SP5K_H_ */
