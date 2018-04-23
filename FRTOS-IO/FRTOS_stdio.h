/*
 * FRTOS_stdio.h
 *
 *  Created on: 23 de abr. de 2018
 *      Author: pablo
 */

#ifndef FRTOS_IO_FRTOS_STDIO_H_
#define FRTOS_IO_FRTOS_STDIO_H_

#include <stdarg.h>

int FRTOS_printf(const char *format, ...);
int FRTOS_sprintf(char *out, const char *format, ...);
int FRTOS_snprintf( char *buf, unsigned int count, const char *format, ... );

int mini_vsnprintf(char* buffer, unsigned int buffer_len, const char *fmt, va_list va);
int mini_snprintf(char* buffer, unsigned int buffer_len, const char *fmt, ...);


#endif /* FRTOS_IO_FRTOS_STDIO_H_ */
