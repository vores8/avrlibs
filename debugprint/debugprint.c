#include "debugprint.h"

#ifdef DEBUG_PRINT

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uart.h>


void debugPrint(const char *format, ...) {
	char buff[128];
	va_list args;
	va_start(args, format);
	vsnprintf(buff, sizeof(buff), format, args);
	va_end(args);
	buff[sizeof(buff) / sizeof(buff[0]) - 1] = '\0';
	uart_puts(buff);
}
#else
void debugPrint(const char *format, ...) {
}
#endif
