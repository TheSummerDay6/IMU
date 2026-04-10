#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h" 
#include "stdio.h"
#include "string.h"
#include <stdarg.h>

extern uint8_t UARTBUF[256];
extern uint8_t len;

void uart_printf(const char *format, ...);
void uart_init(void);
void uart_receive_show(void);
void uart_func(const char* targStr,  uint8_t* flag);

#endif
