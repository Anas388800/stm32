/*
 * uart_debug.h
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */

#ifndef INC_UART_DEBUG_H_
#define INC_UART_DEBUG_H_

#include <stdarg.h>
#include <stdint.h>

void UART_Debug_Init(void);
void UART_Debug(const char *fmt, ...); // thread-safe wrapper printf-like

#endif /* INC_UART_DEBUG_H_ */
