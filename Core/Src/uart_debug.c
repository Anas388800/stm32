/*
 * uart_debug.c
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */


#include "uart_debug.h"
#include "os_ressource.h"
#include "board.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Simple thread-safe printf over UART using mutex */
void UART_Debug_Init(void) { /* nothing for now */ }

void UART_Debug(const char *fmt, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (UartMutex != NULL) {
        if (xSemaphoreTake(UartMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            xSemaphoreGive(UartMutex);
        }
    } else {
        /* Fallback if mutex not created yet (early boot) */
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}
