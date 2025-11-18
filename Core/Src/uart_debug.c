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
    char buffer[128]; 					            // Buffer local pour stocker le message formaté
    va_list args;	  					            // Déclare une liste pour les arguments variables
    va_start(args, fmt); 				            // Initialise la liste d'arguments à partir de fmt
    vsnprintf(buffer, sizeof(buffer), fmt, args);   // Formate la chaîne avec les arguments et l'écrit dans buffer
    va_end(args); 									// Termine l'utilisation des arguments variables

    if (UartMutex != NULL) {                        // Vérifie si le mutex UART a été créé
        if (xSemaphoreTake(UartMutex, pdMS_TO_TICKS(50)) == pdTRUE) { // Tente de prendre le mutex pendant max 50 ms
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY); // Envoie le message sur UART
            xSemaphoreGive(UartMutex);              // Libere le mutex pour les autres tâches
        }
    } else { 										// Si le mutex n’existe pas encore (début du boot)
        /* Fallback if mutex not created yet (early boot) */
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}
