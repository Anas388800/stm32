/*
 * os_ressource.c
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */


#include "os_ressource.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "uart_debug.h"

QueueHandle_t DistanceQueue = NULL;
QueueHandle_t SPIQueue = NULL;
SemaphoreHandle_t UartMutex = NULL;

void OS_Resources_Create(void)
{
    DistanceQueue = xQueueCreate(5, sizeof(uint8_t));
    if (DistanceQueue == NULL) UART_Debug("ERROR: DistanceQueue not created\r\n");

    SPIQueue = xQueueCreate(5, 64); // 64 bytes per message
    if (SPIQueue == NULL) UART_Debug("ERROR: SPIQueue not created\r\n");

    UartMutex = xSemaphoreCreateMutex();
    if (UartMutex == NULL) UART_Debug("ERROR: UartMutex not created\r\n");

    if (DistanceQueue!= NULL && SPIQueue != NULL && UartMutex != NULL)
    	UART_Debug("OS ressource created\r\n");
}

void OS_Resources_Delete(void)
{
    if (DistanceQueue) vQueueDelete(DistanceQueue);
    if (SPIQueue) vQueueDelete(SPIQueue);
    if (UartMutex) vSemaphoreDelete(UartMutex);
}

