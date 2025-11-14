/*
 * os_ressource.h
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */

#ifndef INC_OS_RESSOURCE_H_
#define INC_OS_RESSOURCE_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Queues / Mutexes exportés */
extern QueueHandle_t DistanceQueue;
extern QueueHandle_t SPIQueue;
extern SemaphoreHandle_t UartMutex;

/* Création / destruction ressources OS */
void OS_Resources_Create(void);
void OS_Resources_Delete(void);

#endif /* INC_OS_RESSOURCE_H_ */
