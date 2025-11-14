/*
 * app_init.c
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */


#include "app_init.h"
#include "board.h"
#include "os_ressource.h"
#include "uart_debug.h"
#include "ultrasonic.h"
#include "servo.h"
#include "buzzer.h"
#include "spi_if.h"
#include "cmsis_os.h"
#include "main.h"


/* Thread attributes (copiés / adaptés) */
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
const osThreadAttr_t ServoControlTas_attributes = {
  .name = "ServoControlTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
const osThreadAttr_t SPITask_attributes = {
  .name = "SPITask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t defaultTaskHandle;
osThreadId_t UltrasonicTaskHandle;
osThreadId_t ServoControlTasHandle;
osThreadId_t SPITaskHandle;

static void DefaultTask(void *argument) { for(;;) { osDelay(1); } }

void App_Init(void)
{
    /* create queues/mutex etc */
    OS_Resources_Create();

    /* init modules */
    UART_Debug_Init();
    UART_Debug("App_Init: starting modules...\r\n");

    ultrasonic_init();
    servo_init();
    buzzer_init();
    spi_if_init();

    /* Any board-level init that must be done before starting tasks */
    Board_InitBeforeOs();

    /* Create tasks */
    defaultTaskHandle = osThreadNew(DefaultTask, NULL, &defaultTask_attributes);
    UltrasonicTaskHandle = osThreadNew(ultrasonic_task, NULL, &UltrasonicTask_attributes);
    ServoControlTasHandle = osThreadNew(servo_task, NULL, &ServoControlTas_attributes);
    SPITaskHandle = osThreadNew(spi_task, NULL, &SPITask_attributes);
}
