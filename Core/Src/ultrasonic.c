/*
 * ultrasonic.c
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */


#include "ultrasonic.h"
#include "board.h"
#include "os_ressource.h"
#include "uart_debug.h"
#include "sensors.h"
#include "buzzer.h"
#include "cmsis_os.h"
#include "main.h"


static uint32_t IC_Val1 = 0;
static uint32_t IC_Val2 = 0;
static uint32_t Difference = 0;
static uint8_t Is_First_Captured = 0;
static uint8_t Distance = 0;

void ultrasonic_init(void)
{
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    UART_Debug(" -> ULTRASONIC timer started...\r\n");
}

void ultrasonic_trigger(void)
{
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

/* API called from HAL callback */
void ultrasonic_capture_cb(TIM_HandleTypeDef *htim)
{
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;

    if (!Is_First_Captured) {
        IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        Is_First_Captured = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {
        IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        __HAL_TIM_SET_COUNTER(htim, 0);
        if (IC_Val2 > IC_Val1) Difference = IC_Val2 - IC_Val1;
        else Difference = (0xffff - IC_Val1) + IC_Val2;
        Distance = (uint8_t)(Difference * 0.034 / 2);
        Is_First_Captured = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (DistanceQueue) xQueueSendFromISR(DistanceQueue, &Distance, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Task implementing ultrasonic behavior (was StartUltrasonicTask) */
void ultrasonic_task(void *argument)
{
    uint8_t distance = 0;
    uint8_t dummy = 0;
    UART_Debug("Ultrasonic Task Started...\r\n");

    for (;;)
    {
        /* vider la queue pour avoir la mesure propre */
        while (DistanceQueue && xQueueReceive(DistanceQueue, &dummy, 0) == pdPASS) { ; }

        ultrasonic_trigger();

        if (DistanceQueue && xQueueReceive(DistanceQueue, &distance, pdMS_TO_TICKS(100)) == pdPASS)
        {
            if (distance < 2 || distance > 80) { vTaskDelay(pdMS_TO_TICKS(60)); continue; }

            char msg[64];
            sprintf(msg, "Distance: %d cm\r\n", distance);
            UART_Debug(msg);

            if (distance < 40) {
                uint16_t buz = (uint16_t)map_int32(distance, 5, 40, 100, 900);
                buzzer_set_tone(buz);
            } else {
                buzzer_set_tone(0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(60));
    }
}
