/*
 * buzzer.c
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */


#include "buzzer.h"
#include "board.h"
#include "uart_debug.h"

/* Very simple buzzer API: set small pulses using TIM3 */
void buzzer_init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    UART_Debug(" -> buzzer timer started...\r\n");
}

/* If delay_ms==0 -> stop buzzer; else play quick toggles (blocking short) */
void buzzer_set_tone(uint16_t delay_ms)
{
    if (delay_ms == 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        return;
    }
    __HAL_TIM_SET_PRESCALER(&htim3, 83);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 200);
    HAL_Delay(delay_ms);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_Delay(delay_ms);
}

