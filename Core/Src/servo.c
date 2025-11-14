/*
 * servo.c
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */


#include "servo.h"
#include "board.h"
#include "uart_debug.h"
#include "sensors.h"
#include "cmsis_os.h"

void servo_init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    UART_Debug(" -> Servo timer started...\r\n");
}

void servo_set_angle(uint8_t angle)
{
    uint32_t pulse_length = 1000 + ((angle * 1000) / 180);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_length);
}

/* Task reading ADC and moving servo (was StartServoControlTas) */
void servo_task(void *argument)
{
    UART_Debug("Servo Control Task Started...\r\n");
    for (;;)
    {
        uint32_t adc_value = 0;
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
            adc_value = HAL_ADC_GetValue(&hadc1);
            //UART_Debug("ADC Value: %u\r\n", adc_value);
        }
        uint8_t angle_servo = (uint8_t)map_int32((int32_t)adc_value, 0, 4095, 0, 180);
        servo_set_angle(angle_servo);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

