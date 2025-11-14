/*
 * ultrasonic.h
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "stm32f4xx_hal.h"

void ultrasonic_init(void);
void ultrasonic_trigger(void);
void ultrasonic_task(void *argument);
void ultrasonic_capture_cb(TIM_HandleTypeDef *htim); // appelé par callback HAL

#endif /* INC_ULTRASONIC_H_ */
