/*
 * board.h
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */

#ifndef INC_BOARD_H_
#define INC_BOARD_H_

#include "stm32f4xx_hal.h"

/* Export HAL handles */
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

/* Board init / MX functions (implemented in board.c) */
void Board_Init(void);
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_USART2_UART_Init(void);
void MX_SPI1_Init(void);

/* Called before starting the RTOS (start HW that must run IRQs) */
void Board_InitBeforeOs(void);

#endif /* INC_BOARD_H_ */
