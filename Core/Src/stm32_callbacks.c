/*
 * stm32_callbacks.c
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */


#include "stm32f4xx_hal.h"
#include "ultrasonic.h"
#include "spi_if.h"
#include "board.h"
#include "os_ressource.h"

/* TIM input capture */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    ultrasonic_capture_cb(htim);
}

/* SPI Rx complete (DMA or IT) */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
    	spi_if_dma_rx_complete(hspi);

    }
}
