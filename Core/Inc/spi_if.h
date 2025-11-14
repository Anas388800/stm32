/*
 * spi_if.h
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */

#ifndef INC_SPI_IF_H_
#define INC_SPI_IF_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

void spi_if_init(void);
void spi_if_dma_rx_complete(SPI_HandleTypeDef *hspi);
void spi_task(void *argument);

#endif /* INC_SPI_IF_H_ */
