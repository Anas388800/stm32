/*
 * sensors.h
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include <stdint.h>

/* Mapping générique (signed) */
int32_t map_int32(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

#endif /* INC_SENSORS_H_ */
