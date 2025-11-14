/*
 * sensors.c
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */

#include "sensors.h"

int32_t map_int32(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

