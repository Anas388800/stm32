/*
 * servo.h
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>

void servo_init(void);
void servo_task(void *argument);
void servo_set_angle(uint8_t angle);

#endif /* INC_SERVO_H_ */
