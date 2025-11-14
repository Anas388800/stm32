/*
 * buzzer.h
 *
 *  Created on: 6 nov. 2025
 *      Author: a888451
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include <stdint.h>

void buzzer_init(void);
void buzzer_set_tone(uint16_t delay_ms); // small API to play brief tone (0 -> off)

#endif /* INC_BUZZER_H_ */
