/*
 * pwm.c
 *
 *  Created on: Jan 29, 2022
 *      Author: Andre
 */
#include "tim.h"

void PWM_setPulse(uint16_t per_mille)
{
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, per_mille);
}
