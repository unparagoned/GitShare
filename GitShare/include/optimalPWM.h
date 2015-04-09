/*
 * optimalPWM.h
 *
 *  Created on: Jun 5, 2012
 *      Author: Unparagoned
 */

#ifndef OPTIMALPWM_H_
#define OPTIMALPWM_H_

#include "stm32f10x.h"

void optimumDuty(uint16_t *period, uint16_t *duty);

#endif /* OPTIMALPWM_H_ */
