/*
 * voltageControl.h
 *
 *  Created on: 31 Jul 2012
 *      Author: Unparagoned
 */

#ifndef VOLTAGECONTROL_H_
#define VOLTAGECONTROL_H_
#include "stm32f10x.h"
#include "myMaths.h"
//divide voltage by VOLTAGEDIVIDER TO GET reading voltage
#define VOLTAGEDIVIDER 51


//value read =voltagemultiplier*voltage, voltage multiplier =24.34=2^12/(3.3*51)
#define VOLTAGEMULTIPLIER 24


uint16_t getDutyCycle(TIM_TypeDef * TIMx);
void reduceOutput(uint16_t voltage);
void setDutyCycle(TIM_TypeDef * TIMx, uint16_t duty);

void setOutputVoltage();
#endif /* VOLTAGECONTROL_H_ */
