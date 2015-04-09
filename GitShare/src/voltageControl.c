/*
 * voltageControl.c
 *
 *  Created on: 31 Jul 2012
 *      Author: Unparagoned
 *
 *      Controls boost voltage.
 *
 *      Would also control output voltage.
 *
 */



#include "voltageControl.h"

extern __IO uint16_t ADCConvertedValuePC1;
uint8_t boostVoltage=50;

uint8_t outputStatus=0;
//values used to set the voltages, this is for two changing voltages, to work in pairs.
uint16_t chargeHVal=150;
uint16_t chargeLVal=150;
uint16_t dischargeHVal=10000;
uint16_t dischargeLVal=9500;

uint16_t delay;
void voltageInit(uint8_t voltage)
{
	boostVoltage=voltage;
}

//needs bounds may try to increase past max.
void updateBoostDuty()
{
	uint16_t currentVoltage=round(ADCConvertedValuePC1,100);



	//changes duty cycle depending on difference from optimum. If value is too low it increases duty and too high decreases duty.
	int16_t difference=(currentVoltage-round(boostVoltage*VOLTAGEMULTIPLIER,100))>>3;
	uint16_t currentDuty=getDutyCycle(TIM1);
	difference=difference>currentDuty?(currentDuty*9)/10:difference;
	currentDuty=currentDuty>1?currentDuty:1;
	setDutyCycle(TIM1, currentDuty-difference);

}

//returns duty as a range from 0-1024
uint16_t getDutyCycle(TIM_TypeDef * TIMx)
{
	return (((TIMx->CCR3)<<10)/TIMx->ARR);
}

//from 0 to 1024
//max bound check here
void setDutyCycle(TIM_TypeDef * TIMx, uint16_t duty)
{

	duty=(duty>1024)?900:duty;



			//should set duty as a percentage of period.
			TIMx->CCR3=(TIMx->ARR*duty)>>10;
}



//turns on discharge for a period to discharge output cap to required level
/*
 * Ideally would read or know voltage, and then discharge for required time.
 */
void reduceOutput(uint16_t voltage)
{



//sets buck off, so there is no direct drain to ground.
	uint16_t tempDuty=TIM8->CCR3;
	TIM8->CCR3=0;


	//delay is cause delay in circuit and stop pass through from boost cap
	Delay(5000);
	GPIO_ResetBits(GPIOB,  GPIO_Pin_10);
	Delay(5000);
	GPIO_SetBits(GPIOB, GPIO_Pin_10);

	//delay is cause delay in circuit and stop pass through from boost cap
	Delay(400);
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_Init(GPIOB, &GPIO_InitStructure);
	//TIM_CtrlPWMOutputs(TIM8, ENABLE);
	TIM8->CCR3=tempDuty;

}
void setOutputVoltage()
{
	TIM_SetCounter(TIM8,0);
	TIM_SetCounter(TIM2,0);

	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);

	if(outputStatus)
	{
		TIM8->CCR3=chargeHVal;
		TIM2->CCR3=dischargeHVal;

	}
	else
	{
		TIM8->CCR3=chargeLVal;
		TIM2->CCR3=dischargeLVal;
	}

	outputStatus=outputStatus?0:1;
}
