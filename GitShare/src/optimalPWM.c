/*
 * optimalPWM.c
 *This determines the period and duty cycle for optimal voltage output
 *  Created on: Jun 4, 2012
 *      Author: Unparagoned
 */

#include "optimalPWM.h"

extern __IO uint16_t ADCConvertedValue;
extern __IO uint16_t ADCConvertedValuePC1;
extern __IO uint32_t TimingDelay;

typedef struct{

	uint16_t period;
	uint16_t duty;
	uint16_t voltage;

}dutyVoltage;

uint16_t getVoltage();


//sets the pulse and duty to the optimal value
void optimumDuty(uint16_t *period, uint16_t *duty)
{


	uint16_t periodCounter;
	dutyVoltage optimumDuty[3];
	uint16_t periodRange[2];
	uint16_t dutyRange[2];
	uint16_t maxVoltage=0;
	uint16_t currentPeriod=TIM1->ARR;

	//temp doesn't go through periods, just one period for now
for(periodCounter=2;periodCounter<65535; periodCounter++)
{
	periodCounter=currentPeriod; //temp loop off

	periodRange[0]=periodRange[1]=periodCounter;
	dutyRange[0]=1;
	dutyRange[1]=periodRange[1]-1;
	optimumDutyRange(&optimumDuty,periodRange,dutyRange);

	dutyRange[0]=optimumDuty[0].duty;
	dutyRange[1]=optimumDuty[2].duty;

	optimumDutyRange(&optimumDuty,periodRange,dutyRange);

	if(optimumDuty[1].voltage>maxVoltage)
	{
		maxVoltage=optimumDuty[1].voltage;
		*period=optimumDuty[1].period;
		*duty=optimumDuty[1].duty;
	}

break;//temp loop off
}


}


/*
 * finds the optimum duty and period
 * current implementation has the period fixed.
 * Go through whole range and then find optimal, this is because of the error in readings...maybe not
 */
void optimumDutyRange(dutyVoltage (*optimumTimings)[3], uint16_t periodRange[2], uint16_t dutyRange[2])
{

	uint16_t periodCounter, dutyCounter=0;
	//four percent, use power of twos to make things nicer
	uint16_t percentDetail=4;
	uint16_t periodPercent=(periodRange[1]-periodRange[0])*percentDetail/100;
		//make sure it doesn't become too small
	uint16_t measuredVoltage;
	uint16_t previousVoltage;
	periodPercent=(periodPercent>0?periodPercent:1);


	uint16_t dutyPercent=(dutyRange[1]-dutyRange[0])*percentDetail/100;
		//make sure it doesn't become too small
	dutyPercent=(dutyPercent>0?dutyPercent:1);
	dutyVoltage currentValue;

	uint16_t maxVoltage;





	for(periodCounter=periodRange[0]; periodCounter<=periodRange[1];periodCounter+=periodPercent)
	{
		currentValue.period=periodCounter;
		previousVoltage=0;

		for(dutyCounter=0; dutyCounter<=dutyRange[1];dutyCounter+=dutyPercent)
		{
			currentValue.duty=dutyCounter;
			//set duty to dutyCounter
			TIM1->CCR3=dutyCounter;
			//delay 1ms
			sysDelay(1);
			while(TimingDelay);
			//measure once since voltage will vary and don't want that
			//measuredVoltage=ADCConvertedValue;
			measuredVoltage=getVoltage();
			//read voltage -increasing
			currentValue.voltage=measuredVoltage;

			if(measuredVoltage/25>=previousVoltage/25)
			{
				*optimumTimings[0]=*optimumTimings[1];
				*optimumTimings[1]=currentValue;
			}
			//decreasing
			if(measuredVoltage/25<previousVoltage/25)
			{
				*optimumTimings[2]=currentValue;
				break;
			}
			previousVoltage=measuredVoltage;


		}

	}



}

uint16_t getVoltage()
{
	uint16_t voltageRecord[100];
		uint8_t tempCounter;
		uint16_t tempMax=0;
		uint16_t tempMin=65535;
		uint16_t readADC=ADCConvertedValue;
		uint32_t averageVoltage=0;

		for(tempCounter=0;tempCounter<100;tempCounter++)
		{
			readADC=ADCConvertedValue;
			if(readADC>tempMax)
				{
				 tempMax=readADC;
				}
			if(readADC<tempMin)
				{
				 tempMin=readADC;
				}
			averageVoltage+=readADC;

			voltageRecord[tempCounter]=readADC;
		}

		uint16_t unfilteredAverage=averageVoltage/100;
		uint16_t fivePercent=unfilteredAverage/20;
		uint16_t avMax=unfilteredAverage+fivePercent;
		uint16_t avMin=unfilteredAverage-fivePercent;
		uint32_t newAverage=0;
		uint8_t validValues=0;
		for(tempCounter=0;tempCounter<100;tempCounter++)
		{
			if(voltageRecord[tempCounter]<avMax && voltageRecord[tempCounter]>avMin)
			{
				validValues++;
				newAverage+=voltageRecord[tempCounter];

			}
		}

	averageVoltage/=100;
	newAverage/=validValues;

	return newAverage;
}
