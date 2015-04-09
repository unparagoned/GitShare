/*
 * menu.h
 *
 *  Created on: May 26, 2012
 *      Author: Unparagoned
 */

#ifndef MENU_H_
#define MENU_H_

#include "stm32f10x.h"
#include "fonts.h"
#include "myHeaders.h"
#include "sound.h"

//Menu type gives the first digit, the subdigits
#define ntopMenu 0
#define nPWMMenu 11
#define nPWMSubMenu 21
#define nPWMActionMenu 91


#define nAnalogMenu 10
#define nAnalogSubMenu 20
#define nAnalog1ActionMenu 30
#define nAnalog2ActionMenu 40
#define nAnalogActionMenu 90



#define nGPIOMenu 3000

#define nReadingsMenu 13

#define nPWM 1
#define nAnalog 2
#define nGPIO 3
#define nReadings 4
#define nVoltage 5

#define nPortA 10
#define nPortB 20
#define nPortC 30
#define nPortD 40
#define nPortE 50
#define nPortF 60

#define nTimer1 10
#define nTimer2 20
#define nTimer8 80

#define nChargeH 10
#define nChargeL 20
#define nDischargeH 30
#define nDischargeL 40


#define nPeriod 100
#define nDuty 200

#define nLowPins 0
#define nHighPins 0

//where are these numbers from?
#define nButtonPress 32768
#define nSliderTouch 10000

#define nAnalogPC0 10
#define nAnalogPC1 20
//options structure

/*
 * name of option,
 * function of option
 * option pointer
 * selection for history
 */
typedef struct
{
	uint8_t *name;
	void(*function)(void *selection);
	void *optionPoint;
	uint16_t selection;

} option;


//menu structure
typedef struct
{
	//menu name 15 max char
	uint8_t *name;
	uint8_t numberOfOptions;
	option options[8];
	uint16_t selectionHistory; //this kind of keeps track of which pins/timer have been selected.
	uint8_t (*optionSelected)(uint16_t xPos, uint16_t yPos);
	uint8_t slider;

} menu;



void menuInit();
uint8_t menuInterface();
void menuOption();
void startMenu();
uint8_t getRelTouch(uint16_t * xPos, uint16_t *yPos);
#endif /* MENU_H_ */
