/*
 * myMaths.c
 *
 *  Created on: 31 Jul 2012
 *      Author: Unparagoned
 */

//this rounds the number to the nearest unit
//eg. 2860 to 100, rounds to  2900.

#include "myMaths.h"
uint16_t round(uint16_t number, uint16_t units)
{
	uint16_t rounded=units*(number/units);
	rounded+=(number%units)>(units/2)?units:0;
return rounded;

}
