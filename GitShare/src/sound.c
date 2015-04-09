/*
 * sound.c
 *
 *Designed to play wav files
 *  Created on: 10 Sep 2013
 *      Author: Jesse
 */
#include "sound.h"

uint16_t soundBufferSizeVar=SOUNDBUFFERSIZE;

uint8_t soundBuffer[SOUNDBUFFERSIZE];
uint16_t soundBufferIndex=99;
uint8_t soundBufferUpdate=0;
uint16_t soundVolume=512;

//return 1 if buffer needs to be filled, returns 2 if second half of buffer needs to be filled, returns 0 if buffer is fine
void updateSoundPWM()
{
	//soundVolume=127;
	if(soundVolume<128)
	{
		soundVolume=128;
	}
	if(soundVolume>600)
	{
	//	soundVolume=1024;
	}
	else
	{
//		soundVolume=soundVolume;
	}

	TIM1->CCR3=(soundBuffer[soundBufferIndex]*soundVolume)/1024;

	if(++soundBufferIndex>=SOUNDBUFFERSIZE)
		{
		soundBufferIndex=0;
		soundBufferUpdate=2;
		}

}


