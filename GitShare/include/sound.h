/*
 * sound.h
 *
 *To play wav files
 *  Created on: 10 Sep 2013
 *      Author: Jesse
 */

#ifndef SOUND_H_
#define SOUND_H_

#include "stm32f10x.h"


#define SOUNDBUFFERSIZE 12288


extern uint8_t soundBuffer[SOUNDBUFFERSIZE];
extern uint16_t soundBufferSizeVar;
extern uint8_t soundBufferUpdate;
extern uint16_t soundBufferIndex;
extern uint16_t soundVolume;

void updateSoundPWM();


#endif /* SOUND_H_ */
