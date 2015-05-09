/*
 * main.h
 *
 *  Created on: May 14, 2011
 *      Author: Jesse
 */

#ifndef MAIN_H_
#define MAIN_H_



//#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "storage.h"
#include "dosfs.h"
#include <stddef.h>
#include "stm32f10x.h"
#include "stm32_eval_sdio_sd.h"
#include "stm3210e_eval_fsmc_nor.h"
#include "stm3210e_eval_fsmc_nand.h"
#include "stm3210e_eval_fsmc_sram.h"
#include "SPI_FLASH.h"

#include "menu.h"
#include "optimalPWM.h"
#include "voltageControl.h"

#include "sound.h"

 #include "stm32_eval.h"
 #include "stm3210e_eval_lcd.h"
//#include "menu.h"

#include "myHeaders.h"


#define MAX_BMP_FILES 25
#define  MAX_MENU_LEVELS 4
#define  NOKEY  0
#define  SEL    1
#define  RIGHT  2
#define  LEFT   3
#define  UP     4
#define  DOWN   5
#define  KEY    6

#define SLIDE_SIZE    76866
#define SLIDE1		  0x64000000
#define SLIDE2		  0x64025842
#define Bank1_SRAM3_ADDR    ((uint32_t)0x68000000)
#define countof(a) (sizeof(a) / sizeof(*(a)))
void Menu_Init(void);
void DisplayMenu(void);
void SelFunc(void);
void UpFunc(void);
void DownFunc(void);
void ReturnFunc(void);
uint8_t ReadKey(void);
void IdleFunc(void);
void DisplayIcons(void);
void LCD_NORDisplay(uint32_t address);
void ShowMenuIcons(void);
void STM32_LCD_DemoIntro(void);
uint32_t CheckBitmapFiles(void);
void InternalFlashToLCD(void);
void InternalFlashToLCD_DMA(void);
void InternalFlashToLCD_Speed(void);
void NORFlashToLCD(void);
void NORFlashToLCD_DMA(void);
void NORFlashToLCD_Speed(void);
void NANDFlashToLCD(void);
void NANDFlashToLCD_Speed(void);
void ExternalSRAMToLCD(void);
void ExternalSRAMToLCD_DMA(void);
void ExternalSRAMToLCD_Speed(void);
void SDCardToLCD(void);
void SDCardToLCD_Speed(void);
void TimingMeasurement_Config(void);
void DisplayTimingCompute(void);
void CopyToInternalFlash(void);
void CopyToExternalSRAM(void);
void CopyToNANDFlash(void);
void CopyToSDCard(void);
void NAND_PhysicalErase(void);

void STM32_LCD_Demo(void);
void InterruptConfig(void);
void RCC_Config(void);
void NVIC_Config(void);
void GPIO_Config(void);
void PWMConfig(void);
void SysTick_Configuration(void);
void Delay(uint32_t nCount);
uint32_t DelayJoyStick(uint32_t nTime);
void Decrement_TimingDelay(void);


void CheckBitmapFilesStatus(void);
void IntExtOnOffConfig(FunctionalState NewState);
void ADC_Config(void);

void mySDWrite(void);
void updateBuffer(uint32_t i);


void copiedInt(void);
void frameDisplayedInt(void);
void bufferVideo(void);

//this function will draw the buffer from bufferAddress, for the amount of sizeBytes.
void drawBufferToLCD(uint16_t* bufferAddress, uint16_t sizeBytes);
void readSD(uint16_t * writeAddress, uint16_t fromSDAdress);
uint8_t read_once(void);
void Read_Ads7846(void);

void LCDDMA(uint32_t sourceAddress, uint16_t pixels);
void displayFrame();
void readSoundFile();
uint16_t setSoundPWM();

void updateWAVBuffer(uint8_t section);

//void initFlashVariables();
//void saveVariablesToFlash();

void initFlashVariables(uint16_t * readArray, uint8_t elements);


void saveVariablesToFlash(uint16_t *writeArray, uint8_t elements);


#endif /* MAIN_H_ */
