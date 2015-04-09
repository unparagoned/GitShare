//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f10x.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 2 / 3)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)


// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
/**
 *****************************************************************************
 **
 **  File        : main.c
 **
 **  Abstract    : main function.
 **
 **  Functions   : main
 **
 **  Environment : Atollic TrueSTUDIO/STM32
 **                STMicroelectronics STM32F10x Standard Peripherals Library
 **
ECGYM
Version: 0.1

Created by: Jesse Karadia
Unparagoned

Date: 05 January 2012

Description: Control device providing LCD touch interface for a tens machine.
Control DC-DC converter and the tens machine.

Features
PWM PA8,PA10,PA11,  output implemented. TIM1
PWM PB0,1 TIM8 control voltage buck
PB10 -Voltage drop for buck.
PC0, PC1 ADC implemented
 DAC -PA4,5,

Implemented GPIO or channels. PA1,2,3,(TIM5) 6,7.(TIM3) PB8,9(TIM4),10,11(TIM2) (not all timers implemented yet).
Timers TIM5 to control pins


Implementation details
PA7-PNP to control pulse
PA2,PA3 outputs
PC0 readings in
PA10-Boost PWM
PA6 Cap drain.
PA4 controlled by menu (basic)
PA5 -DAC out to control voltage- to opamp.
PC1-ADC in to measure voltage control.


To Do create debug interface, to look at problem areas.
-Likely want to implement feature or function mode.
MODE MYARM & MYDEBUG atm

//To Implement
 *Menu
 *

 **
 **
 *****************************************************************************
 */

/* Includes */

//#include "stm32f10x.h" can remove for debug purposes



#include "main.h"


//version 1 uses spi 1
#define VERSION 1
/* Private typedef */





//not that this will get overwritten or crash program if program is big and uses this space
//flash to store variables when off. can store ~4kb
#define flashStartAddress ((uint32_t)0x0807E000)
#define flashEndAddress ((uint32_t)0x0807F000)
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
#define FLASH_PAGE_SIZE    ((uint16_t)0x800)
uint32_t permanentVar=0;
uint16_t boostPeriod;
uint16_t boostOn;

uint16_t outputPeriods=0;
extern uint16_t LCDSizeX, LCDSizeY;
extern uint16_t numberOfCycles;

uint16_t tempNumberOfReads=0; //remove when sound properly implemented
/*
 * Map Permanent storage
 * Boost-Period, on
 */

//this initialises any permanent variables from flash.
//There is a possibility that power was cut from write, etc or something is wrong.
//So do a check on all variables, if one is out of bounds reset all from defaults.
//alternatively use two flash areas as in EEPROM notes.
/*
void initFlashVariables()
{
	//simply read data at the flash address into RAM
	//value at the address of a pointer.
	boostPeriod=*(uint16_t*)(flashStartAddress);
	boostOn=*(uint16_t*) (flashStartAddress+2);
}
*/
//more general read and write
void initFlashVariables(uint16_t * readArray, uint8_t elements)
{
	//simply read data at the flash address into RAM
	//value at the address of a pointer.
	int i=0;
	for(;i<elements;i++)
	{
		*(readArray+i)=*(uint16_t*)(flashStartAddress+2*i);
	}
}

void saveVariablesToFlash(uint16_t *writeArray, uint8_t elements)
{
	uint32_t EraseCounter = 0x00, Address = 0x00;
	uint32_t Data = 0x3210ABCD;
	uint32_t NbrOfPage = 0x00;
	volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
	volatile TestStatus MemoryProgramStatus = PASSED;
	/* Porgram FLASH Bank1 ********************************************************/
	/* Unlock the Flash Bank1 Program Erase controller */
	FLASH_UnlockBank1();

	/* Define the number of page to be erased */
	NbrOfPage = (flashEndAddress - flashStartAddress) / FLASH_PAGE_SIZE;

	/* Clear All pending flags */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	/* Erase the FLASH pages */
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		FLASHStatus = FLASH_ErasePage(flashStartAddress + (FLASH_PAGE_SIZE * EraseCounter));
	}

	/* Program Flash Bank1 */
	Address = flashStartAddress;

	while((Address < flashEndAddress) && (FLASHStatus == FLASH_COMPLETE))
	{
		//FLASHStatus = FLASH_ProgramWord(Address, Data);
		Address = Address + 4;
	}
	int i=0;
	for(;i<elements;i++)
	{
		FLASHStatus = FLASH_ProgramHalfWord(flashStartAddress+2*i, *(writeArray+i));
	}

	FLASH_LockBank1();
}

//this saves all variables to flash.
#ifdef NOTDEFINED
void saveVariablesToFlash()
{
	uint32_t EraseCounter = 0x00, Address = 0x00;
	uint32_t Data = 0x3210ABCD;
	uint32_t NbrOfPage = 0x00;
	volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
	volatile TestStatus MemoryProgramStatus = PASSED;
	/* Porgram FLASH Bank1 ------ */
	/* Unlock the Flash Bank1 Program Erase controller */
	FLASH_UnlockBank1();

	/* Define the number of page to be erased */
	NbrOfPage = (flashEndAddress - flashStartAddress) / FLASH_PAGE_SIZE;

	/* Clear All pending flags */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	/* Erase the FLASH pages */
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		FLASHStatus = FLASH_ErasePage(flashStartAddress + (FLASH_PAGE_SIZE * EraseCounter));
	}

	/* Program Flash Bank1 */
	Address = flashStartAddress;

	while((Address < flashEndAddress) && (FLASHStatus == FLASH_COMPLETE))
	{
		//FLASHStatus = FLASH_ProgramWord(Address, Data);
		Address = Address + 4;
	}
	FLASHStatus = FLASH_ProgramHalfWord(flashStartAddress, boostPeriod);
	FLASHStatus = FLASH_ProgramHalfWord(flashStartAddress+2, boostOn);

	FLASH_LockBank1();
}

#endif


void sysDelay(__IO uint32_t nTime);



/* Private define  */
DAC_InitTypeDef            DAC_InitStructure;

__IO uint16_t timerIntBase=500;

#define BlockSize            512 /* Block Size in Bytes */
#define BufferWordsSize      (BlockSize >> 2)

#define VIDEODATA    ((uint32_t)0x60020000)

#define MESSAGE1   " ECGYM    "
#define MESSAGE2   "  Touch Screen "
#define MESSAGE3   "   Built By    "
#define MESSAGE4   "   Unparagoned  "

#define MESSAGE5   " Version 0.1 "
#define MESSAGE6   "  "

//touch commands
#define CMD_RDX 0X90
#define CMD_RDY	0XD0


#define DOUT GPIOA->IDR&1<<6
#define PEN  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)   //PB6  GPIOC
#define TDIN  (1<<7)  // PA7
#define TCLK  (1<<5)  // PA5

#define MAINOPTIONS 1
#define VIDEO 4
#define DRAW 3
#define QUITOPTIONS 2


/* Private macro */
#define TDIN_SET(x) GPIOA->ODR=(GPIOA->ODR&~TDIN)|(x ? TDIN:0)
#define TCLK_SET(x) GPIOA->ODR=(GPIOA->ODR&~TCLK)|(x ? TCLK:0)
#define TCS_SET(x)  GPIOA->ODR=(GPIOA->ODR&~TCS)|(x ? TCS:0)

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC2_DR_Address    ((uint32_t)0x4001284C)
#define ADC3_DR_Address    ((uint32_t)0x40013C4C)

ADC_InitTypeDef  ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;

__IO uint16_t ADCConvertedValue;
__IO uint16_t ADCConvertedValuePC1;
__IO uint16_t ADCConvertedValuePC2;
/* Private variables */

//LCD variables
extern __IO uint32_t DMAComplete;

//Video format variables
uint16_t metaDataOffset=0;
uint16_t addressOffset=8;

uint16_t frameNumber=1;
uint16_t metaDataOffsetLoc=20;
uint16_t addressOffsetLoc=8;


uint16_t thecolor=65535;
uint16_t color1=0;
uint16_t tempCol=36500;
uint32_t SDAddress=0;

//sound stuff
uint16_t wavmetaDataOffsetLoc=20;
uint16_t wavaddressOffsetLoc=8;
uint32_t wavSDAddress=0;
uint16_t wavmetaDataOffset=0;
uint16_t wavaddressOffset=8;
uint16_t wavframeNumber=1;
//Touch variables
uint16_t X,Y;
uint8_t TCS=(1<<7); //this isn't defined since it depends on the dev board

uint8_t programState=MAINOPTIONS;
uint8_t previousProgramState=MAINOPTIONS;

uint8_t _it1;
uint8_t _it0;
EXTI_InitTypeDef EXTI_InitStructure;
//sound stuff in sound sound.h
//#define SOUNDBUFFERSIZE 12288
//uint16_t soundBufferSizeVar=SOUNDBUFFERSIZE;
uint16_t displayBuffer[320*16]; //holds 16 lines of information
//uint8_t soundBuffer[SOUNDBUFFERSIZE];
//uint16_t soundBufferIndex;
//uint8_t soundBufferUpdate=0;



SD_Error Status = SD_OK;


static __IO ErrorStatus HSEStartUpStatus = SUCCESS;
//static __IO uint32_t TimingDelay = 0;
__IO uint32_t TimingDelay = 0; //don't want it to be static
extern unsigned int LCD_IDP;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;

extern __IO uint8_t largeTransfer;
extern __IO uint8_t SDIOEnd;
/* Private function prototypes */
void NVIC_Configuration(void);



/* Private functions */

void sysDelay(__IO uint32_t nTime)
{
	TimingDelay = nTime;

	// while(TimingDelay != 0);
}




/**
 * @brief  Idle function.
 * @param  None
 * @retval : None
 */
void IdleFunc(void)
{
	/* Nothing to execute: return */
	return;
}

/**
 * @brief  Displays data to LCD using DMA transfer
 * @param  sourceAddress address to pointer to pixels, pixels-the number of pixels to display or size of data array
 * @retval :
 */
void LCDDMA(uint32_t sourceAddress, uint16_t pixels)
{

	//temp debuging
	//LCDDisplaySlow(sourceAddress,pixels);
	//return;

	//why wait at end when it can wait only required at start.
	// Gives Large FPS boost. But doesn't alway work

	while(DMAComplete == 0)
	{
	}

	DMA_InitTypeDef  DMA_InitStructure;
	numberOfCycles=1;


	/* DMA1 channel3 configuration */
	// DMA_DeInit(DMA1_Channel3);
	DMA_DeInit(DMA2_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)sourceAddress;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)VIDEODATA;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = pixels;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;

	//  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	DMA_Init(DMA2_Channel4, &DMA_InitStructure);
	/* Enable DMA Channel3 Transfer Complete interrupt */
	//DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	DMA_ITConfig(DMA2_Channel4, DMA_IT_TC, ENABLE);
	/* Enable DMA1 channel3 */
	//DMA_Cmd(DMA1_Channel3, ENABLE);

	DMAComplete = 0;
	DMA_Cmd(DMA2_Channel4, ENABLE);
	/* Wait for DMA transfer Complete */
	//why wait? Can wait at start. Wait at start caused some problems.
	//enable temporary, since sd card can write directly
	/*
   while(DMAComplete == 0)
   {
   }
	 */
}

void LCDDisplaySlow(uint32_t sourceAddress, uint16_t pixels)
{
	uint16_t *pixelData;
	pixelData=sourceAddress;
	uint16_t tempCounter=0;
	uint16_t pixelCounter=0;
	uint16_t tempColor=*pixelData;
	for(tempCounter=0;tempCounter<pixels;tempCounter++)
	{
		LCD_Write_DATA(pixelData[tempCounter]);
		//	 LCD_Write_DATA(LCD_COLOR_GREEN);
		// LCD_Write_DATA(LCD_COLOR_BLUE2);
		// LCD_Write_DATA(tempColor);
		if(tempCounter%320==0)
		{
			// tempColor=pixelData[tempCounter];
			pixelCounter=0;
		}
	}
	return;
	// LCD_WriteRAM_Prepare();

	for(pixelCounter=0;pixelCounter<pixels;pixelCounter++)
	{
		LCD_Write_DATA(*pixelData);
		pixelData++;
		if(pixelCounter%320==0)
		{
			tempCounter=0;
		}
	}
}
/**
 * @brief  Assigns string to a char array
 * @param  textPointer, arraySize-size of text array, text-text to be copied,
 * @retval :returns 1 if sucessful, 0 if error.
 */
uint8_t assignString(uint8_t *textPointer, uint8_t arraySize, char text[] )
{
	uint8_t charPosition;

	for(charPosition=(arraySize);charPosition>0;charPosition--)
	{
		textPointer[charPosition-1]=text[charPosition-1];
	}
	return 1;
}

/**
 * @brief  Displays a crude options menu. -Should replace with a proper GUI library
 * @param  option- type of option menu to display
 * @retval :
 */
void drawOptions(uint8_t option)
{
	LCD_SetTextColor(Black);
	LCD_SetBackColor(Grey);
	uint8_t  optionTextA[9];
	uint8_t  optionTextB[9];
	uint8_t padding[]="        ";

	if(option==1)
	{
		assignString(optionTextA,9,"  Draw  ");
		assignString(optionTextB,9,"  Video ");

	}
	if(option==2)
	{
		assignString(optionTextA,9," Resume ");
		assignString(optionTextB,9,"  Quit  ");
	}

	LCD_DisplayStringLine(Line2,padding);
	LCD_DisplayStringLine(Line3,optionTextA);
	LCD_DisplayStringLine(Line4,padding);

	LCD_DisplayStringLine(Line6,padding);
	LCD_DisplayStringLine(Line7,optionTextB);
	LCD_DisplayStringLine(Line8,padding);

}
void PWMGPIOConfig()
{

	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);
}
void ADCGPIOConfig()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//LED out
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */

void updateTimerInt(uint16_t timerTime)
{


	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = timerTime;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Disable);

	return;
}



// ----- main() ---------------------------------------------------------------



int
main(int argc, char* argv[])
{
  // Send a greeting to the trace device (skipped on Release).

	//commented out since it causes device to hard fault crash when not debugging.
 // trace_puts("Hello ARM World!");

//	printf("Hello world \n");
//	trace_printf("Hello Worlds \n");

  //sets the state of the program.
  	thisState=MYARM;

  	// TODO

  	if (SysTick_Config(SystemCoreClock / 1000))
  	{
  		/* Capture error */
  		while (1);
  	}


  	//initFlashVariables();
  	//saveVariablesToFlash();

  	int i = 0;

  	LCD_CtrlLinesConfig();

  	RCC_Configuration();
  	GPIO_InitTypeDef GPIO_InitStructure;
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	/* DAC channel1 Configuration */
  	DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
  	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_4095;
  	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  	DAC_Cmd(DAC_Channel_1, ENABLE);
  	DAC_Cmd(DAC_Channel_2, ENABLE);
  	/* Set DAC dual channel DHR12RD register */
  	//disabled as I probably don't need it.
  	//DAC_SetDualChannelData(DAC_Align_12b_R, 0x100, 0x100);


  	//GPIO setup
  	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);


  	//analogue softswitch off
  	GPIO_ResetBits(GPIOA, GPIO_Pin_1);

  	//cap drain when off
  	GPIO_ResetBits(GPIOA, GPIO_Pin_6);


  	PWMGPIOConfig();
  	ADCGPIOConfig();
  	ADCConfig();
  	PWMConfig(TIM1, 5);

  	//see if we can get the timer to do an interrupt for us
  	  /* Enable the TIM2 Interrupt */


  	TIM_ITConfig(TIM1,TIM_IT_Update, ENABLE);

  	ADCConfig2();


  	//tim5 enable
  	TimerPeriod = (SystemCoreClock / 17570 ) - 1;

  	//TimerPeriod=1000;
  	/* Time Base configuration */
  	TIM_TimeBaseStructure.TIM_Prescaler = 8;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  	TIM_SetCompare1(TIM5, 200);
  	// updateTimerInt(0);

  	TIM_ITConfig(TIM5,TIM_IT_CC1, ENABLE);

  	/* TIM1 counter enable */
  	TIM_Cmd(TIM5, ENABLE);




  	/* Configure the FSMC Parallel interface -------------------------------------*/
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  	LCD_FSMCConfig();

  	STM3210E_LCD_Init();

  	LCD_Clear(White);

  	/* Set the LCD Back Color */
  	LCD_SetBackColor(Blue);
  	/* Set the LCD Text Color */
  	LCD_SetTextColor(White);
  	LCD_DisplayStringLine(Line0, (uint8_t *)MESSAGE1);
  	LCD_DisplayStringLine(Line1, (uint8_t *)MESSAGE2);
  	LCD_DisplayStringLine(Line2, (uint8_t *)MESSAGE3);
  	LCD_DisplayStringLine(Line3, (uint8_t *)MESSAGE4);
  	LCD_DisplayStringLine(Line4, (uint8_t *)MESSAGE5);
  	LCD_DisplayStringLine(Line5, (uint8_t *)MESSAGE6);

  	/* DMA1 and DMA2 clock enable */
  	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);

  	/* Interrupt Config */
  	NVIC_Configuration();


  	EXTI_InitTypeDef EXTI_InitStructure;
  	NVIC_InitTypeDef NVIC_InitStructure;
  	/* Enable the BUTTON Clock */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);		 //触摸IRQ

  	/* Configure Key Button EXTI Line to generate an interrupt on falling edge */
  	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  	NVIC_Init(&NVIC_InitStructure);
  	/*------------------------------ SD Init ---------------------------------- */
  	if((Status = SD_Init()) != SD_OK)
  	{
  		STM_EVAL_LEDOn(LED4);
  	}

  	/* Display message on STM3210X-EVAL LCD */
  	/* Clear the LCD */
  	LCD_Clear(White);

  	/* Set the LCD Back Color */
  	LCD_SetBackColor(Blue);
  	/* Set the LCD Text Color */
  	LCD_SetTextColor(White);
  	LCD_DisplayStringLine(Line0, (uint8_t *)MESSAGE1);
  	LCD_DisplayStringLine(Line1, (uint8_t *)MESSAGE2);
  	LCD_DisplayStringLine(Line2, (uint8_t *)MESSAGE3);
  	LCD_DisplayStringLine(Line3, (uint8_t *)MESSAGE4);
  	LCD_DisplayStringLine(Line4, (uint8_t *)MESSAGE5);
  	LCD_DisplayStringLine(Line5, (uint8_t *)MESSAGE6);

  	//while loop for display
  	/*move to global
    uint16_t thecolor=65535;
    uint16_t color1=0;
    uint16_t tempCol=36500;
    uint32_t SDAddress=0;
  	 */
  	Storage_Init();


  	/*temp in global to move display out of main
    uint16_t frameNumber=1;
    uint16_t metaDataOffsetLoc=20;
    uint16_t addressOffsetLoc=8;
  	 */
  	metaDataOffset= metaDataOffsetLoc;
  	addressOffset=addressOffsetLoc;

  	//  SPI1_Init();
  	SPI2_Init();




  	uint8_t pauseState=0;
  	/* TODO Addcode */
  	_it1=0;
  	_it0=0;
  	LCD_Clear(White);
  	programState=VIDEO;

  	uint16_t x=1000;

  	//soft analogue on
  	GPIO_SetBits(GPIOA, GPIO_Pin_1);

  	//cap drain off
  	GPIO_SetBits(GPIOA, GPIO_Pin_6);



  	menuInit();
  	loadAction(0);

  	uint16_t thePeriod=0;
  	uint16_t theDuty=0;

  	uint16_t timeArray[1000][2];
  	uint16_t timerCounter;

  	//The sets voltage to 50V
  	TIM1->ARR=300;
  	TIM1->CCR3=10;
  	outputPeriods=0;
  	for(timerCounter=0;timerCounter<1000;timerCounter++)
  	{
  		timeArray[timerCounter][0]=ADCConvertedValue;
  		timeArray[timerCounter][1]=outputPeriods;
  		delay(1000);
  	}

  	uint32_t ADCAverage=0;
  	// optimumDuty(&thePeriod,&theDuty);

  	GPIO_SetBits(GPIOA, GPIO_Pin_7);
  	GPIO_SetBits(GPIOA, GPIO_Pin_6);




  	playSound("music.wav");
  	/* TODO Addcode */

  	uint16_t tempCounterS=0;
  	uint8_t updateSoundVar=1;
  	while(1)
  	{
  //temp sound test code








  			if(soundBufferIndex>=SOUNDBUFFERSIZE/2 && updateSoundVar==1)
  			{
  				updateSoundVar= 2;
  				soundBufferUpdate=1;
  			}
  			if(soundBufferUpdate==2)
  			{
  				updateSoundVar=1;
  			}



  		updateWAVBuffer(soundBufferUpdate);
  		soundBufferUpdate=0;






  	//	reduceOutput(0);

  		GPIO_SetBits(GPIOB,  GPIO_Pin_8|GPIO_Pin_9);
  		GPIO_ResetBits(GPIOB, GPIO_Pin_8|GPIO_Pin_9);

  		while(menuInterface());


  		// uint8_t TimerDisplay[]="123456";
  		//  sprintf(TimerDisplay, "%d",TIM_GetCounter(TIM5));
  		//  LCD_DisplayStringLine(Line5, TimerDisplay);
  		//This while loop just displays touch position to the screen.
  		/*
  	while(1)
  	{
  uint16_t positionx, positiony;
  if(_it1)
  {
  	//getRelTouch(&positionx,&positiony);
  }
  uint8_t TimerDisplay[]="X pos is:some number, y pos is some number";
  sprintf(TimerDisplay, "X:%d, y%d",positionx, positiony);
  LCD_DisplayStringLine(Line5, TimerDisplay);
  displayFrame();
  	}
  		 */
  		//DAC input varies allot leading to spike on output, average it out
  		if(timerCounter<10)
  		{
  			timerCounter++;
  			ADCAverage+=ADCConvertedValue;
  		}
  		if(timerCounter>9)
  		{
  			//DAC->DHR12R2=ADCAverage/10;
  			//DAC->SWTRIGR=3;

  			ADCAverage=0;
  			timerCounter=0;
  		}

  		if(programState<=2)
  		{
  			drawOptions(programState);
  		}

  		if(_it1)
  		{
  			startMenu();
  		}


  		if(pauseState==0 && programState==4)
  		{

  			displayFrame();

  			//This should be adjusted by software rather than directly adc anyway. Maybe we can set the actual value on touch. Use VR1 for other adjustments now
  			// adjustPWM(ADCConvertedValue);
  			// adjustPWM(3180);
  		}
  		//don't need to do this anymore. this pin used by sound now,
  		//updateBoostDuty();
  	}

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %uHz\n", SystemCoreClock);

  timer_start();

  blink_led_init();
  
  uint32_t seconds = 0;


}

void displayFrame()
{
	//temporarily disable to make sound faster
//return;
	while(DMAComplete == 0)
	{
	}

	uint8_t FolderLoc[]="";
	uint16_t fileResolution=2;
	uint8_t * FileName="rgb.avi";
	uint16_t numberOfBlocks=294;
	LCD_SetCursor(LCDSizeX-1, LCDSizeY-1);
	//LCD_SetCursor(239, 319);
	if(LCD_IDP==0)
	{
		FileName="rgb2.avi";
		fileResolution=2;
		numberOfBlocks=1494;
		setArea(0,0,799,479);
	}



	if(frameNumber==1)
	{
		SDAddress=getFileSector(FolderLoc,FileName);
		SDAddress+=8192;
		metaDataOffset= metaDataOffsetLoc;
		addressOffset=addressOffsetLoc;

	}

	LCD_WriteRAM_Prepare();


	//SD_WaitReadOperation();
	//while(SD_GetStatus() != SD_TRANSFER_OK);
	//this skips metadata
	SD_ReadMultiBlocks((uint8_t*)displayBuffer, SDAddress, 512, 6);
	//SD_LCDReadMultiBlocks((uint8_t*)VIDEODATA, SDAddress, 512, 20);
	SD_WaitReadOperation();
	while(SD_GetStatus() != SD_TRANSFER_OK);
	//66 is 0x42, size of meta data. 33 unit16 is equiv.

	LCDDMA(displayBuffer+metaDataOffset/2, (3072-metaDataOffset)/2);
	SDAddress+=3072;
	//while(DMAComplete==0);

	SDIOEnd=0;
	largeTransfer=1;
	SD_LCDReadMultiBlocks((uint8_t*)VIDEODATA, SDAddress, 512, numberOfBlocks);
	SD_WaitReadOperation();
	while(SD_GetStatus() != SD_TRANSFER_OK);
	//while(DMAComplete==0);

	SDAddress+=numberOfBlocks*512;

	largeTransfer=0;
	// LCD_WriteRAM_Prepare();
	//Reads end bit because of metadata.
	//SD_LCDReadMultiBlocks((uint8_t*)VIDEODATA, SDAddress, 512, 1);
	//Delay(1000);
	SDIOEnd=0;
	SD_ReadMultiBlocks((uint8_t*)displayBuffer, SDAddress, 512, 1);
	SD_WaitReadOperation();
	while(SD_GetStatus() != SD_TRANSFER_OK);
	//66 is 0x42, size of meta data.
	//drawBufferToLCD(displayBuffer,metaDataOffset);
	LCDDMA(displayBuffer, (metaDataOffset)/2);



	metaDataOffset+=addressOffset;

	if(metaDataOffset>512)
	{
		metaDataOffset-=512;
		SDAddress+=512;
	}

	if(frameNumber++>875)
	{
		frameNumber=1;
	}
}

//doesn't play it but reads it, rename and redo this
void playSound(uint8_t* fileName)
{

	while(DMAComplete == 0)
	{
	}

	uint8_t FolderLoc[]="";
	//uint16_t fileResolution=2;
	//uint8_t * FileName="rgb.avi";
	//uint16_t numberOfBlocks=294;
	//LCD_SetCursor(LCDSizeX-1, LCDSizeY-1);
	//LCD_SetCursor(239, 319);

	if(wavframeNumber==1)
	{
		wavSDAddress=getFileSector(FolderLoc,fileName);
		//wavSDAddress+=8192;
		wavmetaDataOffset= wavmetaDataOffsetLoc;
		wavaddressOffset=wavaddressOffsetLoc;

	}

	SD_ReadMultiBlocks((uint8_t*)soundBuffer, wavSDAddress, 512, 24);
	//SD_LCDReadMultiBlocks((uint8_t*)VIDEODATA, SDAddress, 512, 20);
	SD_WaitReadOperation();
	while(SD_GetStatus() != SD_TRANSFER_OK);
	//66 is 0x42, size of meta data. 33 unit16 is equiv.

		wavSDAddress+=12288;
	//while(DMAComplete==0);

	SDIOEnd=0;
	//largeTransfer=1;
	wavmetaDataOffset+=wavaddressOffset;



	if(wavframeNumber++>87500)
	{
		wavframeNumber=1;
	}

	TIM1->ARR=256;

	soundBufferIndex=44; //skips the meta deta

}

void updateWAVBuffer(uint8_t section)
{
	if(tempNumberOfReads++>8500)
	{
		tempNumberOfReads=0;
		uint8_t FolderLoc[]="";
		wavSDAddress=getFileSector(FolderLoc,"music.wave");

		return;
	}
	if(section==0)
	{
		return;
	}
	uint8_t* bufferLoc=0;
	if(section==1)
	{
		bufferLoc=(uint8_t*)soundBuffer;
	}
	if(section==2)
	{
		//no divide by 2, since 8 it vs 16 bit
		bufferLoc=(uint8_t*)(soundBuffer+SOUNDBUFFERSIZE/2);
	}

	SD_ReadMultiBlocks(bufferLoc, wavSDAddress, 512, 12);
	//SD_LCDReadMultiBlocks((uint8_t*)VIDEODATA, SDAddress, 512, 20);
	SD_WaitReadOperation();
	while(SD_GetStatus() != SD_TRANSFER_OK);
	//66 is 0x42, size of meta data. 33 unit16 is equiv.

		wavSDAddress+=SOUNDBUFFERSIZE/2;



}

//return 1 if buffer needs to be filled, returns 2 if second half of buffer needs to be filled, returns 0 if buffer is fine
uint16_t setSoundPWM()
{
	TIM1->CCR3=soundBuffer[soundBufferIndex];
	soundBufferIndex+=1;

	if(TIM1->CCR3>0)
	{
		//soundBufferIndex++;
	}

	if(soundBufferIndex>=SOUNDBUFFERSIZE)
	{
		soundBufferIndex=0;
		return 2;
	}

	if(soundBufferIndex==SOUNDBUFFERSIZE/2)
	{
		return 1;
	}


	return 0;
}


void PWMConfig(TIM_TypeDef *theTimer, uint16_t dutyCycle)
{

//looks like timers are initiated more than once.


	//this sets the counter to 2^12
	TimerPeriod = (SystemCoreClock / 17570 ) - 1;
	/* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
	Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
	/* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 and 2N */
	Channel2Pulse = (uint16_t) (((uint32_t) 375 * (TimerPeriod - 1)) / 1000);
	/* Compute CCR3 value to generate a duty cycle at 25%  for channel 3 and 3N */
	Channel3Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1)) / 100);
	/* Compute CCR4 value to generate a duty cycle at 12.5%  for channel 4 */
	Channel4Pulse = (uint16_t) (((uint32_t) 125 * (TimerPeriod- 1)) / 1000);
	Channel3Pulse = (uint16_t) (((uint32_t) dutyCycle * (TimerPeriod - 1)) / 100);
	Channel2Pulse =Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 8;

	TIM_TimeBaseInit(theTimer, &TIM_TimeBaseStructure);

	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

//this enables all 4 channels on whatever timer calls this function. So TIM11.

	TIM_OC1Init(theTimer, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
	TIM_OC2Init(theTimer, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	TIM_OC3Init(theTimer, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
	TIM_OC4Init(theTimer, &TIM_OCInitStructure);

	/* theTimer counter enable */
	TIM_Cmd(theTimer, ENABLE);

	/* theTimer Main Output Enable */
	TIM_CtrlPWMOutputs(theTimer, ENABLE);

	//tim8 config
	theTimer=TIM8;

	TIM_TimeBaseStructure.TIM_Prescaler = 0;

	/* TIM8 PWM mode */
	TimerPeriod=500;
	Channel3Pulse=160;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	TIM_TimeBaseInit(theTimer, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;



	// TIM_OC2Init(theTimer, &TIM_OCInitStructure);
	//TIM8 channel 3, init
	TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	TIM_OC3Init(theTimer, &TIM_OCInitStructure);

	/* theTimer counter enable */
	TIM_Cmd(theTimer, ENABLE);

	/* theTimer Main Output Enable */
	TIM_CtrlPWMOutputs(theTimer, ENABLE);


	theTimer=TIM2;



		/* Channel 1, 2,3 and 4 Configuration in PWM mode */
		TimerPeriod=500;
		Channel3Pulse=160;
		TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
		TIM_TimeBaseInit(theTimer, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
		TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;



		// TIM_OC2Init(theTimer, &TIM_OCInitStructure);
		//TIM2 channel 3 enable - fails assert check, CtrlPWM isn't valid for TIM2
		TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
		TIM_OC3Init(theTimer, &TIM_OCInitStructure);

		  TIM_OC3PreloadConfig(theTimer, TIM_OCPreload_Enable);

		  TIM_ARRPreloadConfig(theTimer, ENABLE);

		  /* TIM3 enable counter */
		  TIM_Cmd(theTimer, ENABLE);


		  //not valid for TIM2
		/* theTimer Main Output Enable */
	///	TIM_CtrlPWMOutputs(theTimer, ENABLE);

}

void adjustPWM(uint16_t dutyCycle)
{
	//should just be the following line, disabled for the moment.
	TIM1->CCR3=dutyCycle;

	//TIM_DeInit(TIM1);

	/* TIM1 counter enable */
	//	 TIM_Cmd(TIM1, DISABLE);

	/* TIM1 Main Output Enable */
	//	 TIM_CtrlPWMOutputs(TIM1, DISABLE);
	//Channel3Pulse = (uint16_t) (((uint32_t) dutyCycle * (TimerPeriod - 1)) / 100);

	// TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	//TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	/* TIM1 counter enable */
	//TIM_Cmd(TIM1, ENABLE);

	/* TIM1 Main Output Enable */
	//TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void ADCConfig()
{

	//configures ADC for PC0,1,2

	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;// &(TIM1->CCR3);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);



	/* ADC1 Configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel14 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_13Cycles5);




	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}
void ADCConfig2()
{
	// TODO

	//configures ADC for PC0,1,2

	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA2_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC3_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValuePC1;// &(TIM1->CCR3);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA2_Channel5, ENABLE);



	/* ADC3 Configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC3, &ADC_InitStructure);

	/* ADC3 regular channel14 configuration */

	ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5);
	// ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_13Cycles5);



	/* Enable AWD interrupt */
	ADC_ITConfig(ADC3, ADC_IT_AWD, ENABLE);
	/* Enable ADC3 DMA */
	ADC_DMACmd(ADC3, ENABLE);
	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);

	/* Enable ADC3 reset calibration register */
	ADC_ResetCalibration(ADC3);
	/* Check the end of ADC3 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC3));

	/* Start ADC3 calibration */
	ADC_StartCalibration(ADC3);
	/* Check the end of ADC3 calibration */
	while(ADC_GetCalibrationStatus(ADC3));

	/* Start ADC3 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC3, ENABLE);

}

/**
 * @brief  Configures the different system clocks.
 * @param  None
 * @retval None
 */
void RCC_Configuration(void)
{
	//adc clock speed
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);
	/* TIM1, GPIOA, GPIOB, GPIOE and AFIO clocks enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 |RCC_APB2Periph_ADC2| RCC_APB2Periph_ADC3 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8 |  RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOE|
			RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);

	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC| RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM5, ENABLE);


}

/**
 * @brief  Configures the used IRQ Channels and sets their priority.
 * @param  None
 * @retval : None
 */
void InterruptConfig(void)
{

	//doesn't even look, like this is called. this is though void NVIC_Configuration(void)

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);

	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x0);

	/* Enable the EXTI3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the EXTI9_5 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the EXTI15_10 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable DMA channel3 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel =  DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//ADC int
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	  NVIC_InitStructure.NVIC_IRQChannel =  TIM1_UP_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configure a SysTick Base time to 10 ms.
 * @param  None
 * @retval : None
 */
void SysTick_Configuration(void)
{
	/* Setup SysTick Timer for 10 msec interrupts  */
	if (SysTick_Config(SystemCoreClock / 100))
	{
		/* Capture error */
		while (1);
	}
}

/**
 * @brief  Enables or disables EXTI for the menu navigation keys :
 *   EXTI lines 3, 7 and 15 which correpond respectively
 *   to "DOWN", "SEL" and "UP".
 * @param wState: New state of the navigation keys. This parameter
 *   can be: ENABLE or DISABLE.
 * @retval : None
 */
void IntExtOnOffConfig(FunctionalState NewState)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Initializes the EXTI_InitStructure */
	EXTI_StructInit(&EXTI_InitStructure);

	/* Disable the EXTI line 3, 7 and 15 on falling edge */
	if(NewState == DISABLE)
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line3 | EXTI_Line7 | EXTI_Line15;
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;
		EXTI_Init(&EXTI_InitStructure);
	}
	/* Enable the EXTI line 3, 7 and 15 on falling edge */
	else
	{
		/* Clear the the EXTI line 3, 7 and 15 interrupt pending bit */
		EXTI_ClearITPendingBit(EXTI_Line3 | EXTI_Line7 | EXTI_Line15);

		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Line = EXTI_Line3 | EXTI_Line7 | EXTI_Line15;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
	}
}

/**
 * @brief  Configures the different GPIO ports pins.
 * @param  None
 * @retval : None
 */
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PG.07, PG.08, PG.13, PG.14 and PG.15 as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	/* Configure PD.03 as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* RIGHT Button */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource13);

	/* LEFT Button */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource14);

	/* DOWN Button */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource3);

	/* UP Button */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource15);

	/* SEL Button */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource7);

	/* KEY Button */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource8);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	/* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIO_SetBits(GPIOF, GPIO_Pin_6);
}

/**
 * @brief  Inserts a delay time.
 * @param ount: specifies the delay time length (time base 10 ms).
 * @retval : None
 */
void Delay(__IO uint32_t nCount)
{
	for(; nCount != 0; nCount--);
}


/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval : None
 */
void Decrement_TimingDelay(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}
}


/**
 * @brief  Configures SDIO IRQ channel.
 * @param  None
 * @retval None
 */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//touch interupt


	/* Enable the EXTI9_5 Interrupt */

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn  ;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	NVIC_InitStructure.NVIC_IRQChannel =  DMA1_Channel3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel =  DMA2_Channel4_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel =  TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel =  TIM1_UP_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);


}


void delay_us(uint16_t delay)
{
	Delay(delay);
}

uint8_t read_once(void)
{	unsigned int a,b;
TCS_SET(0);
delay_us(5);
SPI_TypeDef *currentChan=SPI2;




SPI_SendByteChan(currentChan,CMD_RDY);

delay_us(5);
a=SPI_SendByteChan(currentChan,0);
a=a<<8;
a|=SPI_SendByteChan(currentChan,0);
delay_us(5);
TCS_SET(1);
a>>=3;
Y=a;
delay_us(15);
TCS_SET(0);
delay_us(5);
SPI_SendByteChan(currentChan,CMD_RDX);
delay_us(5);
b=SPI_ReadByteChan(currentChan,0);
b=b<<8;
b|=SPI_ReadByteChan(currentChan,0);
delay_us(5);
b>>=3;
X=b;

TCS_SET(1);
if(X>372&&Y>35&&X<4000&&Y<4061)return 1;//读数成功(范围限制)
else return 0;			                 //读数失败
}

void Read_Ads7846(void)
{	float X1,Y1,hh;
u16 x1,x2,y1,y2,xx;
u8 t,t1,count=0;
u16 databuffer[2][10];//数据组
u16 temp=0;
do					  //循环读数10次
{
	//reads position 10 times for a good average and pen in contact long enough.
	if(read_once())//读数成功
	{
		databuffer[0][count]=X;
		databuffer[1][count]=Y;
		count++;
	}
	t=PEN;
}while(!t&&count<10);
//}while(count<10);
if(count==10)//一定要读到10次数据,否则丢弃
{
	do//sort into order
	{
		t1=0;
		for(t=0;t<count-1;t++)
		{
			if(databuffer[0][t]>databuffer[0][t+1])//升序排列
			{
				temp=databuffer[0][t+1];
				databuffer[0][t+1]=databuffer[0][t];
				databuffer[0][t]=temp;
				t1=1;
			}
		}
	}while(t1);
	do//some kind of sort into order
	{
		t1=0;
		for(t=0;t<count-1;t++)
		{
			if(databuffer[1][t]>databuffer[1][t+1])//升序排列
			{
				temp=databuffer[1][t+1];
				databuffer[1][t+1]=databuffer[1][t];
				databuffer[1][t]=temp;
				t1=1;
			}
		}
	}while(t1);
	x1=databuffer[0][3]; x2=databuffer[0][4]; //x3=databuffer[0][8];
	y1=databuffer[1][3]; y2=databuffer[1][4]; //y3=databuffer[1][8];
	//y1=36;
	//drawbigpoint(239-((17*(y1-180))>>8),319-((3*(x1-372))>>5));
	uint16_t x,y;

	x=239-((17*(y1-180))>>8);
	y=319-((3*(x1-372))>>5);

	if(LCD_IDP==0x8989 || LCD_IDP==0x9320)
	{
		x=239-x;
	}

	//safety balance incase touch is different on each screen.
	//Using 238 instead of 239 is because circle of radius 1 is drawn.
	x=(x>238?238:x);
	y=(y>318?318:y);

	if(programState==MAINOPTIONS)
	{

		if(x<120)
		{
			programState=DRAW;
		}
		else
		{
			programState=VIDEO;
		}
		LCD_Clear(White);
		return;
	}
	if(programState==QUITOPTIONS)
	{
		LCD_Clear(White);
		if(x>120)
		{
			programState=MAINOPTIONS;
			LCD_Clear(White);
		}
		else
		{
			programState=previousProgramState;
		}
		return;
	}
	if(programState>2)
	{
		if( programState>2)
		{
			if(x<36 && y<48)
			{
				startMenu();
				return;
				previousProgramState=programState;
				programState=QUITOPTIONS;

				return;
			}
		}
	}
	if(programState==DRAW)
	{

		LCD_SetTextColor(Black);
		LCD_SetBackColor(Red);
		LCD_DrawFullCircle(x,y,1);
	}
	return;


}
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
//defined elsewhere

void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	trace_printf ("assert_param() failed: file \"%s\", line %d\n", file, line);
	/* Infinite loop */
	while (1)
	{
	}
}

#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
// defined elsewhere

void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
	 trace_printf ("assert_param() failed: file \"%s\", line %d\n", file, line);
	while(1)
	{}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
//useing libary one

void __assert(const char *file, int line, const char *failedexpr)
{
	 trace_printf ("assert_param() failed: file \"%s\", line %d\n", file, line);
	__assert_func (file, line, NULL, failedexpr);
}

#ifdef USE_SEE
#ifndef USE_DEFAULT_TIMEOUT_CALLBACK
/**
 * @brief  Basic management of the timeout situation.
 * @param  None.
 * @retval sEE_FAIL.
 */
uint32_t sEE_TIMEOUT_UserCallback(void)
{
	/* Return with error code */
	return sEE_FAIL;
}
#endif
#endif /* USE_SEE */


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
