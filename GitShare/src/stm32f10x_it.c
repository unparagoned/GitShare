/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t DMAComplete = 1;
__IO uint8_t largeTransfer=0;
extern uint16_t numberOfCycles;
uint8_t transferCount=0;

uint8_t tempVStatus=0;
uint16_t capture=0;
extern uint8_t _it1;
extern uint8_t _it0;
extern __IO uint16_t ADCConvertedValue;
extern __IO uint16_t ADCConvertedValuePC1;
__IO uint8_t SDIOEnd=0;
extern uint16_t outputPeriods;

uint16_t outputChannel=0; //0 means no output, other numbers refer to channel
uint16_t outputVoltage=0; //use to turn off vout before the channel
uint16_t outputValue=0;
int16_t step=10;

uint8_t tempVarDelete=0;

uint16_t TimFactor=63;

uint16_t TimFactorPeak=33;

uint16_t periodMultiplier=0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
	/* Decrement the TimingDelay variable */
	Decrement_TimingDelay();


}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles External interrupt Line 3 request.
  * @param  None
  * @retval : None
  */
void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
    //DownFunc();
    /* Clear the EXTI Line 3 */
    EXTI_ClearITPendingBit(EXTI_Line3);
  }
}

/**
  * @brief  This function handles DMA1 Channel 3 interrupt request.
  * @param  None
  * @retval : None
  */
void DMA1_Channel3_IRQHandler(void)
{
  DMA_InitTypeDef  DMA_InitStructure;



    DMA_Cmd(DMA1_Channel3, DISABLE);
    //LCD_WriteReg(LCD_REG_3, 0x1018);



  DMA_ClearITPendingBit(DMA1_IT_GL3);
 // DMAComplete = 1;
}

void DMA2_Channel4_5_IRQHandler(void)
{
	transferCount++;
	if(transferCount>=numberOfCycles)
	{
	 DMA_Cmd(DMA2_Channel4, DISABLE);
	 DMA_ClearITPendingBit(DMA2_IT_TC4);
	 transferCount=0;
	 DMAComplete=1;
	}

	DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);
	DMA_ClearFlag(DMA2_FLAG_TC5 | DMA2_FLAG_TE5 | DMA2_FLAG_HT5 | DMA2_FLAG_GL5);

}
/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval : None
  */
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
	  //set appropriate interrupt, reset other
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6))
	{
		_it0=1;
		_it1=0;
	}
	else
	{
		_it1=1;
		_it0=0;
	}

	//startMenu();
    EXTI_ClearITPendingBit(EXTI_Line6);
  }
}

/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval : None
  */
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line15) != RESET)
  {
   // UpFunc();
    /* Clear the EXTI Line 15 */
    EXTI_ClearITPendingBit(EXTI_Line15);
  }
}

/**
  * @brief  This function handles SDIO global interrupt request.
  * @param  None
  * @retval : None
  */
void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
  SDIOEnd=1;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

void	TIM1_UP_IRQHandler(void)
{
	 TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	// tempVarDelete++;
	 //just see if interupts work as planned


 	updateSoundPWM();
}




/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/


void TIM5_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);

	capture = TIM_GetCounter(TIM5);
	if(periodMultiplier>0)
	{
		periodMultiplier--;
		return;
	}

	//capture + 10000);
	//	 TIM_SetAutoreload(TIM5, (SystemCoreClock / 17570 )/2 -1);


	//made global so can be modified by menu
	//  uint16_t TimFactor=ADCConvertedValuePC1>>6;
	//uint16_t TimFactor=63;
	//uint16_t TimFactorPeak=ADCConvertedValue>>6;
	//uint16_t TimFactorPeak=33;

	uint16_t preOpenFactor=1;
	//uint16_t preOpenFactor=0;

	//turns off output
	if(outputChannel)
	{
		if(outputVoltage)
		{
			//output off - delay so there isn't a voltage spike when output is in tristate
			GPIO_ResetBits(GPIOA, GPIO_Pin_7);
			DAC->DHR12R2=0;

			DAC->SWTRIGR=3;
			TIM_SetAutoreload(TIM5, preOpenFactor*(SystemCoreClock / 17570 )/8 -1);//the stabilizing capacitors create a large delay, this is the minimum time required to prevent voltage spikes
			outputVoltage=0;
			return;
		}
		//output off
		//	GPIO_ResetBits(GPIOA, GPIO_Pin_7);

		GPIO_ResetBits(GPIOA,  GPIO_Pin_2 | GPIO_Pin_3);
		outputChannel=0;
		// TIM_SetCompare1(TIM5, capture+500);
		TIM_SetAutoreload(TIM5, TimFactor*(SystemCoreClock / 17570 )/8 -1);
		periodMultiplier=5; //makes peak time 5 times longer - testing, implement properly later
		outputPeriods++;

		//changes output voltage disabled
		//setOutputVoltage();


		//set to zero so output cap doesn't charge up. Set output voltage just before output turns on.
		//not set to zero atm, so there aren't large spikes on output. ? set to zero to test
		//DAC->DHR12R2=0;

		//DAC->SWTRIGR=3;


	}
	else
	{
		if(!GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_7))
		{

			//this is the pre-open stuff, gets over the inital untable bit.

			GPIO_SetBits(GPIOA, GPIO_Pin_7);

			//set to zero now depending on design
			//DAC->DHR12R2=outputValue;
			DAC->DHR12R2=0;

			DAC->SWTRIGR=3;

			TIM_SetAutoreload(TIM5, preOpenFactor*(SystemCoreClock / 17570 )/8 -1);





		}
		else
		{
			//output on
			GPIO_ResetBits(GPIOA, GPIO_Pin_7);


			if(tempVStatus)
			{
				if(thisState==MYARM)
				{
					outputValue+=step;
					if(outputValue>4080)
					{
						step*=-1;
					}
					if(outputValue<(step*-1)) //is this right, shouldn't it just be compared to steps? not -steps?
					{
						step*=-1;
					}
				}
				else if(thisState==MYDEBUG)
				{
					//output value is set by analog control automatically no need to put it in here.
				}


				DAC->DHR12R2=outputValue;

				DAC->SWTRIGR=3;
				tempVStatus=0;
				//GPIO_ResetBits(GPIOA, GPIO_Pin_3);
				GPIO_SetBits(GPIOA, GPIO_Pin_3);
				outputChannel=3;
				outputVoltage=1;
			}
			else
			{
				if(thisState==MYARM)
				{
					DAC->DHR12R2=3300-outputValue;
				}
				else if(thisState==MYDEBUG)
				{
					DAC->DHR12R2=outputValue;
					//output value is set by analog control automatically no need to put it in here.
				}
				DAC->SWTRIGR=3;
				tempVStatus=1;
				//GPIO_ResetBits(GPIOA, GPIO_Pin_2);
				GPIO_SetBits(GPIOA, GPIO_Pin_2);
				outputChannel=2;
				outputVoltage=1;
			}


			//GPIO_ResetBits(GPIOA, GPIO_Pin_2|GPIO_Pin_3);

			//TIM_SetCompare1(TIM5, capture+200);
			TIM_SetAutoreload(TIM5, TimFactorPeak*(SystemCoreClock / 17570 )/8 -1);

			//disables outputs. These control the voltage, really should have a better method, linking period or something.
			TIM_CtrlPWMOutputs(TIM8, ENABLE);
			TIM_CtrlPWMOutputs(TIM2, ENABLE);


		}
	}
}

/**
  * @}
  */ 
