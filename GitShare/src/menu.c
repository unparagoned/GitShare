#include "menu.h"

#include "myMaths.h"

#define NOTOUCH  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6)
extern __IO uint32_t TimingDelay;
extern __IO uint16_t ADCConvertedValue;
extern __IO uint16_t ADCConvertedValuePC1;
extern uint8_t tempD;
extern unsigned int LCD_IDP;
extern __IO uint32_t DMAComplete;
extern uint8_t _it1;
extern uint8_t _it0;
extern uint16_t chargeHVal, chargeLVal, dischargeHVal, dischargeLVal;

extern uint16_t boostPeriod;
extern uint16_t boostOn;

extern uint16_t outputValue;

extern uint16_t TimFactor;
extern uint16_t TimFactorPeak;

uint8_t localInt;
uint16_t globalXPos;
uint16_t globalYPos;

void menuSelection(void *selection);

void menuCopy(menu *original, menu *copy);

void PWMAction(int16_t *adjustment);
void PWMSliderAction(uint16_t *yPos);
void GPIOAction(uint8_t * state);
void saveAction(uint8_t * ignore);
//functions should be defined in header file
//void loadAction(uint8_t * ignore);
void MYDEBUGAction(uint8_t * ignore);
void readingsAction(uint16_t *pin);
void analogAction(int16_t *adjustment);


uint8_t selectOptionSimple(uint16_t xPos, uint16_t yPos);
uint8_t selectOptionSlider(uint16_t xPos, uint16_t yPos);
uint8_t getTouch(uint16_t * xPos, uint16_t *yPos);

//decides whether the menu is open or not
uint8_t menuOpen=0;

int16_t sIncrease=1;
int16_t sDecrease=-1;

int16_t lIncrease=20;
int16_t lDecrease=-20;

uint8_t On=1;
uint8_t Off=0;

uint8_t zero=0;

uint16_t sliderValue=0;

uint8_t optionSelected;

uint16_t buttonPressLength=0;

uint8_t displayReadings=0;

uint16_t analogPC0=10;
uint16_t analogPC1=20;
//name, current value, menuNumber,number of options, options[8]
menu currentMenu;

//temo defined
menu boostMenu;
menu controlMenu;
menu vDropMenu;
menu periodMenu;
menu analogMenu;
menu GPIOMenu;
menu readingsMenu;
menu PWMMenu;
menu analogMenu;
menu dutyMenu;
menu analog1ActionMenu;
menu analog2ActionMenu;
menu pinRangeMenu;
menu lowPinMenu;
menu highPinMenu;
menu GPIOActionMenu;
menu voltageMenu;
menu chargeHMenu;
menu chargeLMenu;
menu dischargeHMenu;
menu dischargeLMenu;


//all menu names should be shorter than this
uint8_t actionName[]="This is the action Menu and output";


/*
 * 	uint8_t * menu name
	uint8_t numberOfOptions
	option the option, max of 8
	uint16_t selectionHistory this kind of keeps track of which pins/timer have been selected.
	uint8_t (*optionSelected)(uint16_t xPos, uint16_t yPos), function that acts on position
	uint8_t slider whether menu has a slider


	Option
	 * name of option,
 * function of option
 * option pointer
 * selection, for history
 */

menu topMenu={"Main Menu",8, {
								{"PWM",&menuSelection, &PWMMenu, nPWM},
								{"Analog",&menuSelection, &analogMenu, nAnalog},
								{"GPIO",&menuSelection, &GPIOMenu, nGPIO},
								{ "Readings",&menuSelection, &readingsMenu, nReadings},
								{ "Voltage",&menuSelection, &voltageMenu, nVoltage},
								{ "Save",&saveAction, &zero},
								{ "Load",&loadAction, &zero},
								{ "MYDEBUG",&MYDEBUGAction, &zero}
							},
			0, &selectOptionSimple, 0

			};

menu PWMMenu={"PWM Menu",3,	{
								{"Boost",&menuSelection, &boostMenu, nTimer1},
								{"Control",&menuSelection, &controlMenu, nTimer8},
								{"Vdrop", &menuSelection, &vDropMenu,nTimer2}
							},
							0, &selectOptionSimple, 0
			};


menu PWMSubMenu={"PWM Sub Menu",2, {
										{"Period",&menuSelection, &periodMenu, nPeriod},
										{"Duty",&menuSelection, &dutyMenu, nDuty},
									},
									0, &selectOptionSimple, 0
};

menu PWMActionMenu={"PWM action",3, {
								{"+1%",&PWMAction, &sIncrease, nButtonPress},
								{"-1%",&PWMAction, &sDecrease,nButtonPress},
								{"     +10%       ",&PWMSliderAction, &globalYPos, nSliderTouch}

							},
							0, &selectOptionSlider, 2
			};

menu analogMenu={"Analog Menu",2,	{
								{"1 out",&menuSelection, &analog1ActionMenu, nAnalog1ActionMenu},
								{"2 out",&menuSelection, &analog2ActionMenu, nAnalog2ActionMenu}
							},
							0, &selectOptionSimple, 0
			};

menu analogActionMenu={"Analog action",4, {
								{"+1%",&analogAction, &sIncrease, nButtonPress},
								{"-1%",&analogAction, &sDecrease, nButtonPress},
								{"+10%",&analogAction, &lIncrease, nButtonPress},
								{"-10%",&analogAction, &lDecrease, nButtonPress}
							},
							0, &selectOptionSimple, 0
			};

menu GPIOMenu={"Select Port",5, {
								{"A",&menuSelection, &pinRangeMenu, nPortA},
								{"B",&menuSelection, &pinRangeMenu, nPortB},
								{"C",&menuSelection, &pinRangeMenu, nPortC},
								{"D",&menuSelection, &pinRangeMenu, nPortD},
								{"E",&menuSelection, &pinRangeMenu, nPortE}
							},
			0, &selectOptionSimple, 0
			};

menu pinRangeMenu={"Select pin Range",2,{
										{"0-7",&menuSelection, &lowPinMenu, nLowPins},
										{"7-15",&menuSelection, &highPinMenu, nHighPins}
										},
										0, &selectOptionSimple, 0
};

menu lowPinMenu={"Select Pin",8, {
									{"0",&menuSelection, &GPIOActionMenu, 0},
									{"1",&menuSelection, &GPIOActionMenu, 1*100},
									{"2",&menuSelection, &GPIOActionMenu, 2*100},
									{"3",&menuSelection, &GPIOActionMenu, 3*100},
									{"4",&menuSelection, &GPIOActionMenu, 4*100},
									{"5",&menuSelection, &GPIOActionMenu, 5*100},
									{"6",&menuSelection, &GPIOActionMenu, 6*100},
									{"7",&menuSelection, &GPIOActionMenu, 7*100}
								},
0, &selectOptionSimple, 0
};

menu highPinMenu={"Select Pin",8, {
									{"8",&menuSelection, &GPIOActionMenu, 8*100},
									{"9",&menuSelection, &GPIOActionMenu, 9*100},
									{"10",&menuSelection, &GPIOActionMenu, 10*100},
									{"11",&menuSelection, &GPIOActionMenu, 11*100},
									{"12",&menuSelection, &GPIOActionMenu, 12*100},
									{"13",&menuSelection, &GPIOActionMenu, 13*100},
									{"14",&menuSelection, &GPIOActionMenu, 14*100},
									{"15",&menuSelection, &GPIOActionMenu, 15*100}
								},
0, &selectOptionSimple, 0
};

menu GPIOActionMenu={"Pin action",2, {
								{"On", &GPIOAction, &On},
								{"Off",&GPIOAction, &Off}
							},
							0, &selectOptionSimple, 0
			};

menu readingsMenu={"Readings",2, {
									{"PC0 Anal", &readingsAction, &analogPC0,nAnalogPC0},
									{"PC1 Anal", &readingsAction, &analogPC1,nAnalogPC1}

},	0, &selectOptionSimple, 0
			};


menu voltageMenu={"PWM Menu",4,	{
							{"Chg. H",&menuSelection, &chargeHMenu, nChargeH},
							{"Chg. L",&menuSelection, &chargeLMenu, nChargeL},
							{"Dis H", &menuSelection, &dischargeHMenu,nDischargeH},
							{"Dis L", &menuSelection, &dischargeLMenu,nDischargeL}
						},
						0, &selectOptionSimple, 0
		};


menu voltageActionMenu={"voltage action",3, {
							{"+1%",&PWMAction, &sIncrease, nButtonPress},
							{"-1%",&PWMAction, &sDecrease,nButtonPress},
							{"    range     ",&PWMSliderAction, &globalYPos, nSliderTouch}

						},
						0, &selectOptionSlider, 2
		};



void menuInit()
{
menuCopy(&PWMSubMenu, &boostMenu);
boostMenu.name="Boost Menu";

menuCopy(&PWMSubMenu, &controlMenu);
controlMenu.name="Control Menu";

menuCopy(&PWMSubMenu, &vDropMenu);
vDropMenu.name="Vdrop Menu";


menuCopy(&PWMActionMenu, &periodMenu);
periodMenu.name="Period Menu";

menuCopy(&PWMActionMenu, &dutyMenu);
dutyMenu.name="Duty Menu";

menuCopy(&PWMActionMenu, &chargeHMenu);
dutyMenu.name="Charge H Menu";

menuCopy(&PWMActionMenu, &chargeLMenu);
dutyMenu.name="Charge L Menu";

menuCopy(&PWMActionMenu, &dischargeHMenu);
dutyMenu.name="Discharge H Menu";

menuCopy(&PWMActionMenu, &dischargeLMenu);
dutyMenu.name="Discharge L Menu";


menuCopy(&analogActionMenu, &analog1ActionMenu);
analog1ActionMenu.name=" Analog 1 Action";

menuCopy(&analogActionMenu, &analog2ActionMenu);
analog2ActionMenu.name="Analog 2 Action ";

}



/*

menu PWMSubMenu={"PWM Option Menu",nPWMSubMenu, 0, 0,2,{"Period","Duty"}};

menu PWMActionMenu={"PWM Control Menu", nPWMActionMenu, 0, 1,4,{"Increase 1","Decrease 1","Increase 10", "Decrease 10"}};



menu analogMenu={"Analog Menu", nAnalogMenu,1,0,2,{"Out 1","Out 2"}};

menu analogSubMenu={"Analog Menu", nAnalogSubMenu,1,0,4,{"Increase 1","Decrease 1","Increase 10", "Decrease 10"}};

menu GPIOMenu={"GPIO Menu", 0,1,0,6,{"A","B","C","D","E","F"}};

menu GPIOSubMenu={"GPIO Sub Menu", 0,1,0,3,{"0-4","5-10","11-15"}};

menu GPIORangeMenu={"GPIO range Menu", 0,1,0,3,{"0-4","5-10","11-15"}};

menu GPIOLowPinMenu={"GPIO 0-5 Menu", 0,1,0,6,{"0","1","2","3","4","5"}};

menu GPIOMidPinMenu={"GPIO 6-10 Menu", 0,1,0,5,{"6","7","8","9","10"}};

menu GPIOHighPinMenu={"GPIO 6-10 Menu", 0,1,0,5,{"11","12","13","14","15"}};
*/

void delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

void startMenu()
{

	if(TimingDelay)
	{
		_it1=0;

		return;
	}
	if(menuOpen==1)
	{
		menuOption();
	}
	else
	{
	menuOpen=1;
	currentMenu=topMenu;
	sysDelay(200);
		//while(TimingDelay);
	}
	//delay(1000000);


}

//returns whether the menu is open or not
uint8_t menuInterface()
{
	if(!menuOpen)
	{
		return 0;
	}
	if(_it1)
	{
		menuOption();
		_it1=0;
		displayFrame();
	}
	if(_it0)
	{
		localInt=0;
		buttonPressLength=0;
	}
	if(localInt==1)
	{
		menuOption();
		displayFrame();
	}

	if(displayReadings)
	{
		uint16_t nil=0;
		readingsAction(&nil);
		displayFrame();
	}

	drawMenu();

	return 1;
}

extern uint16_t X,Y;
//on touch this determines which option was selected,
//should co-ords be passed or can they be gained automatically?
void menuOption()
{
	displayReadings=0;
	//this stops touches that are too close together
	if(TimingDelay)
	{
		return;
	}

	uint16_t x,y;

	//this default means the value will just return and not trigger any event.look below for the bits between 100&150
	x=148;
	y=148;

	//getTouch(&x,&y);
	getRelTouch(&x,&y);
	globalYPos=y;


			//line selected is really option selected

	uint16_t menuDetails;
	//menu case



	//exit menu
	if(x<100) //32
	{
		sysDelay(200);
		if(!localInt)
		{
		menuOpen=0;
		sysDelay(200);
		}
		return;
	}
	if(x<150) //48
	{
		return;
	}


optionSelected=currentMenu.optionSelected(x,y);
		//selectOptionSimple(x,y);

	//adds delay so touch doesn't go through multiple menus

//option selected is zero indexed, and need to check if there are options
if(optionSelected>currentMenu.numberOfOptions-1 || !currentMenu.numberOfOptions)
{
	//no option selected
	//optionSelected=0; //prevents weird stuff, should use more elegant solution
	return;
}

/*this checks to see if it's an button
 * It also checks to see if that option wants constant updating without requiring a separate touch
 * This usually allows length of time functionality
 * LocalInt allows program to keep poling and updating this kind of button.
 */
if(currentMenu.options[optionSelected].selection==nButtonPress)
{
	//this records how long button has been pressed.
	buttonPressLength++;
	localInt=1;
}

/*
 * This allows constant poling of slider
 */
if(currentMenu.options[optionSelected].selection/1000==nSliderTouch/1000)
{
	//this records how long button has been pressed.
	//buttonPressLength++;
	localInt=1;
}

//selection history keeps track of options, and pins or timers or functions to be used.
//selects function of option, then passes it the variable of the option.
uint16_t selectionHistory=currentMenu.selectionHistory+currentMenu.options[optionSelected].selection;
currentMenu.options[optionSelected].function(currentMenu.options[optionSelected].optionPoint);
if(localInt==0)
{
currentMenu.selectionHistory=selectionHistory;
}
sysDelay(200);
}

//returns whether screen is being touched or not
uint8_t getTouch(uint16_t * xPos, uint16_t *yPos)
{
	//convert touch to line
		float X1,Y1,hh;
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
				t=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
			}while(!t&&count<10);



			if(count==10)
			{

			}
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

				if(LCD_IDP==0)
				{
					x=479-((15*(x1-180))>>7);
					y=799-((25*(y1-180))>>7);
				}
				if(1)
				{
					x=239-x;
				}

				//safety balance incase touch is different on each screen.
				//Using 238 instead of 239 is because circle of radius 1 is drawn.

				//is the screen being touched
				uint8_t screenTouch=1;
				if(LCD_IDP!=0)
				{
				if(x>240 ||y>320)
				{
					screenTouch=0;
				}
				x=(x>238?238:x);
				y=(y>318?318:y);
				}
				*xPos=x;
				*yPos=y;

				return screenTouch;

}


/*returns whether screen is being touched or not. It returns the accurate values rather than pixel position
 * This is more accurate and better for different resolutions. Conversion to actual pixel position doesn't need to be done straight away.
 * range 0-1024(or 1000)
 */
uint8_t getRelTouch(uint16_t * xPos, uint16_t *yPos)
{
	//convert touch to line
	float X1,Y1,hh;
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
		t=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
	}while(!t&&count<10);



	if(count==10)
	{

	}
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

	uint16_t x=0;
	uint16_t y=0;
uint8_t tempCounter;
for(tempCounter=0;tempCounter<10;tempCounter++)
{
	x+=databuffer[1][tempCounter]/10;
	y+=databuffer[0][tempCounter]/10;

	//screen hasn't been touched, it's not a proper touch
	if(databuffer[1][tempCounter]>4096||databuffer[0][tempCounter]>4096)
	{

		return 0;
	}
}
//x/=count;
//y/=count;


y=4096-y;
	//x=y1;
//	y=4096-x1;



x=x>512?x-512:0;
y=y>512?y-512:0;


	x=(10*x)>>5;
	y=(10*y)>>5;
	if(LCD_IDP==0)
	{
	uint16_t temp=x;
	x=1024-y;
	y=1024-temp;
	}


	//safety balance incase touch is different on each screen.
	//Using 238 instead of 239 is because circle of radius 1 is drawn.

	//is the screen being touched
	uint8_t screenTouch=1;

		if(x>1024 ||y>1024)
		{
			screenTouch=0;
		}


	*xPos=x;
	*yPos=y;

	return screenTouch;

}

void drawMenu()
{
	 while(DMAComplete == 0)
	   {
	   }
	//displays the current menu
	LCD_DisplayStringLine(LINE(0), currentMenu.name);

	int i=0;
	for(i=0; i<currentMenu.numberOfOptions;i++)
	{

		if(i==currentMenu.slider && currentMenu.slider)
		{
			LCD_DisplayStringLineSlider(LINE(2*i+2), currentMenu.options[i].name,sliderValue);

		}
		else
		{
			if(i<4)
			{
				LCD_DisplayStringLine(LINE(2*i+2), currentMenu.options[i].name);
			}
			else
			{
				LCD_DisplayStringLineFromCentre(LINE(2*(i-4)+2), currentMenu.options[i].name);
			}
		}

	}

}
void menuSelection(void *selection)
{
 //  menu *newMenu=selection;



	currentMenu=*(menu*)selection;

	sysDelay(200);
}

//TODO Analog
void analogAction(int16_t *adjustment)
{

	uint16_t topMenuType=currentMenu.selectionHistory/10;



	uint16_t lengthAdj=1;
	if(buttonPressLength>10)
	{
		lengthAdj=10;
	}
	if(buttonPressLength>20)
	{
		lengthAdj=100;
	}

	if(topMenuType==nAnalog1ActionMenu/10)
	{
	DAC->DHR12R1+=*adjustment*lengthAdj;
	}

	else if(topMenuType==nAnalog2ActionMenu/10)
	{
		//DAC->DHR12R2+=*adjustment*lengthAdj; don't fix this directly, if it is stuck on it will overheat components made for pulsing only.
		outputValue+=*adjustment*lengthAdj;

	}

	DAC->SWTRIGR=3;
}

void PWMAction(int16_t * adjustment)
{
	TIM_TypeDef * TIMx;

	uint16_t topMenuType=currentMenu.selectionHistory%10;


	//use selection history to select correct function/pin
	uint16_t lengthAdj=1;
	if(buttonPressLength>10)
	{
		lengthAdj=10;
	}
	if(buttonPressLength>20)
	{
		lengthAdj=100;
	}
	if(buttonPressLength>30)
	{
		lengthAdj=1000;
	}

	if(topMenuType==nPWM)
	{

		if((currentMenu.selectionHistory%100)/10==nTimer1/10)
		{
			TIMx=TIM1;
		}
		if((currentMenu.selectionHistory%100)/10==nTimer8/10)
		{
			TIMx=TIM8;
		}
		if((currentMenu.selectionHistory%100)/10==nTimer2/10)
		{
			TIMx=TIM2;
		}


		switch(currentMenu.selectionHistory/100)
		{

		case nPeriod/100:
		{
			//change period
			if((int16_t)TIMx->ARR+*adjustment*lengthAdj<0)
			{
				TIMx->ARR=50;
			}
			else
			{
				TIMx->ARR+=*adjustment*lengthAdj;
			}
			sprintf(actionName,"period is %d",TIMx->ARR);
			currentMenu.name=actionName;
			//sliderValue=TIM1->ARR/205;
			sliderValue=TIMx->ARR>>6;
			break;
		}

		case nDuty/100:
		{
			//change duty
			if((int16_t)TIMx->CCR3+*adjustment*lengthAdj<0)
			{
				TIMx->CCR3=1;
			}
			else
			{
				TIMx->CCR3+=*adjustment*lengthAdj;
			}
			sprintf(actionName,"Duty is %d",TIMx->CCR3);
			currentMenu.name=actionName;
			//sliderValue=TIM1->CCR3/205;
			sliderValue=TIMx->CCR3>>6;
			break;
		}
		}
	}

	if(topMenuType==nVoltage)
	{
		switch(currentMenu.selectionHistory/10)
		{

		case nChargeH/10:
		{
			chargeHVal+=*adjustment*lengthAdj;
			sprintf(actionName,"Duty is %d",chargeHVal);
			currentMenu.name=actionName;
			break;
		}
		case nChargeL/10:
		{
			chargeLVal+=*adjustment*lengthAdj;
			sprintf(actionName,"Duty is %d",chargeLVal);
			currentMenu.name=actionName;
			break;
		}
		case nDischargeH/10:
		{
			dischargeHVal+=*adjustment*lengthAdj;
			sprintf(actionName,"Duty is %d",dischargeHVal);
			currentMenu.name=actionName;
			break;
		}
		case nDischargeL/10:
		{
			dischargeLVal+=*adjustment*lengthAdj;
			sprintf(actionName,"Duty is %d",dischargeLVal);
			currentMenu.name=actionName;
			break;
		}
		}



	}

}

//temporarily controls volume & output peak and frequency
//be careful not to adjust the first pwm period with adjust sound out.
void PWMSliderAction(uint16_t *yPos)
{

	TIM_TypeDef * TIMx;

	if((currentMenu.selectionHistory%100)/10==nTimer1/10)
	{
		TIMx=TIM1;
	}
	if((currentMenu.selectionHistory%100)/10==nTimer8/10)
	{
		TIMx=TIM8;
	}
	if((currentMenu.selectionHistory%100)/10==nTimer2/10)
	{
		TIMx=TIM2;
	}

	switch((currentMenu.selectionHistory%1000)/100)
	{
		case nPeriod/100:
		{
				//TIMx->ARR=*yPos*205;

				TIMx->ARR=*yPos<<6;
				sprintf(actionName,"period is %d",TIMx->ARR);
				currentMenu.name=actionName;
				sliderValue=*yPos;
				//output stuff
				TimFactor=*yPos/8+1;
				break;

		}
		case nDuty/100:
		{
			//TIMx->CCR3=*yPos*205;
			if((*yPos<<6)>TIMx->ARR)
			{
				*yPos=TIMx->ARR>>6;
			}
			if((*yPos<10))
			{
				*yPos=1;
			}
			TIMx->CCR3=*yPos<<6;
			sprintf(actionName,"Duty is %d",TIMx->CCR3);
			currentMenu.name=actionName;
			sliderValue=*yPos;
			TimFactorPeak=*yPos/10+1;
			if(TimFactorPeak>TimFactor/2)
			{
				TimFactorPeak=TimFactor/2+1;
			}


			break;
		}

		}
	//this is temporary, sound should have it's own menu to control it.
	soundVolume=*yPos;
}

void voltageAction(uint16_t *adjustment);
void voltageSliderAction(uint16_t *yPos);



void GPIOAction(uint8_t * state)
{
	GPIO_TypeDef *GPIOPort;
	switch((currentMenu.selectionHistory%100)/10)
	{
		case nPortA/10:
		GPIOPort=GPIOA;
		break;
		case nPortB/10:
		GPIOPort=GPIOB;
		break;
		case nPortC/10:
		GPIOPort=GPIOC;
		break;
		case nPortD/10:
		GPIOPort=GPIOE;
		break;
		case nPortE/10:
		GPIOPort=GPIOE;
		break;
	}

	if(*state==On)
	{
		 GPIO_SetBits(GPIOPort, 1<<(currentMenu.selectionHistory/100) );
	}
	else if(*state==Off)
	{
		GPIO_ResetBits(GPIOPort, 1<<(currentMenu.selectionHistory/100) );
	}

}

void saveAction(uint8_t *ignore)
{
	boostPeriod=TIM1->ARR;
	boostOn=TIM1->CCR3;
	uint16_t readArray[]={boostPeriod,boostOn,DAC->DHR12R1,outputValue };
	saveVariablesToFlash(readArray,4);

}

void loadAction(uint8_t *ignore)
{
	uint16_t readArray[4];
	initFlashVariables(readArray,4);

	TIM1->ARR=readArray[0];
	TIM1->CCR3=readArray[1];
	DAC->DHR12R1=readArray[2];
	outputValue=readArray[3];


}

//simply toggles the MYDEBUG state, but it goes back to MYARM, there may be lots of states.
void MYDEBUGAction(uint8_t *ignore)
{
	if(thisState==MYARM)
	{
		thisState=MYDEBUG;
	}
	else
	{
		thisState=MYARM;
	}
}

void readingsAction(uint16_t *pin)
{



	displayReadings=1;
	static uint16_t selection=0;
	uint16_t tempVar;
	if(*pin)
	{
		selection=*pin;
	}
	//this approximately converts it to mV
	if(selection==nAnalogPC0)
	{
	tempVar=ADCConvertedValue;
	}
	if(selection==nAnalogPC1)
	{
	tempVar=ADCConvertedValuePC1;
	}
	sprintf(actionName,"%d & %d",tempVar, round(tempVar,100));
	//sprintf(actionName,"Reading is %d",ADCConvertedValue);
	currentMenu.name=actionName;




}

//remember to update this copy function whenever menu changes
void menuCopy(menu *original, menu *copy)
{
	int i;
	for(i=0;i<20;i++)
	{
	//copy->name[i]=original->name[i];
	}
	copy->name=original->name;
	copy->numberOfOptions=original->numberOfOptions;
	copy->optionSelected=original->optionSelected;

	for(i=0;i<8;i++)
	{
	copy->options[i]=original->options[i];
	}
	copy->selectionHistory=original->selectionHistory;

	copy->slider=original->slider;
}

uint8_t selectOptionSimple(uint16_t xPos, uint16_t yPos)
{
	//this is based on fixed resolution
	/*
	uint8_t optionTouched=(xPos+12)/48;

	if(yPos<160)
	{
		optionTouched+=4;
	}

	optionTouched--;
	return optionTouched;
	*/
	//try and implement on a relative touch system.
	uint8_t optionTouched=(xPos+50)/200;

	if(yPos<512)
	{
		optionTouched+=4;
	}

	optionTouched--;
	return optionTouched;
}

uint8_t selectOptionSlider(uint16_t xPos, uint16_t yPos)
{
	//old pixel based
	/*
	uint8_t optionTouched=(xPos+12)/48;

	if(yPos<160)
	{
		optionTouched+=4;
	}

	optionTouched--;

	//this means slider can't be at the top
	if(currentMenu.slider)
	{
		if(optionTouched==(currentMenu.slider+4))
		{
			optionTouched-=4;
		}
	}
	return optionTouched;
	*/
	//relative position based
	uint8_t optionTouched=(xPos+50)/200;

	if(yPos<500)
	{
		optionTouched+=4;
	}

		optionTouched--;

	//this means slider can't be at the top
	if(currentMenu.slider)
	{
		if(optionTouched==(currentMenu.slider+4))
		{
			optionTouched-=4;
		}
	}



	return optionTouched;
}


