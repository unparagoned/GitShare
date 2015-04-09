/**
  ******************************************************************************
  * @file    stm3210e_eval_lcd.c
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file includes the LCD driver for AM-240320L8TNQW00H 
  *          (LCD_ILI9320) and AM-240320LDTNQW00H (LCD_SPFD5408B) Liquid Crystal
  *          Display Module of STM3210E-EVAL board.
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
#include "stm3210e_eval_lcd.h"
#include "fonts.h"


/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup STM3210E_EVAL
  * @{
  */
    
/** @defgroup STM3210E_EVAL_LCD 
  * @brief This file includes the LCD driver for AM-240320L8TNQW00H 
  *        (LCD_ILI9320) and AM-240320LDTNQW00H (LCD_SPFD5408B) Liquid Crystal
  *        Display Module of STM3210E-EVAL board.
  * @{
  */ 

/** @defgroup STM3210E_EVAL_LCD_Private_TypesDefinitions
  * @{
  */ 
typedef struct
{
  __IO uint16_t LCD_REG;
  __IO uint16_t LCD_RAM;
} LCD_TypeDef;
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LCD_Private_Defines
  * @{
  */
/* Note: LCD /CS is CE4 - Bank 4 of NOR/SRAM Bank 1~4 */
#define LCD_BASE           (uint32_t)(0x60000000)
#define LCD                ((LCD_TypeDef *) LCD_BASE)
#define MAX_POLY_CORNERS   200
#define POLY_Y(Z)          ((int32_t)((Points + Z)->X))
#define POLY_X(Z)          ((int32_t)((Points + Z)->Y))

#define Bank1_LCD_D    ((uint32_t)0x60020000)    //disp Data ADDR
#define Bank1_LCD_C    ((uint32_t)0x60000000)	 //disp Reg ADDR


extern uint8_t TCS;
/**
  * @}
  */ 

/** @defgroup STM3210E_EVAL_LCD_Private_Macros
  * @{
  */
#define ABS(X)  ((X) > 0 ? (X) : -(X))    
/**
  * @}
  */ 
  
/** @defgroup STM3210E_EVAL_LCD_Private_Variables
  * @{
  */ 
static sFONT *LCD_Currentfonts;
/* Global variables to set the written text color */
static  __IO uint16_t TextColor = 0x0000, BackColor = 0xFFFF;
unsigned int LCD_IDP;
  
uint16_t LCDSizeX;
uint16_t LCDSizeY;
/**
  * @}
  */ 


/** @defgroup STM3210E_EVAL_LCD_Private_FunctionPrototypes
  * @{
  */ 
#ifndef USE_Delay
static void delay(vu32 nCount);
#endif /* USE_Delay*/
static void PutPixel(int16_t x, int16_t y);
static void LCD_PolyLineRelativeClosed(pPoint Points, uint16_t PointCount, uint16_t Closed);
/**
  * @}
  */ 

void LCD_Write_COM(uint8_t reg)
{
	//LCD_WriteReg(reg, 0x0);
	LCD->LCD_REG = reg;
	//Delay(500);
}

void LCD_Write_DATA(uint16_t data)
{
	LCD_WriteRAM(data);
	//Delay(500);
}

/** @defgroup STM3210E_EVAL_LCD_Private_Functions
  * @{
  */ 

/**
  * @brief  DeInitializes the LCD.
  * @param  None
  * @retval None
  */
void LCD_DeInit(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;

  /*!< LCD Display Off */
  LCD_DisplayOff();

  /* BANK 4 (of NOR/SRAM Bank 1~4) is disabled */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
  
  /*!< LCD_SPI DeInit */
  FSMC_NORSRAMDeInit(FSMC_Bank1_NORSRAM4);
   
  /* Set PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
     PD.10(D15), PD.14(D0), PD.15(D1) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
     PE.14(D11), PE.15(D12) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  /* Set PF.00(A0 (RS)) as alternate function push pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  /* Set PG.12(NE4 (LCD/CS)) as alternate function push pull - CE3(LCD /CS) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOG, &GPIO_InitStructure); 
}

/**
  * @brief  Initializes the LCD.
  * @param  None
  * @retval None
  */

void lcd_rst()
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    Delay(0x1FFFFf);
    GPIO_SetBits(GPIOE, GPIO_Pin_1 );
	Delay(0xFFFf);
}



void STM3210E_LCD_Init(void)
{


	uint32_t color1=0;

	Delay(200);
	uint16_t displayColor=0;


	//LCD_IDP=0X9325;
	//temp alternative
	//LCD_IDP=0x10001;
	//TODO are the device codes being read properly, will I need to reset it like previously.
	lcd_rst();	 	//  TFT 复位操作
	Delay(200);
	LCD_IDP=LCD_ReadReg(0x00);
	//LCD_IDP=0x0;
	LCDSizeX=240;
	LCDSizeY=320;
	if((LCD_IDP==0X9325)||(LCD_IDP==0X4532))
	{

		Delay(200);

		//############# void Power_Set(void) ################//
		LCD_WR_CMD(0x0000,0x0001);
		Delay(10);

		LCD_WR_CMD(0x0015,0x0030);
		LCD_WR_CMD(0x0011,0x0040);
		LCD_WR_CMD(0x0010,0x1628);
		LCD_WR_CMD(0x0012,0x0000);
		LCD_WR_CMD(0x0013,0x104d);
		Delay(10);
		LCD_WR_CMD(0x0012,0x0010);
		Delay(10);
		LCD_WR_CMD(0x0010,0x2620);
		LCD_WR_CMD(0x0013,0x344d); //304d
		Delay(10);

		LCD_WR_CMD(0x0001,0x0100);
		LCD_WR_CMD(0x0002,0x0300);
		LCD_WR_CMD(0x0003,0x1008); //1030
		LCD_WR_CMD(0x0008,0x0604);
		LCD_WR_CMD(0x0009,0x0000);
		LCD_WR_CMD(0x000A,0x0008);

		LCD_WR_CMD(0x0041,0x0002);
		LCD_WR_CMD(0x0060,0x2700);
		LCD_WR_CMD(0x0061,0x0001);
		LCD_WR_CMD(0x0090,0x0182);
		LCD_WR_CMD(0x0093,0x0001);
		LCD_WR_CMD(0x00a3,0x0010);
		Delay(10);

		//################# void Gamma_Set(void) ####################//
		LCD_WR_CMD(0x30,0x0000);
		LCD_WR_CMD(0x31,0x0502);
		LCD_WR_CMD(0x32,0x0307);
		LCD_WR_CMD(0x33,0x0305);
		LCD_WR_CMD(0x34,0x0004);
		LCD_WR_CMD(0x35,0x0402);
		LCD_WR_CMD(0x36,0x0707);
		LCD_WR_CMD(0x37,0x0503);
		LCD_WR_CMD(0x38,0x1505);
		LCD_WR_CMD(0x39,0x1505);
		Delay(10);

		//################## void Display_ON(void) ####################//
		LCD_WR_CMD(0x0007,0x0001);
		Delay(10);
		LCD_WR_CMD(0x0007,0x0021);
		LCD_WR_CMD(0x0007,0x0023);
		Delay(10);
		LCD_WR_CMD(0x0007,0x0033);
		Delay(10);
		LCD_WR_CMD(0x0007,0x0133);






		//ini();

		LCD_WR_CMD(32, 0);
		LCD_WR_CMD(33, 0x013F);
		*(__IO uint16_t *) (Bank1_LCD_C)= 34;
		for(color1=0;color1<76800;color1++)
		{
			LCD_WR_Data(0xffff);
		}
		color1=0;
		//while(1);

	}

	 else if(LCD_IDP==0x9320 || LCD_IDP==0x9300)
	  {
		 TCS=(1<<4);
	    LCD_WriteReg(0x00,0x0000);
		LCD_WriteReg(0x01,0x0100);	/* Driver Output Contral */
		LCD_WriteReg(0x02,0x0700);	/* LCD Driver Waveform Contral */
		LCD_WriteReg(0x03,0x1008);	/* Entry Mode Set */

		LCD_WriteReg(0x04,0x0000);	/* Scalling Contral */
	    LCD_WriteReg(0x08,0x0202);	/* Display Contral */
		LCD_WriteReg(0x09,0x0000);	/* Display Contral 3.(0x0000) */
		LCD_WriteReg(0x0a,0x0000);	/* Frame Cycle Contal.(0x0000) */
	    LCD_WriteReg(0x0c,(1<<0));	/* Extern Display Interface Contral */
		LCD_WriteReg(0x0d,0x0000);	/* Frame Maker Position */
		LCD_WriteReg(0x0f,0x0000);	/* Extern Display Interface Contral 2. */

	    Delay(10);  /* delay 100 ms */
		LCD_WriteReg(0x07,0x0101);	/* Display Contral */
	    Delay(10);  /* delay 100 ms */

		LCD_WriteReg(0x10,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));	/* Power Control 1.(0x16b0)	*/
		LCD_WriteReg(0x11,0x0007);								/* Power Control 2 */
		LCD_WriteReg(0x12,(1<<8)|(1<<4)|(0<<0));				/* Power Control 3.(0x0138)	*/
		LCD_WriteReg(0x13,0x0b00);								/* Power Control 4 */
		LCD_WriteReg(0x29,0x0000);								/* Power Control 7 */

		LCD_WriteReg(0x2b,(1<<14)|(1<<4));

		LCD_WriteReg(0x50,0);       /* Set X Start */
		LCD_WriteReg(0x51,239);	    /* Set X End */
		LCD_WriteReg(0x52,0);	    /* Set Y Start */
		LCD_WriteReg(0x53,319);	    /* Set Y End */

		LCD_WriteReg(0x60,0x2700);	/* Driver Output Control */
		LCD_WriteReg(0x61,0x0001);	/* Driver Output Control */
		LCD_WriteReg(0x6a,0x0000);	/* Vertical Srcoll Control */

		LCD_WriteReg(0x80,0x0000);	/* Display Position? Partial Display 1 */
		LCD_WriteReg(0x81,0x0000);	/* RAM Address Start? Partial Display 1 */
		LCD_WriteReg(0x82,0x0000);	/* RAM Address End-Partial Display 1 */
		LCD_WriteReg(0x83,0x0000);	/* Displsy Position? Partial Display 2 */
		LCD_WriteReg(0x84,0x0000);	/* RAM Address Start? Partial Display 2 */
		LCD_WriteReg(0x85,0x0000);	/* RAM Address End? Partial Display 2 */

	    LCD_WriteReg(0x90,(0<<7)|(16<<0));	/* Frame Cycle Contral.(0x0013)	*/
		LCD_WriteReg(0x92,0x0000);	/* Panel Interface Contral 2.(0x0000) */
		LCD_WriteReg(0x93,0x0001);	/* Panel Interface Contral 3. */
	    LCD_WriteReg(0x95,0x0110);	/* Frame Cycle Contral.(0x0110)	*/
		LCD_WriteReg(0x97,(0<<8));
		LCD_WriteReg(0x98,0x0000);	/* Frame Cycle Contral */

	    LCD_WriteReg(0x07,0x0173);
	  }
	//3.2" sd1289
	else if(LCD_IDP==0x8989)
	{
		LCD_WR_CMD(0x0000,0x0001);    delay(1);  //打开晶振
		LCD_WR_CMD(0x0003,0xA8A4);    delay(1);   //0xA8A4
		LCD_WR_CMD(0x000C,0x0000);    delay(1);
		LCD_WR_CMD(0x000D,0x080C);    delay(1);
		LCD_WR_CMD(0x000E,0x2B00);    delay(1);
		LCD_WR_CMD(0x001E,0x00B7);    delay(1);
		LCD_WR_CMD(0x0001,0x2B3F);    delay(1);   //驱动输出控制320*240  0x6B3F
		LCD_WR_CMD(0x0002,0x0600);    delay(1);
		LCD_WR_CMD(0x0010,0x0000);    delay(1);
		LCD_WR_CMD(0x0011,0x6048);    delay(1);        //0x4030           //定义数据格式  16位色
		LCD_WR_CMD(0x0005,0x0000);    delay(1);
		LCD_WR_CMD(0x0006,0x0000);    delay(1);
		LCD_WR_CMD(0x0016,0xEF1C);    delay(1);
		LCD_WR_CMD(0x0017,0x0003);    delay(1);
		LCD_WR_CMD(0x0007,0x0233);    delay(1);        //0x0233
		LCD_WR_CMD(0x000B,0x0000);    delay(1);
		LCD_WR_CMD(0x000F,0x0000);    delay(1);        //扫描开始地址
		LCD_WR_CMD(0x0041,0x0000);    delay(1);
		LCD_WR_CMD(0x0042,0x0000);    delay(1);
		LCD_WR_CMD(0x0048,0x0000);    delay(1);
		LCD_WR_CMD(0x0049,0x013F);    delay(1);
		LCD_WR_CMD(0x004A,0x0000);    delay(1);
		LCD_WR_CMD(0x004B,0x0000);    delay(1);
		LCD_WR_CMD(0x0044,0xEF00);    delay(1);
		LCD_WR_CMD(0x0045,0x0000);    delay(1);
		LCD_WR_CMD(0x0046,0x013F);    delay(1);
		LCD_WR_CMD(0x0030,0x0707);    delay(1);
		LCD_WR_CMD(0x0031,0x0204);    delay(1);
		LCD_WR_CMD(0x0032,0x0204);    delay(1);
		LCD_WR_CMD(0x0033,0x0502);    delay(1);
		LCD_WR_CMD(0x0034,0x0507);    delay(1);
		LCD_WR_CMD(0x0035,0x0204);    delay(1);
		LCD_WR_CMD(0x0036,0x0204);    delay(1);
		LCD_WR_CMD(0x0037,0x0502);    delay(1);
		LCD_WR_CMD(0x003A,0x0302);    delay(1);
		LCD_WR_CMD(0x003B,0x0302);    delay(1);
		LCD_WR_CMD(0x0023,0x0000);    delay(1);
		LCD_WR_CMD(0x0024,0x0000);    delay(1);
		LCD_WR_CMD(0x0025,0x8000);    delay(1);
		LCD_WR_CMD(0x004f,0);        //行首址0
		LCD_WR_CMD(0x004e,0);        //列首址0
		*(__IO uint16_t *) (Bank1_LCD_C)=0x22;



		*(__IO uint16_t *) (Bank1_LCD_C)= 34;
		for(color1=0;color1<76800;color1++)
		{
			LCD_WR_Data(0xffff);
		}
		color1=0;


		 displayColor=0;
		 LCD_Write_COM(0x22);
		for(color1=0;color1<380000;color1++)
		{
			LCD_WR_Data(displayColor);
			if(color1%800==0)
			{
				displayColor++;

			}

		}

		LCD_SetCursor(150,100);

		LCD_Write_COM(0x22);
			for(color1=0;color1<380000;color1++)
			{
				LCD_WR_Data(color1);

				if(color1%200==0)
				{
					color1=color1+1;
				}
			}
	}
	else if(LCD_IDP==0)
	{
		LCDSizeX=480;
		LCDSizeY=800;

		//LCD_Write_COM(0x01); //soft reset
	//	delay(10);

				LCD_Write_COM(0xE2);		//PLL multiplier, set PLL clock to 120M
				LCD_Write_DATA(0x1E);	    //N=0x36 for 6.5M, 0x23 for 10M crystal
				LCD_Write_DATA(0x02);
				LCD_Write_DATA(0x54);

				LCD_Write_COM(0xE0);		// PLL enable
				LCD_Write_DATA(0x01);
				delay(10);
				LCD_Write_COM(0xE0);
				LCD_Write_DATA(0x03);
				delay(10);
				LCD_Write_COM(0x01);		// software reset
				delay(100);
				LCD_Write_COM(0xE6);		//PLL setting for PCLK, depends on resolution
				LCD_Write_DATA(0x03);
				LCD_Write_DATA(0xFF);
				LCD_Write_DATA(0xFF);

				LCD_Write_COM(0xB0);		//LCD SPECIFICATION
				LCD_Write_DATA(0x24);
				LCD_Write_DATA(0x00);


				LCD_Write_DATA(0x03);		//Set HDP	799
				LCD_Write_DATA(0x1F);
				LCD_Write_DATA(0x01);		//Set VDP	479
				LCD_Write_DATA(0xDF);
				LCD_Write_DATA(0x00);

				LCD_Write_COM(0xB4);		//HSYNC
				LCD_Write_DATA(0x03);		//Set HT	928
				LCD_Write_DATA(0xA0);
				LCD_Write_DATA(0x00);		//Set HPS	46
				LCD_Write_DATA(0x2E);
				LCD_Write_DATA(0x30);		//Set HPW	48
				LCD_Write_DATA(0x00);		//Set LPS	15
				LCD_Write_DATA(0x0F);
				LCD_Write_DATA(0x00);

				LCD_Write_COM(0xB6);		//VSYNC
				LCD_Write_DATA(0x02);		//Set VT	525
				LCD_Write_DATA(0x0D);
				LCD_Write_DATA(0x00);		//Set VPS	16
				LCD_Write_DATA(0x10);
				LCD_Write_DATA(0x10);		//Set VPW	16
				LCD_Write_DATA(0x00);		//Set FPS	8
				LCD_Write_DATA(0x08);

				LCD_Write_COM(0xBA);
				LCD_Write_DATA(0x0F);		//GPIO[3:0] out 1

				LCD_Write_COM(0xB8);
				LCD_Write_DATA(0x07);	    //GPIO3=input, GPIO[2:0]=output
				LCD_Write_DATA(0x01);		//GPIO0 normal

				LCD_Write_COM(0x3A);		//pixel Format
				LCD_Write_DATA(0x50);

				LCD_Write_COM(0x36);		//rotation
				LCD_Write_DATA(0xC2);
				//LCD_Write_DATA(0x22);

				LCD_Write_COM(0xF0);		//pixel data interface
				LCD_Write_DATA(0x03);



				delay(1);

				//setXY(0, 0, 799, 479);

				uint16_t x1,x2,y1,y2;
				x1=0;
				x2=799;
				y1=0;
				y2=479;
				LCD_Write_COM(0x2a);
				LCD_Write_DATA(x1>>8);
				LCD_Write_DATA(x1);
				LCD_Write_DATA(x2>>8);
				LCD_Write_DATA(x2);
				LCD_Write_COM(0x2b);
				LCD_Write_DATA(y1>>8);
				LCD_Write_DATA(y1);
				LCD_Write_DATA(y2>>8);
				LCD_Write_DATA(y2);


//exit idle mode
				LCD_Write_COM(0x38);
				//pixel format

				//gamma
				LCD_Write_COM(0x26);
					LCD_Write_DATA(0b00001000);

				LCD_Write_COM(0xbc);		//processing
				LCD_Write_DATA(0x30);  //contrast
				LCD_Write_DATA(0x45);  //brightness
				LCD_Write_DATA(0x40); //saturation
				LCD_Write_DATA(0x00); //on post-processing

				LCD_Write_COM(0x29);		//display on




				LCD_Write_COM(0xBE);		//set PWM for B/L
				LCD_Write_DATA(0x06);
				LCD_Write_DATA(0xf0);
				LCD_Write_DATA(0x01);
				LCD_Write_DATA(0xff);
				LCD_Write_DATA(0x00);
				LCD_Write_DATA(0x0f);

				LCD_Write_COM(0xd0);
				LCD_Write_DATA(0x05);//LCD_Write_DATA(0x0d);





				LCD_Write_COM(0x2C);
					displayColor=0;
					for(color1=0;color1<380000;color1++)
					{
						LCD_WR_Data(displayColor);
						if(color1%800==0)
						{
							displayColor++;
							LCD_Write_COM(0x3C);
						}

					}

					LCD_Write_COM(0x2C);

					for(color1=0;color1<384000;color1++)
					{
						LCD_WR_Data(LCD_COLOR_WHITE);
					}
					setArea(320,240,319,239);
					LCD_Write_COM(0x2C);
					for(color1=0;color1<380000;color1++)
					{
						LCD_WR_Data(color1);
						if(color1%800==0)
						{
							color1++;

						}
					}



	}




	Delay(200);
	color1=0;
	LCD_SetFont(&LCD_DEFAULT_FONT);


}


/*
 * likely need to swap x and y. to be consistent with other displays.
 * This usage of x,y is likely to be better and more correct.
 * But lets use other usage for consistency and all dependent code. And time.
 * Can implement x,y swap in set position code.
 */
void setArea(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{

								LCD_Write_COM(0x2a);
						  		LCD_Write_DATA(x1>>8);
						  		LCD_Write_DATA(x1);
						  		LCD_Write_DATA(x2>>8);
						  		LCD_Write_DATA(x2);
								LCD_Write_COM(0x2b);
						  		LCD_Write_DATA(y1>>8);
						  		LCD_Write_DATA(y1);
						  		LCD_Write_DATA(y2>>8);
						  		LCD_Write_DATA(y2);
}
/**
  * @brief  Sets the LCD Text and Background colors.
  * @param  _TextColor: specifies the Text Color.
  * @param  _BackColor: specifies the Background Color.
  * @retval None
  */
void LCD_SetColors(__IO uint16_t _TextColor, __IO uint16_t _BackColor)
{
  TextColor = _TextColor; 
  BackColor = _BackColor;
}

/**
  * @brief  Gets the LCD Text and Background colors.
  * @param  _TextColor: pointer to the variable that will contain the Text 
            Color.
  * @param  _BackColor: pointer to the variable that will contain the Background 
            Color.
  * @retval None
  */
void LCD_GetColors(__IO uint16_t *_TextColor, __IO uint16_t *_BackColor)
{
  *_TextColor = TextColor; *_BackColor = BackColor;
}

/**
  * @brief  Sets the Text color.
  * @param  Color: specifies the Text color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetTextColor(__IO uint16_t Color)
{
  TextColor = Color;
}


/**
  * @brief  Sets the Background color.
  * @param  Color: specifies the Background color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetBackColor(__IO uint16_t Color)
{
  BackColor = Color;
}

/**
  * @brief  Sets the Text Font.
  * @param  fonts: specifies the font to be used.
  * @retval None
  */
void LCD_SetFont(sFONT *fonts)
{
  LCD_Currentfonts = fonts;
}

/**
  * @brief  Gets the Text Font.
  * @param  None.
  * @retval the used font.
  */
sFONT *LCD_GetFont(void)
{
  return LCD_Currentfonts;
}

/**
  * @brief  Clears the selected line.
  * @param  Line: the Line to be cleared.
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..n
  * @retval None
  */
void LCD_ClearLine(uint16_t Line)
{
  uint16_t refcolumn = LCDSizeY - 1;
  /* Send the string character by character on lCD */
  while (((refcolumn + 1)&0xFFFF) >= LCD_Currentfonts->Width)
  {
    /* Display one character on LCD */
    LCD_DisplayChar(Line, refcolumn, ' ');
    /* Decrement the column position by 16 */
    refcolumn -= LCD_Currentfonts->Width;
  }
}


/**
  * @brief  Clears the hole LCD.
  * @param  Color: the color of the background.
  * @retval None
  */
void LCD_Clear(uint16_t Color)
{
  uint32_t index = 0;
  
  LCD_SetCursor(0xEF, 0x13F);
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index = 0; index < 76800; index++)
  {
    //LCD->LCD_RAM = Color;
    *(__IO uint16_t *) (Bank1_LCD_D)=Color;
  }  
}


/**
  * @brief  Sets the cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position. 
  * @retval None
  */
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	//temp for 3.2 screen, commented out is for orig screen

	if(LCD_IDP==0x9235 || LCD_IDP==0x4532 || LCD_IDP==0x9320)
	{
		LCD_WriteReg(LCD_REG_32, Xpos);
		LCD_WriteReg(LCD_REG_33, Ypos);
	}

	if(LCD_IDP==0x8989)
	{
		LCD_WriteReg(0x4E, Xpos);
		LCD_WriteReg(0x4F, Ypos);
	}

	//this likely need some clever thought.
	if(LCD_IDP==0x0)
	{

		//temp full screen for video
		//Xpos=320;
		//Ypos=240;
		//return;
		uint16_t x1,x2,y1,y2;
					x1=0;
					x2=Ypos;
					y1=0;
					y2=Xpos;

					LCD_Write_COM(0x2a);
					  		LCD_Write_DATA(x1>>8);
					  		LCD_Write_DATA(x1);
					  		LCD_Write_DATA(x2>>8);
					  		LCD_Write_DATA(x2);
							LCD_Write_COM(0x2b);
					  		LCD_Write_DATA(y1>>8);
					  		LCD_Write_DATA(y1);
					  		LCD_Write_DATA(y2>>8);
					  		LCD_Write_DATA(y2);

	}
}


/**
  * @brief  Draws a character on LCD.
  * @param  Xpos: the Line where to display the character shape.
  * @param  Ypos: start column address.
  * @param  c: pointer to the character data.
  * @retval None
  */
void LCD_DrawChar(uint16_t Xpos, uint16_t Ypos, const uint16_t *c)
{
  uint32_t index = 0, i = 0;
  uint16_t Xaddress = 0;
   
  Xaddress = Xpos;
  
  LCD_SetCursor(Xaddress, Ypos);
  
  for(index = 0; index < LCD_Currentfonts->Height; index++)
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(i = 0; i < LCD_Currentfonts->Width; i++)
    {
      if((((c[index] & ((0x80 << ((LCD_Currentfonts->Width / 12 ) * 8 ) ) >> i)) == 0x00) &&(LCD_Currentfonts->Width <= 12))||
        (((c[index] & (0x1 << i)) == 0x00)&&(LCD_Currentfonts->Width > 12 )))

      {

    	  LCD_WriteRAM(BackColor);
      }
      else
      {


        LCD_WriteRAM(TextColor);
      }
    }
    Xaddress++;
    LCD_SetCursor(Xaddress, Ypos);
  }
}


/**
  * @brief  Displays one character (16dots width, 24dots height).
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  Column: start column address.
  * @param  Ascii: character ascii code, must be between 0x20 and 0x7E.
  * @retval None
  */
void LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii)
{
  Ascii -= 32;
  LCD_DrawChar(Line, Column, &LCD_Currentfonts->table[Ascii * LCD_Currentfonts->Height]);
}


/**
  * @brief  Displays a maximum of 20 char on the LCD.
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  *ptr: pointer to string to display on LCD.
  * @retval None
  */
void LCD_DisplayStringLine(uint16_t Line, uint8_t *ptr)
{

	//based on 16*24 char on a 320*240 screen, 20*10 chars can be displayed
	//on an 800*480 screen, 20 lines

	Line=Line*LCDSizeX/240;


	uint16_t refcolumn = LCDSizeY - 1;

  /* Send the string character by character on lCD */
  while ((*ptr != 0) & (((refcolumn + 1) & 0xFFFF) >= LCD_Currentfonts->Width))
  {
    /* Display one character on LCD */
    LCD_DisplayChar(Line, refcolumn, *ptr);
    /* Decrement the column position by 16 */
    refcolumn -= LCD_Currentfonts->Width;
    /* Point on the next character */
    ptr++;
  }
}

//displays menu from centre of screen
void LCD_DisplayStringLineFromCentre(uint16_t Line, uint8_t *ptr)
{

	Line=Line*(LCDSizeX/240);
  uint16_t refcolumn = LCDSizeY/2 - 1;

  /* Send the string character by character on lCD */
  while ((*ptr != 0) & (((refcolumn + 1) & 0xFFFF) >= LCD_Currentfonts->Width))
  {
    /* Display one character on LCD */
    LCD_DisplayChar(Line, refcolumn, *ptr);
    /* Decrement the column position by 16 */
    refcolumn -= LCD_Currentfonts->Width;
    /* Point on the next character */
    ptr++;
  }
}

//Text Slider
//This should hopefully change the text background as visual feedback for the slider
//Start with spaces should do with having the slider long, more clever implementation later
void LCD_DisplayStringLineSlider(uint16_t Line, uint8_t *ptr, uint16_t yPos)
{

	Line=Line*(LCDSizeX/240);
	yPos=(yPos*LCDSizeY)/1000;
  uint16_t refcolumn = LCDSizeY - 1;

  uint16_t backgroundBackup=BackColor;
  /* Send the string character by character on lCD */
  while ((*ptr != 0) & (((refcolumn + 1) & 0xFFFF) >= LCD_Currentfonts->Width))
  {
	if(refcolumn>yPos)
	{
		BackColor=LCD_COLOR_BLUE;
	}
	else
	{
		BackColor=LCD_COLOR_RED;
	}
    /* Display one character on LCD */
    LCD_DisplayChar(Line, refcolumn, *ptr);
    /* Decrement the column position by 16 */
    refcolumn -= LCD_Currentfonts->Width;
    /* Point on the next character */
    ptr++;
  }
  BackColor=backgroundBackup;
}


/**
  * @brief  Sets a display window
  * @param  Xpos: specifies the X buttom left position.
  * @param  Ypos: specifies the Y buttom left position.
  * @param  Height: display window height.
  * @param  Width: display window width.
  * @retval None
  */
void LCD_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  /* Horizontal GRAM Start Address */
  if(Xpos >= Height)
  {
    LCD_WriteReg(LCD_REG_80, (Xpos - Height + 1));
  }
  else
  {
    LCD_WriteReg(LCD_REG_80, 0);
  }
  /* Horizontal GRAM End Address */
  LCD_WriteReg(LCD_REG_81, Xpos);
  /* Vertical GRAM Start Address */
  if(Ypos >= Width)
  {
    LCD_WriteReg(LCD_REG_82, (Ypos - Width + 1));
  }  
  else
  {
    LCD_WriteReg(LCD_REG_82, 0);
  }
  /* Vertical GRAM End Address */
  LCD_WriteReg(LCD_REG_83, Ypos);
  LCD_SetCursor(Xpos, Ypos);
}


/**
  * @brief  Disables LCD Window mode.
  * @param  None
  * @retval None
  */
void LCD_WindowModeDisable(void)
{
  LCD_SetDisplayWindow(239, 0x13F, 240, 320);
  LCD_WriteReg(LCD_REG_3, 0x1018);    
}


/**
  * @brief  Displays a line.
  * @param Xpos: specifies the X position.
  * @param Ypos: specifies the Y position.
  * @param Length: line length.
  * @param Direction: line direction.
  *   This parameter can be one of the following values: Vertical or Horizontal.
  * @retval None
  */
void LCD_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction)
{
  uint32_t i = 0;
  
  LCD_SetCursor(Xpos, Ypos);
  if(Direction == LCD_DIR_HORIZONTAL)
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    for(i = 0; i < Length; i++)
    {
      LCD_WriteRAM(TextColor);
    }
  }
  else
  {
    for(i = 0; i < Length; i++)
    {
      LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
      LCD_WriteRAM(TextColor);
      Xpos++;
      LCD_SetCursor(Xpos, Ypos);
    }
  }
}


/**
  * @brief  Displays a rectangle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: display rectangle height.
  * @param  Width: display rectangle width.
  * @retval None
  */
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  LCD_DrawLine(Xpos, Ypos, Width, LCD_DIR_HORIZONTAL);
  LCD_DrawLine((Xpos + Height), Ypos, Width, LCD_DIR_HORIZONTAL);
  
  LCD_DrawLine(Xpos, Ypos, Height, LCD_DIR_VERTICAL);
  LCD_DrawLine(Xpos, (Ypos - Width + 1), Height, LCD_DIR_VERTICAL);
}


/**
  * @brief  Displays a circle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Radius
  * @retval None
  */
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;/* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);
  CurX = 0;
  CurY = Radius;
  
  while (CurX <= CurY)
  {
    LCD_SetCursor(Xpos + CurX, Ypos + CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos + CurX, Ypos - CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurX, Ypos + CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurX, Ypos - CurY);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos + CurY, Ypos + CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos + CurY, Ypos - CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurY, Ypos + CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    LCD_SetCursor(Xpos - CurY, Ypos - CurX);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    LCD_WriteRAM(TextColor);
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}


/**
  * @brief  Displays a monocolor picture.
  * @param  Pict: pointer to the picture array.
  * @retval None
  */
void LCD_DrawMonoPict(const uint32_t *Pict)
{
  uint32_t index = 0, i = 0;
  LCD_SetCursor(0, (LCDSizeY - 1));
  LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  for(index = 0; index < 2400; index++)
  {
    for(i = 0; i < 32; i++)
    {
      if((Pict[index] & (1 << i)) == 0x00)
      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      }
    }
  }
}


/**
  * @brief  Displays a bitmap picture loaded in the internal Flash.
  * @param  BmpAddress: Bmp picture address in the internal Flash.
  * @retval None
  */
void LCD_WriteBMP(uint32_t BmpAddress)
{
  uint32_t index = 0, size = 0;
  /* Read bitmap size */
  size = *(__IO uint16_t *) (BmpAddress + 2);
  size |= (*(__IO uint16_t *) (BmpAddress + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(__IO uint16_t *) (BmpAddress + 10);
  index |= (*(__IO uint16_t *) (BmpAddress + 12)) << 16;
  size = (size - index)/2;
  BmpAddress += index;
  /* Set GRAM write direction and BGR = 1 */
  /* I/D=00 (Horizontal : decrement, Vertical : decrement) */
  /* AM=1 (address is updated in vertical writing direction) */
  LCD_WriteReg(LCD_REG_3, 0x1008);
 
  LCD_WriteRAM_Prepare();
 
  for(index = 0; index < size; index++)
  {
    LCD_WriteRAM(*(__IO uint16_t *)BmpAddress);
    BmpAddress += 2;
  }
 
  /* Set GRAM write direction and BGR = 1 */
  /* I/D = 01 (Horizontal : increment, Vertical : decrement) */
  /* AM = 1 (address is updated in vertical writing direction) */
  LCD_WriteReg(LCD_REG_3, 0x1018);
}

/**
  * @brief  Displays a full rectangle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: rectangle height.
  * @param  Width: rectangle width.
  * @retval None
  */
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  LCD_SetTextColor(TextColor);

  LCD_DrawLine(Xpos, Ypos, Width, LCD_DIR_HORIZONTAL);
  LCD_DrawLine((Xpos + Height), Ypos, Width, LCD_DIR_HORIZONTAL);
  
  LCD_DrawLine(Xpos, Ypos, Height, LCD_DIR_VERTICAL);
  LCD_DrawLine(Xpos, (Ypos - Width + 1), Height, LCD_DIR_VERTICAL);

  Width -= 2;
  Height--;
  Ypos--;

  LCD_SetTextColor(BackColor);

  while(Height--)
  {
    LCD_DrawLine(++Xpos, Ypos, Width, LCD_DIR_HORIZONTAL);    
  }

  LCD_SetTextColor(TextColor);
}

/**
  * @brief  Displays a full circle.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Radius
  * @retval None
  */
void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;    /* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);

  CurX = 0;
  CurY = Radius;
  
  LCD_SetTextColor(BackColor);

  while (CurX <= CurY)
  {
    if(CurY > 0) 
    {
      LCD_DrawLine(Xpos - CurX, Ypos + CurY, 2*CurY, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(Xpos + CurX, Ypos + CurY, 2*CurY, LCD_DIR_HORIZONTAL);
    }

    if(CurX > 0) 
    {
      LCD_DrawLine(Xpos - CurY, Ypos + CurX, 2*CurX, LCD_DIR_HORIZONTAL);
      LCD_DrawLine(Xpos + CurY, Ypos + CurX, 2*CurX, LCD_DIR_HORIZONTAL);
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  LCD_SetTextColor(TextColor);
  LCD_DrawCircle(Xpos, Ypos, Radius);
}

/**
  * @brief  Displays an uni line (between two points).
  * @param  x1: specifies the point 1 x position.
  * @param  y1: specifies the point 1 y position.
  * @param  x2: specifies the point 2 x position.
  * @param  y2: specifies the point 2 y position.
  * @retval None
  */
void LCD_DrawUniLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
  curpixel = 0;
  
  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */
  
  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }
  
  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }
  
  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }
  
  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    PutPixel(x, y);             /* Draw the current pixel */
    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  }
}

/**
  * @brief  Displays an polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_PolyLine(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0;

  if(PointCount < 2)
  {
    return;
  }

  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    LCD_DrawUniLine(X, Y, Points->X, Points->Y);
  }
}

/**
  * @brief  Displays an relative polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @param  Closed: specifies if the draw is closed or not.
  *           1: closed, 0 : not closed.
  * @retval None
  */
static void LCD_PolyLineRelativeClosed(pPoint Points, uint16_t PointCount, uint16_t Closed)
{
  int16_t X = 0, Y = 0;
  pPoint First = Points;

  if(PointCount < 2)
  {
    return;
  }  
  X = Points->X;
  Y = Points->Y;
  while(--PointCount)
  {
    Points++;
    LCD_DrawUniLine(X, Y, X + Points->X, Y + Points->Y);
    X = X + Points->X;
    Y = Y + Points->Y;
  }
  if(Closed)
  {
    LCD_DrawUniLine(First->X, First->Y, X, Y);
  }  
}

/**
  * @brief  Displays a closed polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_ClosedPolyLine(pPoint Points, uint16_t PointCount)
{
  LCD_PolyLine(Points, PointCount);
  LCD_DrawUniLine(Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y);
}

/**
  * @brief  Displays a relative polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_PolyLineRelative(pPoint Points, uint16_t PointCount)
{
  LCD_PolyLineRelativeClosed(Points, PointCount, 0);
}

/**
  * @brief  Displays a closed relative polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_ClosedPolyLineRelative(pPoint Points, uint16_t PointCount)
{
  LCD_PolyLineRelativeClosed(Points, PointCount, 1);
}


/**
  * @brief  Displays a  full polyline (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_FillPolyLine(pPoint Points, uint16_t PointCount)
{
  /*  public-domain code by Darel Rex Finley, 2007 */
  uint16_t  nodes = 0, nodeX[MAX_POLY_CORNERS], pixelX = 0, pixelY = 0, i = 0,
  j = 0, swap = 0;
  uint16_t  IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;

  IMAGE_LEFT = IMAGE_RIGHT = Points->X;
  IMAGE_TOP= IMAGE_BOTTOM = Points->Y;

  for(i = 1; i < PointCount; i++)
  {
    pixelX = POLY_X(i);
    if(pixelX < IMAGE_LEFT)
    {
      IMAGE_LEFT = pixelX;
    }
    if(pixelX > IMAGE_RIGHT)
    {
      IMAGE_RIGHT = pixelX;
    }
    
    pixelY = POLY_Y(i);
    if(pixelY < IMAGE_TOP)
    { 
      IMAGE_TOP = pixelY;
    }
    if(pixelY > IMAGE_BOTTOM)
    {
      IMAGE_BOTTOM = pixelY;
    }
  }
  
  LCD_SetTextColor(BackColor);  

  /*  Loop through the rows of the image. */
  for (pixelY = IMAGE_TOP; pixelY < IMAGE_BOTTOM; pixelY++) 
  {  
    /* Build a list of nodes. */
    nodes = 0; j = PointCount-1;

    for (i = 0; i < PointCount; i++) 
    {
      if (POLY_Y(i)<(double) pixelY && POLY_Y(j)>=(double) pixelY || POLY_Y(j)<(double) pixelY && POLY_Y(i)>=(double) pixelY) 
      {
        nodeX[nodes++]=(int) (POLY_X(i)+((pixelY-POLY_Y(i))*(POLY_X(j)-POLY_X(i)))/(POLY_Y(j)-POLY_Y(i))); 
      }
      j = i; 
    }
  
    /* Sort the nodes, via a simple "Bubble" sort. */
    i = 0;
    while (i < nodes-1) 
    {
      if (nodeX[i]>nodeX[i+1]) 
      {
        swap = nodeX[i]; 
        nodeX[i] = nodeX[i+1]; 
        nodeX[i+1] = swap; 
        if(i)
        {
          i--; 
        }
      }
      else 
      {
        i++;
      }
    }
  
    /*  Fill the pixels between node pairs. */
    for (i = 0; i < nodes; i+=2) 
    {
      if(nodeX[i] >= IMAGE_RIGHT) 
      {
        break;
      }
      if(nodeX[i+1] > IMAGE_LEFT) 
      {
        if (nodeX[i] < IMAGE_LEFT)
        {
          nodeX[i]=IMAGE_LEFT;
        }
        if(nodeX[i+1] > IMAGE_RIGHT)
        {
          nodeX[i+1] = IMAGE_RIGHT;
        }
        LCD_SetTextColor(BackColor);
        LCD_DrawLine(pixelY, nodeX[i+1], nodeX[i+1] - nodeX[i], LCD_DIR_HORIZONTAL);
        LCD_SetTextColor(TextColor);
        PutPixel(pixelY, nodeX[i+1]);
        PutPixel(pixelY, nodeX[i]);
        /* for (j=nodeX[i]; j<nodeX[i+1]; j++) PutPixel(j,pixelY); */
      }
    }
  } 

  /* draw the edges */
  LCD_SetTextColor(TextColor);
}

/**
  * @brief  Writes to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @param  LCD_RegValue: value to write to the selected register.
  * @retval None
  */
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
  /* Write 16-bit Index, then Write Reg */
  LCD->LCD_REG = LCD_Reg;
  /* Write 16-bit Reg */
 // LCD->LCD_RAM = LCD_RegValue;
  *(__IO uint16_t *) (Bank1_LCD_D)= LCD_RegValue;
}


/**
  * @brief  Reads the selected LCD Register.
  * @param  LCD_Reg: address of the selected register.
  * @retval LCD Register Value.
  */
uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD->LCD_REG = LCD_Reg;
  /* Read 16-bit Reg */
  return (*(__IO uint16_t *) (Bank1_LCD_D));
}


/**
  * @brief  Prepare to write to the LCD RAM.
  * @param  None
  * @retval None
  */
void LCD_WriteRAM_Prepare(void)
{
	if(LCD_IDP==0x0)
	{
		//might want 3C for continuity
		LCD->LCD_REG = 0x2C;
	}
	else
	{
		LCD->LCD_REG = LCD_REG_34;
	}

}

void LCD_WriteRAM_Continue(void)
{
	LCD->LCD_REG = 0x3C;
}

/**
  * @brief  Writes to the LCD RAM.
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAM(uint16_t RGB_Code)
{
  /* Write 16-bit GRAM Reg */
  //LCD->LCD_RAM = RGB_Code;
  *(__IO uint16_t *) (Bank1_LCD_D)= RGB_Code;

}


/**
  * @brief  Reads the LCD RAM.
  * @param  None
  * @retval LCD RAM Value.
  */
uint16_t LCD_ReadRAM(void)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD->LCD_REG = LCD_REG_34; /* Select GRAM Reg */
  /* Read 16-bit Reg */
  return LCD->LCD_RAM;
}


/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void LCD_PowerOn(void)
{
/* Power On sequence ---------------------------------------------------------*/
  LCD_WriteReg(LCD_REG_16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  LCD_WriteReg(LCD_REG_17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
  LCD_WriteReg(LCD_REG_18, 0x0000); /* VREG1OUT voltage */
  LCD_WriteReg(LCD_REG_19, 0x0000); /* VDV[4:0] for VCOM amplitude*/
  _delay_(20);                 /* Dis-charge capacitor power voltage (200ms) */
  LCD_WriteReg(LCD_REG_16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
  LCD_WriteReg(LCD_REG_17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
  _delay_(5);                  /* Delay 50 ms */
  LCD_WriteReg(LCD_REG_18, 0x0139); /* VREG1OUT voltage */
  _delay_(5);                  /* Delay 50 ms */
  LCD_WriteReg(LCD_REG_19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
  LCD_WriteReg(LCD_REG_41, 0x0013); /* VCM[4:0] for VCOMH */
  _delay_(5);                  /* Delay 50 ms */
  LCD_WriteReg(LCD_REG_7, 0x0173);  /* 262K color and display ON */
}


/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void LCD_DisplayOn(void)
{
  /* Display On */
  LCD_WriteReg(LCD_REG_7, 0x0173); /* 262K color and display ON */
}


/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void LCD_DisplayOff(void)
{
  /* Display Off */
  LCD_WriteReg(LCD_REG_7, 0x0); 
}


/**
  * @brief  Configures LCD Control lines (FSMC Pins) in alternate function mode.
  * @param  None
  * @retval None
  */
void LCD_CtrlLinesConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;


	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1| RCC_APB1Periph_USART2 |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
	                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
	                         RCC_APB2Periph_GPIOE, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_6|GPIO_Pin_3;
	//  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_2|GPIO_Pin_3;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;		 //COL4
	  GPIO_Init(GPIOE, &GPIO_InitStructure);

	  GPIO_SetBits(GPIOC, GPIO_Pin_5|GPIO_Pin_2|GPIO_Pin_3);
	  GPIO_SetBits(GPIOE, GPIO_Pin_6);


	  GPIO_SetBits(GPIOB, GPIO_Pin_5);	     //
	  GPIO_SetBits(GPIOD, GPIO_Pin_6|GPIO_Pin_3 );
	     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ; 	 //LCD-RST
	  GPIO_Init(GPIOE, &GPIO_InitStructure);




	  //PB10, PD9, PD10   CS,SI,CLK

	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //推挽输出
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7 ;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;           //SPI1 CS1
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;           //SPI1 CS4
	//  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;           //SPI1 NSS
	  GPIO_Init(GPIOA, &GPIO_InitStructure);



	  //PENIRQ, SO
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  //GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
	  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  //GPIO_Init(GPIOB, &GPIO_InitStructure);






	/*-- GPIO Configuration ------------------------------------------------------*/
	  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
	                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |
	                                GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  /* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
	     PE.14(D11), PE.15(D12) as alternate function push pull */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
	                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
	                                GPIO_Pin_15;
	  GPIO_Init(GPIOE, &GPIO_InitStructure);



	  /* NE1 configuration */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  /* RS */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);


	  GPIO_SetBits(GPIOD, GPIO_Pin_7);			//CS=1
	  GPIO_SetBits(GPIOD, GPIO_Pin_14| GPIO_Pin_15 |GPIO_Pin_0 | GPIO_Pin_1);
	  GPIO_SetBits(GPIOE, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10);
	  GPIO_ResetBits(GPIOE, GPIO_Pin_0);
	  GPIO_ResetBits(GPIOE, GPIO_Pin_1);		//RESET=0
	  GPIO_SetBits(GPIOD, GPIO_Pin_4);		    //RD=1
	  GPIO_SetBits(GPIOD, GPIO_Pin_5);			//WR=1

	  GPIO_SetBits(GPIOB, GPIO_Pin_6);			//PEn
	  //GPIO_SetBits(GPIOA, GPIO_Pin_6);			//PEn
	  GPIO_SetBits(GPIOD, GPIO_Pin_13);			//LIGHT
	  //GPIO_SetBits(GPIOB, GPIO_Pin_7);			//SPI CS3
	  GPIO_SetBits(GPIOC, GPIO_Pin_4);			//SPI CS1
	 // GPIO_SetBits(GPIOB, GPIO_Pin_12);			//SPI CS4
	  GPIO_SetBits(GPIOA, GPIO_Pin_4);			//SPI NSS		//WR=1

}


/**
  * @brief  Configures the Parallel interface (FSMC) for LCD(Parallel mode)
  * @param  None
  * @retval None
  */
void LCD_FSMCConfig(void)
{
	 FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	  FSMC_NORSRAMTimingInitTypeDef  p;
	  //GPIO_InitTypeDef GPIO_InitStructure;



	/*-- FSMC Configuration ------------------------------------------------------*/


	  p.FSMC_AddressSetupTime = 0x02;
	  p.FSMC_AddressHoldTime = 0x00;
	  p.FSMC_DataSetupTime = 0x05;
	  p.FSMC_BusTurnAroundDuration = 0x00;
	  p.FSMC_CLKDivision = 0x00;
	  p.FSMC_DataLatency = 0x00;
	  p.FSMC_AccessMode = FSMC_AccessMode_B;


	  //FSMC_Bank1_NORSRAM10  for PG10 chip select
	  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
	  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
	  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
	  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
	  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
	  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
	  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
	  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;


	  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);

	  /* Enable FSMC Bank1_SRAM Bank */
	  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

/**
  * @brief  Displays a pixel.
  * @param  x: pixel x.
  * @param  y: pixel y.  
  * @retval None
  */
static void PutPixel(int16_t x, int16_t y)
{ 
  if(x < 0 || x > 239 || y < 0 || y > 319)
  {
    return;  
  }
  LCD_DrawLine(x, y, 1, LCD_DIR_HORIZONTAL);
}

#ifndef USE_Delay
/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
static void delay(vu32 nCount)
{
  vu32 index = 0; 
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}
#endif /* USE_Delay*/
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */  

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
