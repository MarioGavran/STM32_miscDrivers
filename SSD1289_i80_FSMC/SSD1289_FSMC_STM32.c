/****************************************************************************************************************
LIBRARY: 	SSD1289_FSMC_STM32.c
			- only for STM32F413ZH devices.
			- contains functions for:
				* initializing GPIO pins used, the FSM Controller and SSD1289 LCD Controller.
				* function for reading/writing data from/to the FSMC bus interface.
				* functions for drawing points and lines, printing characters and strings, and 
				  displaying images

AUTHOR: 	Mario Gavran, E5029604, University of Maribor - FERI
*****************************************************************************************************************/


#include "stm32f4xx.h"                  // Device header
#include "SSD1289_FSMC_STM32.h"		// SSD1289 header
#include <stdlib.h>			// Standard library header

/*GPIO initialization used for FSMC*/
//****************************************************************************************************************
void ssd1289_FSMC_GPIO_init()
{
	// Enable clock on all ports used with FSMC
	RCC->AHB1ENR	|=	RCC_AHB1ENR_GPIOBEN
				|RCC_AHB1ENR_GPIOCEN
				|RCC_AHB1ENR_GPIODEN
				|RCC_AHB1ENR_GPIOEEN;
	
	//mode out - 01
	GPIOC->MODER	|=	GPIO_MODER_MODER6_0;		// PC6  - RST
	//mode af  - 10
	GPIOD->MODER	|=	GPIO_MODER_MODER11_1		// PD11 - A[16]	- RS
				|GPIO_MODER_MODER4_1		// PD4  - NOE   - RD
				|GPIO_MODER_MODER5_1		// PD5  - NWE   - WR
				|GPIO_MODER_MODER7_1		// PD7  - NE1 	- CS
				|GPIO_MODER_MODER14_1		// PD14 - D0
				|GPIO_MODER_MODER15_1;		// PD15 - D1
	GPIOC->MODER	&=	~(GPIO_MODER_MODER11_Msk
				|GPIO_MODER_MODER12_Msk);	// clear before setting
	GPIOC->MODER	|=	GPIO_MODER_MODER11_1		// PC11 - D2
				|GPIO_MODER_MODER12_1;		// PC12 - D3
	GPIOE->MODER	|=	GPIO_MODER_MODER7_1		// PE7  - D4
				|GPIO_MODER_MODER8_1		// PE8  - D5
				|GPIO_MODER_MODER9_1		// PE9  - D6
				|GPIO_MODER_MODER10_1		// PE10 - D7
				|GPIO_MODER_MODER11_1		// PE11 - D8
				|GPIO_MODER_MODER12_1		// PE12 - D9
				|GPIO_MODER_MODER13_1		// PE13 - D10
				|GPIO_MODER_MODER14_1		// PE14 - D11
				|GPIO_MODER_MODER15_1;		// PE15 - D12
	GPIOB->MODER	|=	GPIO_MODER_MODER12_1;		// PB12 - D13
	GPIOD->MODER	|=	GPIO_MODER_MODER9_1		// PD9  - D14
				|GPIO_MODER_MODER10_1;		// PD10 - D15
	
	
	//type - PUSH PULL - jel ti treba uopce ?, posto je Reset value: 0x0000 0000
	GPIOC->OTYPER	&=	 ~(GPIO_OTYPER_OT_6);
	GPIOD->OTYPER	&=	~((GPIO_OTYPER_OT_11)
				|(GPIO_OTYPER_OT_4)
				|(GPIO_OTYPER_OT_5)
				|(GPIO_OTYPER_OT_7)
				|(GPIO_OTYPER_OT_14)
				|(GPIO_OTYPER_OT_15));
	GPIOC->OTYPER	&=	~((GPIO_OTYPER_OT_11)
				|(GPIO_OTYPER_OT_12));
	GPIOE->OTYPER	&=	~((GPIO_OTYPER_OT_7)
				|(GPIO_OTYPER_OT_8)
				|(GPIO_OTYPER_OT_9)
				|(GPIO_OTYPER_OT_10)
				|(GPIO_OTYPER_OT_11)
				|(GPIO_OTYPER_OT_12)
				|(GPIO_OTYPER_OT_13)
				|(GPIO_OTYPER_OT_14)
				|(GPIO_OTYPER_OT_15));
	GPIOB->OTYPER	&=	~(GPIO_OTYPER_OT_12);
	GPIOD->OTYPER	&=	~((GPIO_OTYPER_OT_9)
				|(GPIO_OTYPER_OT_10));
	
	RCC->APB2ENR	|=	RCC_APB2ENR_SYSCFGEN;					// Enable SYSCFG registers.
	SYSCFG->CMPCR	|=	SYSCFG_CMPCR_CMP_PD;					// Enable Compensation cell.
	while(!(SYSCFG->CMPCR & SYSCFG_CMPCR_READY));			// Wait for ENABLE flag
	//SPEED - LOW
	GPIOC->OSPEEDR	&=	~(GPIO_OSPEEDER_OSPEEDR6_Msk);
	GPIOD->OSPEEDR	&=	~((GPIO_OSPEEDER_OSPEEDR11_Msk)
				|(GPIO_OSPEEDER_OSPEEDR4_Msk)
				|(GPIO_OSPEEDER_OSPEEDR5_Msk)
				|(GPIO_OSPEEDER_OSPEEDR7_Msk)
				|(GPIO_OSPEEDER_OSPEEDR14_Msk)
				|(GPIO_OSPEEDER_OSPEEDR15_Msk));
	GPIOC->OSPEEDR	&=	~((GPIO_OSPEEDER_OSPEEDR11_Msk)
				|(GPIO_OSPEEDER_OSPEEDR12_Msk));
	GPIOE->OSPEEDR	&=	~((GPIO_OSPEEDER_OSPEEDR7_Msk)
				|(GPIO_OSPEEDER_OSPEEDR8_Msk)
				|(GPIO_OSPEEDER_OSPEEDR9_Msk)
				|(GPIO_OSPEEDER_OSPEEDR10_Msk)
				|(GPIO_OSPEEDER_OSPEEDR11_Msk)
				|(GPIO_OSPEEDER_OSPEEDR12_Msk)
				|(GPIO_OSPEEDER_OSPEEDR13_Msk)
				|(GPIO_OSPEEDER_OSPEEDR14_Msk)
				|(GPIO_OSPEEDER_OSPEEDR15_Msk));
	GPIOB->OSPEEDR	&=	~(GPIO_OSPEEDER_OSPEEDR12_Msk);
	GPIOD->OSPEEDR	&=	~((GPIO_OSPEEDER_OSPEEDR9_Msk)
				|(GPIO_OSPEEDER_OSPEEDR10_Msk));
										 
	//NO PULL UP/DOWN - jel ti treba uopce ?, posto je Reset value: 0x0000 0000
	GPIOC->PUPDR	&=	~(GPIO_PUPDR_PUPDR6_Msk);
	GPIOD->PUPDR	&=	~((GPIO_PUPDR_PUPDR11_Msk)
				|(GPIO_PUPDR_PUPDR4_Msk)
				|(GPIO_PUPDR_PUPDR5_Msk)
				|(GPIO_PUPDR_PUPDR7_Msk)
				|(GPIO_PUPDR_PUPDR14_Msk)
				|(GPIO_PUPDR_PUPDR15_Msk));
	GPIOC->PUPDR	&=	~((GPIO_PUPDR_PUPDR11_Msk)
				|(GPIO_PUPDR_PUPDR12_Msk));
	GPIOE->PUPDR	&=	~((GPIO_PUPDR_PUPDR7_Msk)
				|(GPIO_PUPDR_PUPDR8_Msk)
				|(GPIO_PUPDR_PUPDR9_Msk)
				|(GPIO_PUPDR_PUPDR10_Msk)
				|(GPIO_PUPDR_PUPDR11_Msk)
				|(GPIO_PUPDR_PUPDR12_Msk)
				|(GPIO_PUPDR_PUPDR13_Msk)
				|(GPIO_PUPDR_PUPDR14_Msk)
				|(GPIO_PUPDR_PUPDR15_Msk));
	GPIOB->PUPDR	&=	~(GPIO_PUPDR_PUPDR12_Msk);
	GPIOD->PUPDR	&=	~((GPIO_PUPDR_PUPDR9_Msk)
				|(GPIO_PUPDR_PUPDR10_Msk));
	
	//AF12(0xC) or AF10(0xA)
	GPIOD->AFR[0] |= 0xC0CC0000;
	GPIOD->AFR[1] |= 0xCC00CCC0;	//PD  4,  5,  7, 9, 10, 11, 14, 15
	
	GPIOC->AFR[1] |= 0x000AA000;	//PC 12, 11
	
	GPIOE->AFR[0] |= 0xC0000000;
	GPIOE->AFR[1] |= 0xCCCCCCCC;	//PE 7, 8, 9, 10, 11, 12, 13, 14, 15
	
	GPIOB->AFR[1] |= 0x000C0000;	//PB 12
}


/*SSD1289 initializaton*/
//**************************************************************************************************//
void ssd1289_FSMC_init()
{
	//enable clock on fsmc peripheral
	RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;
	
	//FSMC_BCR1  0x00	 BTCR[0]
	FSMC_Bank1->BTCR[0]	|= FSMC_BCR1_MBKEN;			// Enable BANK1.
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_MUXEN);			// Disable MUX.
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_MTYP_Msk);		// Select SRAM memory type. (0x2)
	//FSMC_Bank1->BTCR[0]	|= FSMC_BCR1_MTYP_1; 			
	FSMC_Bank1->BTCR[0]	|= FSMC_BCR1_MWID_0;
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_MWID_1); 		// 16bit width.
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_FACCEN);			// Disable NOR Flash memory access operations.
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_BURSTEN);		// Disable burst mode. Asynchronous read operations.
	FSMC_Bank1->BTCR[0]	|= FSMC_BCR1_WREN;			// Enable write operations.
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_WAITEN);			// Disable NWAIT signal.
	FSMC_Bank1->BTCR[0]	|= FSMC_BCR1_EXTMOD;			// Enable extended mode. (mode B, enabling BWTR register)
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_ASYNCWAIT);		// NWAIT signal is not taken in to account.
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_CPSIZE_Msk); 		// No burst split when crossing page boundary.
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_CBURSTRW);		// Disable burst mode. Asynchronous write operations.
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_CCLKEN);			// Disable continuous clock.
	FSMC_Bank1->BTCR[0]	&= ~(FSMC_BCR1_WFDIS);			// Enable write FIFO.
	
	
	//FSMC_BTR1  0x04	 BTCR[1]  -  Read Cycle
	FSMC_Bank1->BTCR[1]	&= ~(FSMC_BTR1_ACCMOD_Msk);		// ACCMOD (ModeA)
	
	FSMC_Bank1->BTCR[1]	&= ~(FSMC_BTR1_BUSTURN_Msk);
	FSMC_Bank1->BTCR[1]	|= (FSMC_BTR1_BUSTURN_0
				| FSMC_BTR1_BUSTURN_3);			// BUSTURN 9 HCLK
	
	FSMC_Bank1->BTCR[1]	&= ~(FSMC_BTR1_DATAST_Msk);
	FSMC_Bank1->BTCR[1]	|= (FSMC_BTR1_DATAST_1 
				| FSMC_BTR1_DATAST_2);			// DATAST 5 HCLK
	
	FSMC_Bank1->BTCR[1]	&= ~(FSMC_BTR1_ADDSET_Msk);
	FSMC_Bank1->BTCR[1]	|= (FSMC_BTR1_ADDSET_1 
				| FSMC_BTR1_ADDSET_2);			// ADDSET 5 HCLK
	
	
	//FSMC_BWTR1 0x00	 BWTR1[0]  -  Write Cycle
	FSMC_Bank1E->BWTR[0]	&= ~(FSMC_BTR1_ACCMOD_Msk);		// ACCMOD (ModeA)
	FSMC_Bank1E->BWTR[0]	&= ~(FSMC_BWTR1_BUSTURN_Msk);
	FSMC_Bank1E->BWTR[0]	|= FSMC_BTR1_BUSTURN_0;			// BUSTURN 1 HCLK
	FSMC_Bank1E->BWTR[0]	&= ~(FSMC_BWTR1_DATAST_Msk);
	FSMC_Bank1E->BWTR[0]	|= FSMC_BWTR1_DATAST_0;			// DATAST 1 HCLK
	FSMC_Bank1E->BWTR[0]	&= ~(FSMC_BWTR1_ADDSET_Msk);
	FSMC_Bank1E->BWTR[0]	|= FSMC_BWTR1_ADDSET_0;			// ADDSET 1 HCLK
}



//**************************************************************************************************//
void ssd1289_Driver_init(void)
{
	Delay_m(100);    //short delay needed on power up
	
	GPIOC->ODR |= (1<<lcd_rst);      // set ~RST pin, to disable reset state on SSD1289
	
	Lcd_Write_Reg (Sleep_mode, 		0x0000U);	// Release from Sleep
	Lcd_Write_Reg (Display_control,		0x0021U);	// GON=1 DTE=0 D[1:0]=01
	Lcd_Write_Reg (Oscillation_Start,	0x0001U);	// Turn ON the oscilator
	Lcd_Write_Reg (Display_control,		0x0023U);	// GON=1 DTE=0 D[1:0]=11
	Lcd_Write_Reg (Sleep_mode,		0x0000U);	// Release from Sleep
	Delay_m(30);					 												// Wait 30ms
	Lcd_Write_Reg (Display_control,		0x0033U);	// GON=1 DTE=1 D[1:0]=11
	Lcd_Write_Reg (Entry_mode,		0x6838U);	// 65k color,  
	Lcd_Write_Reg (LCD_drive_AC_control,	0x0600U);	// B/C=1 EOR=1
	
}


/*WRITE DATA TO REGISTER*/
//**************************************************************************************************//
void Lcd_Write_Reg(uint16_t reg_index, uint16_t reg_data)
{
	SSD1289_REG = reg_index;
	SSD1289_DAT = reg_data;
}


/*WRITE INDEX*/
//**************************************************************************************************//
void	Lcd_Write_Index(uint16_t index)
{
	SSD1289_REG = index;
}


/*WRITE DATA*/
//**************************************************************************************************//
void	Lcd_Write_Data(uint16_t data)
{
	SSD1289_DAT = data;
}


/*READ DATA FROM BUS*/
//**************************************************************************************************//
uint16_t Lcd_Read_Data_From_Bus(void)
{
	volatile uint16_t data=0;
	data = (0xFFFF | SSD1289_DAT);
	return data;
}


/*READ REGISTER VALUE*/
//**************************************************************************************************//
uint16_t Lcd_Read_Reg_Val(uint16_t reg_addr)
{
	volatile uint16_t data=0;
	Lcd_Write_Index(reg_addr);
	data = Lcd_Read_Data_From_Bus();
	return data;
}


/*SET CURSOR*/
//**************************************************************************************************//
void Lcd_Set_Cursor(uint16_t x_kur, uint16_t y_kur)
{	
	Lcd_Write_Reg(Set_GDDRAM_X_address_counter,x_kur);	// Initial GDDRAM X address
	Lcd_Write_Reg(Set_GDDRAM_Y_address_counter,y_kur);	// Initial GDDRAM Y address
	Lcd_Write_Index(RAM_data_write_read);			// Set index to RAM_data_write
}


/*CLEAR LCD SCREEN(SET BACKGROUND)*/
//**************************************************************************************************//
void Lcd_Clear(uint16_t color)
{
	uint32_t	i = 0;
	
	Lcd_Set_Cursor(0x0000U,0x0000U);
	
	for(i=0;i<76800;i++)		// 320 x 240 = 76800
	{
		Lcd_Write_Data(color+1);
	}
}


/*SET WORK AREA*/
//**************************************************************************************************//
void Lcd_Set_Work_Area(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	Lcd_Write_Reg(Horizontal_RAM_address_position,((x2<<8)|x1));
	Lcd_Write_Reg(Vertical_RAM_address_start_position,y1);
	Lcd_Write_Reg(Vertical_RAM_address_end_position,y2);
	
	Lcd_Set_Cursor(x1,y1);
	int i=0;
	for(i=0;i<(x2-x1+1)*(y2-y1+1);i++)		// same as Lcd_Clear()
	{
		Lcd_Write_Data(color+1);
	}
	Lcd_Set_Cursor(x1,y1);
}




/*PRINT CHAR*/
//**************************************************************************************************//
void Lcd_Print_Char( uint16_t x, uint16_t y, uint16_t color, uint16_t bcolor, uint8_t character, uint8_t size)
{
	uint8_t j,i,k,p = 0;
	
	for( j=0; j<8; j++)					// 8 cycles because - 8 words per character
	{
		for( k=0; k< size; k++)				// point size
		{			  
			Lcd_Set_Cursor(x+(j*size)+k, y); 
			for( i=0; i<8; i++)			// 8 cycles because - 8 bits per words.
			{
				if(0x01 & ((ASCIItable[character][j]) >> (7 - i))) 
				{
					for( p=0; p< size; p++)
					{
						Lcd_Write_Data(color);
					}
				}
				else 
				{
					for(p=0; p< size; p++)
					{
						 Lcd_Write_Data(bcolor);
					}
				}
			}
		}
	}
}


/*PRINT STRING*/
//**************************************************************************************************//
void Lcd_Print_String(uint16_t x, uint16_t y, uint16_t color, uint16_t bcolor,  uint8_t *s, uint8_t size)
{
	y=y-8*size;	// Od y oduzmem size zato jer cu ga u prvom loop-u povecati za size, tako da nema probema ako je na pocetku negativan
	while (*s)
	{
		Lcd_Print_Char(x,y=y+8*size,color,bcolor,*s++,size);		// Print character, increment character and increment y cursor by 8
	}
}


/*DRAW IMAGE*/
//**************************************************************************************************//
void Lcd_Draw_Image(const uint16_t *ptr_image)
{
  uint32_t i = 0;
	
	Lcd_Set_Cursor(0x0000U,0x0000U);
  
	for(i = 0; i < 76800; i++)
  {
		Lcd_Write_Data(*(ptr_image++));
  }
}


/*DRAW POINT*/
//**************************************************************************************************//
void Lcd_Draw_Point(uint16_t x, uint16_t y, uint16_t color, uint8_t size)
{
	uint8_t i, j=0;

	for(j=0;j < size; j++)
	{
		Lcd_Set_Cursor(x+j, y);
		for(i=0;i < size;i++)
		{
			Lcd_Write_Data(color);
		}
	}
}


/*DRAW LINE*/
//**************************************************************************************************//
void Lcd_Draw_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t size)
{
	int deltaX = abs(x2 - x1);
	int deltaY = abs(y2 - y1);
	int signX = x1 < x2 ? 1 : -1;
	int signY = y1 < y2 ? 1 : -1;
	int error = deltaX - deltaY;
	
	for (;;)
	{
		Lcd_Draw_Point(x1,y1,color,size);
		
		if(x1 == x2 && y1 == y2)
		break;
		
		int error2 = error * 2;
		
		if(error2 > -deltaY)
		{
			error -= deltaY;
			x1 += signX;
		}
		
		if(error2 < deltaX)
		{
			error += deltaX;
			y1 += signY;
		}
	}
}


/*Delay function*/
//**************************************************************************************************//
void Delay_m(uint32_t ms) 
{
	uint32_t i=0;
	while(i<=(ms*2666)){
		i++;
	}
	i=0;
}

