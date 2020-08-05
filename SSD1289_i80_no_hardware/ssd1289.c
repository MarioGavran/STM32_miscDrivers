/*****************************************************************************************************
LIBRARY: 	ssd1289.c

			- only for STM32F410RBT8 devices used with SSD1289 TFT LCD screen.

			- includes: 
 				* function for initializing GPIO pins used for i8080 parallel interface.
				* functions for reading data from bus, writing data/command to the bus 
				* function for initializing SSD1289 TFT controller

AUTHOR: 	Mario Gavran, E5029604, University of Maribor - FERI
*****************************************************************************************************/


#include "stm32f4xx.h"		// STM32 device header
#include "ssd1289.h"		// TFT controller header
#include <stdint.h>		// Standard int header
#include <stdlib.h>		// Standard library header

#define lcd_rd 		5	// PA5 - Read
#define lcd_wr		6	// PA6 - Write
#define lcd_rs		7	// PA7 - Register select - DC in datasheet (command - 0, data - 1)
#define lcd_cs 		8	// PA8 - Chip select
#define lcd_rst		9	// PA9 - Reset



//**************************************************************************************************//
void	Init_GPIO_SSD1289(void)
{
	// clock
	RCC->AHB1ENR	|=RCC_AHB1ENR_GPIOBEN;	// clock on GPIOB
	RCC->AHB1ENR	|=RCC_AHB1ENR_GPIOAEN;	// clock on GPIOC
	
	// COMMAND bus PA0...PA4
	GPIOA->MODER	|=0x55400U;		// Output mode	(0b 0000 0000 0000 0101 0101 0100 0000 0000 )
	GPIOA->PUPDR 	|=0x00000U;		// NO pull-UP/DOWN resistors
	GPIOA->OTYPER	|=0x00U;		// Push-pull (reset state)
	GPIOA->ODR	|=0x3E0U;		// Initial output state of COMMAND pins (0b 0000 0000 0000 0000 0000 0011 1110 0000 )
	
	// DATA bus PB0...P15
	GPIOB->MODER	=0x55555555U;		// output mode	(0b 0101 0101 0101 0101 0101 0101 0101 0101 )
	GPIOB->PUPDR 	=0x00000000U;		// NO pull-UP/DOWN resistors
	GPIOB->OTYPER	=0x0000U;		// Push-pull (reset state)
	GPIOB->ODR	=0x0000U;		// initial output state of DATA pins
}

//**************************************************************************************************//
void	Init_SSD1289(void)
{
	Delay_m(100);    			// short delay on power up
	
	GPIOA->ODR |= (1<<lcd_rst);     	// set ~RST, to disable reset state on SSD1289
	
	Lcd_Write_Reg (Sleep_mode, 		0x0000U);	// Release from Sleep
	Lcd_Write_Reg (Display_control,		0x0021U);   	// GON=1 DTE=0 D[1:0]=01
	Lcd_Write_Reg (Oscillation_Start,	0x0001U);	// Turn ON the oscilator
	Lcd_Write_Reg (Display_control,		0x0023U);	// GON=1 DTE=0 D[1:0]=11
	Lcd_Write_Reg (Sleep_mode,		0x0000U);	// Release from Sleep
	Delay_m(30);						// Wait 30ms
	Lcd_Write_Reg (Display_control,		0x0033U);	// GON=1 DTE=1 D[1:0]=11
	Lcd_Write_Reg (Entry_mode,		0x6838U);	// 65k color,  
	Lcd_Write_Reg (LCD_drive_AC_control,	0x0600U);
	
	
}

//************************************WRITE INDEX***************************************************//
void Lcd_Write_Index(uint16_t index)
{
	GPIOA->ODR &=~(1<<lcd_rs);
	GPIOA->ODR |= (1<<lcd_rd);
	
	GPIOA->ODR &=~(1<<lcd_cs);
	
	GPIOB->ODR = (index & 0xFFFFU);
	
	GPIOA->ODR &=~(1<<lcd_wr);
	Delay_u(5);
	GPIOA->ODR |= (1<<lcd_wr);
	
	GPIOA->ODR |=	(1<<lcd_cs);
}

//************************************WRITE DATA****************************************************//
void Lcd_Write_Data(uint16_t data)		//isto kao i Lcd_Write_Index samo je D/C(data/command) u '1'
{
	GPIOA->ODR |= (1<<lcd_rs);
	GPIOA->ODR |= (1<<lcd_rd);
	
	GPIOA->ODR &=~(1<<lcd_cs);		
		
	GPIOB->ODR = (data  & 0xFFFFU);
	
	GPIOA->ODR &=~(1<<lcd_wr);
	Delay_u(5);
	GPIOA->ODR |= (1<<lcd_wr);
	
	GPIOA->ODR |= (1<<lcd_cs);
}

//************************************WRITE TO LCD REGISTER*****************************************//
void Lcd_Write_Reg(uint16_t reg_addr, uint16_t reg_data)
{
	GPIOA->ODR &= ~(1<<lcd_cs);
	Lcd_Write_Index(reg_addr);
	Lcd_Write_Data(reg_data);
	GPIOA->ODR |= (1<<lcd_cs);
}

//**************************************READ DATA FROM BUS******************************************//
uint16_t Lcd_Read_Data(void)
{
	volatile uint16_t val = 0;
	
	GPIOA->ODR |=	(1<<lcd_rs);
	GPIOA->ODR &=~(1<<lcd_cs);
	GPIOA->ODR |=	(1<<lcd_wr);
	
	GPIOA->ODR &=~(1<<lcd_rd);
	
	Delay_u(10);
	val = GPIOB->IDR;
	
	GPIOA->ODR |=	(1<<lcd_rd);
	
	GPIOA->ODR |=	(1<<lcd_cs);
	
	return val;
}
//
//**************************************READ FROM REGISTER******************************************//
uint16_t Lcd_Read_Reg(uint16_t reg_addr)
{
	volatile uint16_t data = 0;
	
	GPIOA->ODR &= ~(1<<lcd_cs);
	Lcd_Write_Index(reg_addr);			// set index
	
	data = Lcd_Read_Data();				// read data from bus
	
	GPIOA->ODR |= (1<<lcd_cs);
	return data;
}
/*SET CURSOR*/
//**************************************************************************************************//
void Set_Cursor(uint16_t x_kur, uint16_t y_kur)
{	
	Lcd_Write_Reg(Set_GDDRAM_X_address_counter,x_kur);	// Initial GDDRAM X address
	Lcd_Write_Reg(Set_GDDRAM_Y_address_counter,y_kur);	// Initial GDDRAM Y address
	Lcd_Write_Index(RAM_data_write_read);								// Set index to RAM_data_write
}



//**************************************************************************************************//
void Lcd_Clear(uint16_t color)
{
	uint32_t	i = 0;
	
	Set_Cursor(0x0000U,0x0000U);
	
	for(i=0;i<76800;i++)		// 320 x 240 = 76800
	{
		Lcd_Write_Data(color+1);
	}
}



//**************************************************************************************************//
void Draw_Point (uint16_t x,uint16_t y,uint16_t color,uint8_t size)
{
	uint8_t i, j=0;

	for(j=0;j < size; j++)
	{
		Set_Cursor(x+j, y);
		for(i=0;i < size;i++)
		{
			Lcd_Write_Data(color);
		}
	}
}



//**************************************************************************************************//
void Print_Char( uint16_t x, uint16_t y, uint16_t color, uint16_t bcolor, uint8_t character, uint8_t size)
{
	uint8_t j,i,k,p = 0;
	
	for( j=0; j<8; j++)								// 8 cycles because - 8 words per character
	{
		for( k=0; k< size; k++)					//size - point size
		{			  
			Set_Cursor(x+(j*size)+k, y);	// 
			for( i=0; i<8; i++)						// 8 cycles because - 8 bits per words.
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



//**************************************************************************************************//
void Print_String(uint16_t x, uint16_t y, uint16_t color, uint16_t bcolor,  uint8_t *s, uint8_t size)
{
	y=y-8*size;
	while (*s)
	{
		Print_Char(x,y=y+8*size,color,bcolor,*s++,size);	// Print character, increment character and increment y cursor by 8
	}
}



//**************************************************************************************************//
void Draw_Line (uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color, uint8_t size)
{
	int deltaX = abs(x2 - x1);
	int deltaY = abs(y2 - y1);
	int signX = x1 < x2 ? 1 : -1;
	int signY = y1 < y2 ? 1 : -1;
	int error = deltaX - deltaY;
	
	for (;;)
	{
		Draw_Point(x1,y1,color,size);
		
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



//**************************************************************************************************//
void Set_Work_Area(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	Lcd_Write_Reg(Horizontal_RAM_address_position,((x2<<8)|x1));
	Lcd_Write_Reg(Vertical_RAM_address_start_position,y1);
	Lcd_Write_Reg(Vertical_RAM_address_end_position,y2);
	
	Set_Cursor(x1,y1);
	int i=0;
	for(i=0;i<(x2-x1+1)*(y2-y1+1);i++)		// same as Lcd_Clear()
	{
		Lcd_Write_Data(color+1);
	}
	Set_Cursor(x1,y1);
}




void Draw_Image( const uint16_t *ptr_image)
{
  uint32_t i = 0;
	
	Set_Cursor(0x0000U,0x0000U);
  
	for(i = 0; i < 76800; i++)
  {
		Lcd_Write_Data(*(ptr_image++));
  }
}



//**************************************************************************************************//
void Delay_m(uint32_t ms)
{
	uint32_t i=0;
	while(i<=(ms*8000)){
		i++;
	}
	i=0;
}
//**************************************************************************************************//
//**************************************************************************************************//
void Delay_u(uint32_t us)
{
	uint32_t i=0;
	while(i<=(us*8)){
		i++;
	}
	i=0;
}

