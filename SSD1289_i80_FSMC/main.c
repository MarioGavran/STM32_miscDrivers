#include "stm32f4xx.h"			// Device header
#include "SSD1289_FSMC_STM32.h"
#include <stdint.h>
#include <math.h>
#include "mandelbrot.h"
#include "USART1_STM32F413ZH.h"		// UART2 header file.

void Delay_mm(unsigned int ms);

int main(void)
{	
	// Clock initialization for USART1 peripheral and GPIO pins:
	RCC->AHB1ENR	|= RCC_AHB1ENR_GPIOCEN;		//enable PORTA
	
	GPIOC->MODER	|= GPIO_MODER_MODER9_1;		// Set PC9 as AF.
	GPIOC->MODER	&= ~(GPIO_MODER_MODER9_0);	// Set PC9 as AF.
	
	GPIOC ->AFR[1]	&= ~(GPIO_AFRH_AFSEL9);		// Turn on gpio AF0  on PC9.(MCO2)
	
	//RCC->CFGR	|= (RCC_CFGR_MCO2_1);		
	RCC->CFGR	&= ~(RCC_CFGR_MCO2);		// System clock (SYSCLK) selected.
	RCC->CFGR	&= ~(RCC_CFGR_MCO2PRE);		// No prescaler.
	
	initUSART1();
	
	ssd1289_FSMC_GPIO_init();
	ssd1289_FSMC_init();
	ssd1289_Driver_init();
	
	// Clear the screen and set frame and axis:
	Lcd_Clear(blue);
	Lcd_Draw_Line(5,5,5,315,red,2);			// Horizontal top line
	Lcd_Draw_Line(235,5,235,315,red,2);		// Horizontal bottom 
	Lcd_Draw_Line(5,5,235,5,red,2);			// Vertical right line
	Lcd_Draw_Line(5,315,235,315,red,2);		// Vertical left line
	Lcd_Draw_Line(120,5,120,315,red,2);		// Vertical axis
	Lcd_Draw_Line(5,160,235,160,red,2);		// Horizontal axis
		
	
	// Generating test signal:
	#define pi 3.141592653
	int N=310;					// Test signal period of 310 samples
	float Signal[310]={0};
	int n=0;
	for(n=0;n<N;n++){
		Signal[n] = 15*cos((4*(2*pi)/N)*n)+15*sin((8*(2*pi)/N)*n);
		Lcd_Draw_Point(-Signal[n]+120,5+n,black,2);
	}
	
	// Calculating first 10 coefficients of DFT:
	int k=0;
	float Xre[21],Xim[21]={0};
	for(k=-10;k<=10;k++){
		for(n=0;n<N;n++){
			Xre[k+10] = Xre[k+10] + Signal[n]*cos(((2*pi)/N)*k*n);
			Xim[k+10] = Xim[k+10] + Signal[n]*sin(((2*pi)/N)*k*n);
		}
		Xre[k+10]=Xre[k+10]/31;
		Xim[k+10]=Xim[k+10]/31;
	}
	for(k=0;k<21;k++){
		Lcd_Draw_Point(Xre[k]+120,5+k*15,white,5);
		Lcd_Draw_Point(Xim[k]+120,5+k*15,green,5);
	}
	Delay_m(3500);
	
	
	while(1)
	{
		// Clear the screen and set frame and axis:
		Lcd_Clear(blue);
		Lcd_Draw_Line(5,5,5,315,red,2);			// Horizontal top line
		Lcd_Draw_Line(235,5,235,315,red,2);		// Horizontal bottom 
		Lcd_Draw_Line(5,5,235,5,red,2);			// Vertical right line
		Lcd_Draw_Line(5,315,235,315,red,2);		// Vertical left line
		Lcd_Draw_Line(120,5,120,315,red,2);		// Vertical axis
		Lcd_Draw_Line(5,160,235,160,red,2);		// Horizontal axis
		/*
		int i=0;
		float s=0;
		for(i=0;i<=310;i++)
			{
				s=(-30*sin(0.0202*i)+120);
				Lcd_Draw_Point(s,5+i,black,2);
			}
		*/
		Delay_m(800);
		// moving wave
		float w = 1;
		float s=0;
		int i,j=0;
		/*for(j=0;j<=310;j++)
		{
			for(i=0;i<=310;i++)
			{
				s=(-50*sin(0.0202*i+w*j)+120);
				Lcd_Draw_Point(s,5+i,black,2);
			}
			for(i=0;i<=310;i++)
			{
				s=(-50*sin(0.0202*i+w*j)+120);
				Lcd_Draw_Point(s,5+i,blue,2);
				Lcd_Draw_Point(120,5+i,red,2);
				Lcd_Draw_Point(s,160,red,2);
			}
		}
		*/
		
		Lcd_Clear(blue);
		for(j=5;j<=310;j++)
		{
			for(i=5;i<=235;i++)
			{
				s=50*sin(0.202*i+0.202*j);
				if(s >= (float)0.9)
				{
					Lcd_Draw_Point(i,j,black,2);
				}
			}
		}
		Lcd_Clear(blue);
		
		Delay_m(100);
		Lcd_Print_String(0,0,black,blue,"Zdravo svijete",2);
		Delay_m(1000);
	}
}

