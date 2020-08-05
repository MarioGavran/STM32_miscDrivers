//*****************************************************************************************
/******************************************************************************************
LIBRARY:	ADC1.c
			- Only for STM32F410RBT8 devices.
			- Declares function for initializing the interface and function 
			  for single ADC conversion

AUTHOR: 	Mario Gavran, E5029604, University of Maribor - FERI
*******************************************************************************************/
//******************************************************************************************

#include "stm32f4xx.h"		// Device header
#include "UART2.h"		// UART2 header file
#include "ADC1.h"		// ADC1 header

//******************************************************************************************
//*************************************|              |*************************************
void initADC1(void)
{
	RCC->AHB1ENR	|= RCC_AHB1ENR_GPIOAEN;			// Enable clock on GPIOA
	RCC->APB2ENR	|= RCC_APB2ENR_ADC1EN;			// Enable clock on ADC1 

  	ADC->CCR	|= ADC_CCR_ADCPRE_1;			// Set /6 prescaler.
  	ADC1->CR2	|= ADC_CR2_ADON;			// Turn on the ADC
  	ADC1->CR1	&= ~(ADC_CR1_RES_1 | ADC_CR1_RES_0);	// 12 bit resolution
		
	
	// L[3:0], Define total number of conversions in the regular channel conversion sequence. 
	ADC1->SQR1	&= ~(ADC_SQR1_L_Msk);			// [1111: 16 conversions]
  
	
	// SQ1[4:0], Assign any channel as the 1st in the sequence to be converted.
	ADC1->SQR3	|= ADC_SQR3_SQ1_2;			// [01000: Channel 4]
	
	
	// Set selected channel GPIO pin(PA4) in analog mode
	GPIOA->MODER	|= GPIO_MODER_MODER4;			// [11: Analog mode]
}
//******************************************************************************************
//******************************************************************************************



//******************************************************************************************
//*************************************|              |*************************************
unsigned int ADC1_single_conversion(void)
{
	
	ADC1->CR2	|= ADC_CR2_SWSTART;			// Start conversion of regular channels.
	int c=0;
	while(c<4000)c++;					// Delay
	return (ADC1->DR)&(0xFFF);				// Return 8 bit result.	
}
//******************************************************************************************
//******************************************************************************************

