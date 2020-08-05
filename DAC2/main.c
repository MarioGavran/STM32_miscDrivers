#include "stm32f4xx.h"			// Device header
#include <math.h>

int main(void){
	
	RCC->AHB1ENR	|= RCC_AHB1ENR_GPIOAEN;		// enable clock on PORTA
	
	GPIOA->MODER	|= GPIO_MODER_MODE5_0;		// set analog mode on PA5(BIT0)
	GPIOA->MODER	|= GPIO_MODER_MODE5_1;		// set analog mode on PA5(BIT1)
	
	RCC->APB1ENR	|= RCC_APB1ENR_DACEN;		// enable clock on DAC
	
	DAC->CR		&= ~(DAC_CR_BOFF1);		// enable output buffer
	//DAC->CR	|= DAC_CR_BOFF1;		// disable output buffer
	
	//DAC->CR	|= DAC_CR_TEN1;			// enable external trigger
	DAC->CR		&= ~(DAC_CR_TEN1);		// disable external trigger/conversation starts DAC_DHRx REG IS LOADED
	
	DAC->CR		|= DAC_CR_EN1;			// enable dac
	
	int a	= 0;
	int c	= 0;
	while(1){
		a=a+0x3;
		DAC->DHR12R1	= (a & 0xFFF);		// load data to DATA HOLD REGISTER
		
		while(c<40)	c++;			// delay a few clock cycles
		c=0;
		
		if(a>=0xFFF)	a=0;			//restart value for DATA HOLD REGISTER
	}
	return 0;
}

