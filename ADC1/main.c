#include "stm32f4xx.h"			// Device header
#include "UART2.h"			// UART2 header
#include "ADC1.h"			// ADC1 header


void initUART2(void);
void u2_sendStr(unsigned char *s);
void initADC1(void);
unsigned int ADC1_read(void);

int main(){

 

	/*
	RCC_APB1ENR |= RCC_APB1ENR_TIM5EN;  //ukljuci clock na time
	//TIM5_PSC     = 158;
	TIM5_ARR     = 0xFFFF;              //postavi frekvenciju
	TIM5_CCR1    = 0xFF;
	TIM5_CCMR1  |= TIM_CCMR1_OC1PE;
	TIM5_CR1    |= TIM_CR1_ARPE;
	TIM5_CCMR1  |= TIM_CCMR1_OC1M_1;
	TIM5_CCMR1  |= TIM_CCMR1_OC1M_2;
	TIM5_EGR    |= TIM_EGR_UG;
	TIM5_CCER   |= TIM_CCER_CC1E;
	TIM5_CR1    |= TIM_CR1_CEN;                 */
    
	initADC1();
	initUART2();
	unsigned char a=0;
   
	while(1)
	{
		a=ADC1_single_conversion();
		u2_sendCh(a);
	}
	return 0;
}

