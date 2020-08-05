#include "stm32l0xx.h"			// Device header.
#include "delay_TIM2.h"			// delay_TIM2 header file.


void delay_TIM2_init(void)
{
	RCC->APB1ENR	|= RCC_APB1ENR_TIM2EN;
	
	TIM2->PSC 	= 0;				// prescaler set to 0
	TIM2->ARR 	= 2000;				// auto-reload reg. (ARR=16) reload every 1us
	TIM2->CR1	|= TIM_CR1_URS;			// (Update-interupt Request Source) (URS=1) Only counter-overflow is source.
	TIM2->DIER	|= TIM_DIER_UIE;		// (DIER-DMA Interupt Enable Reg.) (UIE=1) Update-interupt enabled.
	TIM2->EGR	|= TIM_EGR_UG;			// (EGR-Event Generation Reg.) (UG=1) Generates an update.
	
	NVIC_EnableIRQ(TIM2_IRQn);			// NVIC  function to enable interupt on TIM2
}


void delay_TIM2_ms(unsigned int val) {
	TIM2->CR1	|= TIM_CR1_CEN;			// starts the timer
	ticks=0;					// reset ticks variable on every instance of delay fnc.
	while(ticks <= (val));				// wait until ticks=val
	TIM2->CR1	&= ~(TIM_CR1_CEN);		// stops the timer
}

void TIM2_IRQHandler(void)
{
	ticks++;					// ticks++ on every interupt
	TIM2->SR	&= ~(TIM_SR_UIF);		// reset update interupt flag
}

