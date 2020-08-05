#include "stm32f4xx.h"		// Device header
#include "uart6.h"		// UART2 header file


void initUART6(void){
	RCC->APB2ENR 	|= RCC_APB2ENR_USART6EN;	//enable USART2
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOCEN;		//enable PORTC
	
	
	GPIOC->MODER	|= GPIO_MODER_MODER6_1;		//Set PC6 AF
	GPIOC->AFR[0]	|= GPIO_AFRL_AFRL6_3; 		// turn on gpio AF8  on PC6 TX
	GPIOC->MODER 	|= GPIO_MODER_MODER7_1;		//Set PC7 AF
	GPIOC ->AFR[0] 	|= GPIO_AFRL_AFRL7_3; 		// turn on gpio AF8  on PC7 RX

	USART6->CR1	|= USART_CR1_UE;
	USART6->BRR	|= 1667;
	USART6->CR1	|= USART_CR1_TE;
	USART6->CR1	|= USART_CR1_RE; 	
}

void u6_sendCh (unsigned char ch){
	while (!(USART6->SR & USART_SR_TXE)){}
	USART6->DR = ch;
}

void u6_sendStr(unsigned char *s ){
	while (*s){
		u6_sendCh(*s++);
	}
}

unsigned char  u6_getCh(void){
	while (!(USART6->SR & USART_SR_RXNE));
	return (int)USART6->DR;
}

