/************************************************************************************************************************
LIBRARY:	USART1_STM32F413ZH.c.c

			- transmitter and reciever trough USART1 with virtual COM port.

			- only for STM32F410RBT8 devices.

			- declares functions for initializing the interface, sending chars, sending strings and recieving chars.

AUTHOR:		Mario Gavran, E5029604, University of Maribor - FERI
*************************************************************************************************************************/

#include "stm32f4xx.h"				// Device header.
#include "USART1_STM32F413ZH.h"			// UART2 header file.


void initUSART1(void)
{
	// Clock initialization for USART1 peripheral and GPIO pins:
	RCC->APB2ENR 	|= RCC_APB2ENR_USART1EN;	//enable USART1
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOAEN;	//enable PORTA
	
	
	
	// Configure USART1 GPIO pins: 
	GPIOA->MODER	|= GPIO_MODER_MODER9_1;			// Set PA9 as AF.
	GPIOA->MODER	&= ~(GPIO_MODER_MODER9_0);		// Set PA9 as AF.
	GPIOA ->AFR[1]	|= (GPIO_AFRH_AFSEL9_0 
			| GPIO_AFRH_AFSEL9_1 
			| GPIO_AFRH_AFSEL9_2);			// Turn on gpio AF7  on PA9.(TX)
	GPIOA ->AFR[1]	&= ~(GPIO_AFRH_AFSEL9_3);
	
	
	GPIOA->MODER 	|= GPIO_MODER_MODER10_1;		// Set PA10 as AF.
	GPIOA->MODER	&= ~(GPIO_MODER_MODER10_0);		// Set PA10 as AF.
	GPIOA ->AFR[1] 	|= (GPIO_AFRH_AFSEL10_0 
			| GPIO_AFRH_AFSEL10_1 
			| GPIO_AFRH_AFSEL10_2); 		// Turn on gpio AF7  on PA10.(RX)
	GPIOA ->AFR[1]	&= ~(GPIO_AFRH_AFSEL10_3);
	
	// Configure timings and enable:
	
	USART1->CR1 |= USART_CR1_UE;				// USART enable.
	USART1->BRR |= 1667;					// 9600bps.
	USART1->CR1 |= USART_CR1_TE;				// Transmitter enable.
	USART1->CR1 |= USART_CR1_RE;				// Reciver enable.

}
//************************************************************************************************************************
//**************************************************|     SEND CHAR    |**************************************************
void USART1_sendCh (unsigned char ch)
{
	while (!(USART1->SR & USART_SR_TXE)){}
	USART1->DR = ch;
}
//************************************************************************************************************************



//************************************************************************************************************************
//**************************************************|    SEND STRING   |**************************************************
void USART1_sendStr(unsigned char *s)
{
	while (*s)
	{
		USART1_sendCh(*s++);
	}
}
//************************************************************************************************************************



//************************************************************************************************************************
//**************************************************|    RECIVE CHAR   |**************************************************
unsigned char  USART1_getCh(void)
{
	while (!(USART1->SR & USART_SR_RXNE));
	return (int)USART1->DR;
}
//************************************************************************************************************************

