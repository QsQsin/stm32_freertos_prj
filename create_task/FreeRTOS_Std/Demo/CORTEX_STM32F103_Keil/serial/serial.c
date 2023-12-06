/* Standard includes. */
#include <stdio.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x_lib.h"

/* Demo application includes. */
#include "serial.h"

/*
 * See the serial2.h header file.
 */
void SerialPortInit(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	unsigned long ulWantedBaud=11520;

	/* If the queue/semaphore was created correctly then setup the serial port
	hardware. */
	/* Enable USART1 clock */
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE );

		/* Configure USART1 Rx (PA10) as input floating */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init( GPIOA, &GPIO_InitStructure );

		/* Configure USART1 Tx (PA9) as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init( GPIOA, &GPIO_InitStructure );

		USART_InitStructure.USART_BaudRate = ulWantedBaud;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_InitStructure.USART_Clock = USART_Clock_Disable;
		USART_InitStructure.USART_CPOL = USART_CPOL_Low;
		USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
		USART_InitStructure.USART_LastBit = USART_LastBit_Disable;

		USART_Init( USART1, &USART_InitStructure );

		USART_Cmd( USART1, ENABLE );

	/* This demo file only supports a single port but we have to return
	something to comply with the standard demo header file. */
}

int fputc( int ch, FILE *f )
{
	USART_TypeDef* USARTx=USART1;
	while((USARTx->SR & (1<<7)) == 0);
	USARTx->DR=ch;
	return ch;
}








