/*
 * serialEvent.cpp
 *
 *  Created on: Aug 25, 2022
 *      Author: sanjay
 */


#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "cmsis_os.h"
#include "task.h"
#include "serialDebug.h"
#include "serialEvent.h"

extern UART_HandleTypeDef huart2;
osMessageQueueId_t userEventQueue;

unsigned char userCommBuf[32],userCommBuf_[32];
unsigned char userComBufIndex = 0;
unsigned char userComAvailable = 0;


void userEventHandlerInit()
{
	userEventQueue = osMessageQueueNew(16, sizeof(userEvent_t), 0);

	huart2.Instance->CR1 |= USART_CR1_RXNEIE;

	NVIC_EnableIRQ(USART2_IRQn);
}

void userEventHandler()
{
	unsigned char i = 0;
	unsigned int steps;
	userEvent_t ev;
	if (userComAvailable)
	{
		userComAvailable = 0;

		ev.type = userCommBuf[i++];
		ev.value = atoi((char*)&userCommBuf[i]);

		if (osMessageQueuePut(userEventQueue, &ev, 0, 0) != osOK)
		{
			print("user event failed\n");
		}
	}
}

unsigned char userEventAvailable()
{
	return osMessageQueueGetCount(userEventQueue);
}

unsigned char getUserEvent(userEvent_t *ev)
{
	if (osMessageQueueGet(userEventQueue, ev, 0, portMAX_DELAY) == osOK)
		return 1;

	return 0;
}

extern "C" void USART2_IRQHandler()
{
	if (USART2->ISR & USART_ISR_RXNE)
	{
		unsigned char chr = USART2->RDR;

		if (chr == '\n')
		{
			userCommBuf_[userComBufIndex++] = 0;
			memcpy(userCommBuf, userCommBuf_, userComBufIndex);
			userComAvailable = 1;
			userComBufIndex = 0;
		}
		else if (userComBufIndex < 31)
		{
			userCommBuf_[userComBufIndex++] = chr;
		}

	}
}

