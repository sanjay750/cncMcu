/*
 * serialDebug.cpp
 *
 *  Created on: Aug 22, 2022
 *      Author: sanjay
 */


#include "main.h"
#include "cmsis_os.h"
#include "task.h"


#include "serialDebug.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;



osMessageQueueId_t debugEventQueue;

void debugThread(void *arg);
osThreadId_t debugThreadId;

const osThreadAttr_t debugThreadAttr = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

DMA_HandleTypeDef sdbDMA;


void serialDebugInit()
{
	__HAL_RCC_DMA1_CLK_ENABLE();




	sdbDMA.Instance = DMA1_Channel4;
	sdbDMA.DmaBaseAddress = DMA1;
	sdbDMA.ChannelIndex = 4;

	sdbDMA.Init.Mode = DMA_NORMAL;
	sdbDMA.Init.Direction = DMA_MEMORY_TO_PERIPH;
	sdbDMA.Init.PeriphInc = DMA_PINC_DISABLE;
	sdbDMA.Init.MemInc = DMA_MINC_ENABLE;
	sdbDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	sdbDMA.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	sdbDMA.Init.Priority = DMA_PRIORITY_LOW;

	HAL_DMA_Init(&sdbDMA);

	debugEventQueue = osMessageQueueNew(16, sizeof(">000000")-1, NULL);
	configASSERT(debugEventQueue);

	debugThreadId = osThreadNew(debugThread, NULL, &debugThreadAttr);
	configASSERT(debugThreadId);


}



void debugThread(void *arg)
{
	unsigned char msg[16];
	while(1)
	{

		if (osMessageQueueGet(debugEventQueue, msg, 0, portMAX_DELAY) == osOK)
		{
			sdbDMA.Instance->CCR &= ~DMA_CCR_EN;

			sdbDMA.Instance->CMAR = (uint32_t)msg;
			sdbDMA.Instance->CPAR = (uint32_t)&huart1.Instance->TDR;
			sdbDMA.Instance->CNDTR = sizeof(">000000")-4;

			sdbDMA.Instance->CCR |= DMA_CCR_EN;

			huart1.Instance->CR3 |= USART_CR3_DMAT;

			while(!(DMA1->ISR & DMA_ISR_TCIF4))
				osThreadYield();
			DMA1->IFCR = DMA_ISR_TCIF4;

		}
	}
}


