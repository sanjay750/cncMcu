/*
 * serialComHandler.cpp
 *
 *  Created on: Aug 26, 2022
 *      Author: sanjay
 */




#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "stdio.h"
#include "string.h"

#include "serialComHandler.h"

extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart4;
DMA_HandleTypeDef txDma, rxDma;




QueueHandle_t serialComHandler_t::eventQueue;
serialComHandler_t* serialComHandler_t::handlers[256];



uint8_t rxComBuf[1024];
uint32_t rxLen;
comEvt_t rxEvt;
uint8_t *nextPayloadPtr = rxComBuf;

uint8_t loopBackComBuf[1024];




StackType_t serialComTaskStack[512];
StaticTask_t serialComTask;


const osThreadAttr_t test_attributes = {
  .name = "comHandler",
  .cb_mem = &serialComTask,
  .cb_size = sizeof(serialComTask),
  .stack_mem = &serialComTaskStack[0],
  .stack_size = sizeof(serialComTaskStack),
  .priority = (osPriority_t) osPriorityNormal,
};


uint8_t comWatchDog_txPayload[34];
uint8_t comWatchDog_rxPayload[34];
uint8_t comWatchDog_respPayload[34];

int comWatchDogResp(serialComHandler_t*com, uint8_t *data, unsigned int len);

serialComHandler_t comWatchDog =
{
	'W',
	comWatchDogResp,
};

int comWatchDogResp(serialComHandler_t*com, uint8_t *data, unsigned int len)
{
	memcpy(com->respPayload, "from watch dog", sizeof("from watch dog"));
	com->respLen = sizeof("from watch dog");
	return sizeof("from watch dog");
}




uint32_t refLed, refAct;
uint8_t serialAct_ = 0;
#define serialAct()	do{\
	if (serialAct_)\
	{\
		refLed = HAL_GetTick();\
		refAct = refLed;\
		HAL_GPIO_WritePin(SERIAL_COM_GPIO, SERIAL_COM_PIN, GPIO_PIN_SET);\
	}\
	else\
	{\
		refAct = HAL_GetTick();\
	}\
	serialAct_ = 1;\
	}while(0)

void serialActLedHandler()
{
	uint32_t tick = HAL_GetTick();
	if (serialAct_)
	if (tick - refLed >= 50)
	{
		HAL_GPIO_TogglePin(SERIAL_COM_GPIO, SERIAL_COM_PIN);
		refLed = tick;

		if (tick - refAct >= 50)
		{
			serialAct_ = 0;
			HAL_GPIO_WritePin(SERIAL_COM_GPIO, SERIAL_COM_PIN, GPIO_PIN_RESET);
		}

	}
}




void serialComHandlerInit()
{
	serialComHandler_t::eventQueue = osMessageQueueNew(8, sizeof(void *), 0);
	xTaskCreateStatic(serialComHandlerTask, "comHandler", 512, 0, osPriorityNormal, serialComTaskStack, &serialComTask);

	serialComHandler_t::comInit(&comWatchDog, comWatchDog_rxPayload, comWatchDog_txPayload, comWatchDog_respPayload);

	__HAL_RCC_DMA2_CLK_ENABLE();

	txDma.DmaBaseAddress = DMA2;
	txDma.ChannelIndex = 5;
	txDma.Instance = DMA2_Channel5;


	txDma.Init.Mode = DMA_NORMAL;
	txDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
	txDma.Init.PeriphInc = DMA_PINC_DISABLE;
	txDma.Init.MemInc = DMA_MINC_ENABLE;
	txDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	txDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	txDma.Init.Priority = DMA_PRIORITY_MEDIUM;
	HAL_DMA_Init(&txDma);



	rxDma.DmaBaseAddress = DMA2;
	rxDma.ChannelIndex = 3;
	rxDma.Instance = DMA2_Channel3;

	rxDma.Init.Mode = DMA_NORMAL;
	rxDma.Init.Direction = DMA_PERIPH_TO_MEMORY;
	rxDma.Init.PeriphInc = DMA_PINC_DISABLE;
	rxDma.Init.MemInc = DMA_MINC_ENABLE;
	rxDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	rxDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	rxDma.Init.Priority = DMA_PRIORITY_MEDIUM;
	HAL_DMA_Init(&rxDma);

	UART4->CR1 &= ~USART_CR1_UE;
	UART4->RTOR = 10*4;
	UART4->CR2 |= USART_CR2_RTOEN;
	UART4->CR3 |= USART_CR3_DMAR;
	UART4->CR1 |= USART_CR1_RTOIE | USART_CR1_UE;

	rxDma.Instance->CCR &= ~DMA_CCR_EN;
	rxDma.Instance->CMAR = (unsigned int) rxComBuf;
	rxDma.Instance->CPAR = (unsigned int) &UART4->RDR;
	rxDma.Instance->CNDTR = sizeof(rxComBuf);
	rxDma.Instance->CCR |= DMA_CCR_EN;


	__NVIC_SetPriority(UART4_IRQn, 0xE);
	NVIC_EnableIRQ(UART4_IRQn);
	UART4->CR1 |= USART_CR1_UE;

	__NVIC_SetPriority(DMA2_Channel5_IRQn, 0xE);
	NVIC_EnableIRQ(DMA2_Channel5_IRQn);

}

TaskHandle_t dmaTask;

void sendCharViaDma(uint8_t *ptr, uint32_t len)
{
//    uint8_t i = 0;
//    print("TX->:");
//    while (i<15) {printF("0x%02X,",ptr[i++]);}
//    print("\n");

	txDma.Instance->CCR &= ~DMA_CCR_EN;
	txDma.Instance->CMAR = (uint32_t) ptr;
	txDma.Instance->CPAR = (uint32_t) &UART4->TDR;
	txDma.Instance->CNDTR = len;

	UART4->CR3 |= USART_CR3_DMAT;
	txDma.Instance->CCR |= DMA_CCR_EN | DMA_CCR_TCIE;

	dmaTask = osThreadGetId();
	serialAct();
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	xTaskNotifyWait(0, UINT32_MAX, 0, 2000);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

}



extern "C" void DMA2_Channel5_IRQHandler_()
{
	if (DMA2->ISR & DMA_ISR_TCIF5)
	{
		DMA2->IFCR = DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CGIF5;
		if (dmaTask != 0)
			xTaskNotifyFromISR(dmaTask, 1, eSetBits, 0);
	}
}



extern "C" void UART4_IRQHandler_()
{
	if ((UART4->CR1 & USART_CR1_RTOIE) && (UART4->ISR & USART_ISR_RTOF))
	{
		UART4->ICR = USART_ICR_RTOCF;
//		GPIOC->ODR |= GPIO_PIN_9;

		if (UART4->ISR & USART_ISR_ORE)
			UART4->ICR = USART_ISR_ORE;


		rxDma.Instance->CCR &= ~DMA_CCR_EN;

		rxEvt.sig = OUTPUT_FROM_COMP;
		rxEvt.payload = rxComBuf;
		rxEvt.len = sizeof(rxComBuf) - rxDma.Instance->CNDTR;
		comEvt_t *ev = &rxEvt;

		osMessageQueuePut(serialComHandler_t::eventQueue, &ev, 0, 0);

		rxDma.Instance->CMAR = (unsigned int) rxComBuf;
		rxDma.Instance->CPAR = (unsigned int) &UART4->RDR;
		rxDma.Instance->CNDTR = sizeof(rxComBuf);

		rxDma.Instance->CCR |= DMA_CCR_EN;

//		GPIOC->ODR &= ~GPIO_PIN_9;


	}
}





void serialComHandlerTask(void *args)
{
	comEvt_t *ev;
	unsigned int len;

	uint16_t i = 0, j = 0;
	while(1)
	{
		if (osMessageQueueGet(serialComHandler_t::eventQueue, &ev, 0, portMAX_DELAY) != osOK)
			continue;

		switch(ev->sig)
		{
		case OUTPUT_FROM_COMP:

//		    i = 0;
//		    print("RX->:");
//		    while (i<15) {printF("0x%02X,",ev->payload[i++]);}
//		    print("\n");
//			printF("rxbyt %d\n", ev->len);
			len = serialComHandler_t::comHandler(ev->payload, ev->len);



			if (len == 0)
				break;
			for (i = 0, j = 0; i < len; i++, j++)
			{

				loopBackComBuf[j] = ev->payload[i];
				if(ev->payload[i] == '\r')
					loopBackComBuf[++j] = ev->payload[i];
			}
			loopBackComBuf[j++] = '\r';
			loopBackComBuf[j++] = '\n';
			sendCharViaDma(loopBackComBuf, j);

			break;
		case SEND_TO_COMP:

			for (i = 0, j = 0; i < ev->len; i++, j++)
			{

				loopBackComBuf[j] = ev->payload[i];
				if(ev->payload[i] == '\r')
					loopBackComBuf[++j] = ev->payload[i];
			}
			loopBackComBuf[j++] = '\r';
			loopBackComBuf[j++] = '\n';
			sendCharViaDma(loopBackComBuf, j);
			break;

		case LOOP_BACK_COMP:
			memcpy(loopBackComBuf, ev->payload, ev->len);
			len = serialComHandler_t::comHandler(loopBackComBuf, ev->len);
			if (len > 0)
				serialComHandler_t::comHandler(loopBackComBuf, ev->len);
			break;
		}
	}
}


int serialComHandler_t::comInit(serialComHandler_t *handler, unsigned char *rxPayload, unsigned char *txPayload, unsigned char *ackPayload)
{
	if (handler == 0)
		return 0;


	handlers[handler->ID] = handler;

	handler->txPayload = txPayload;
	handler->respPayload = ackPayload;
	handler->rxPayload = rxPayload;

	handler->txPacketId = 0xFFFF;

	return 1;
}


int serialComHandler_t::comHandler(unsigned char *data, unsigned int len)
{

	int l = cast(uint16_t,data[0]);


	if (l > len)
		return 0;




	while (hcrc.State == HAL_CRC_STATE_BUSY) osThreadYield();
	uint16_t crc = HAL_CRC_Calculate(&hcrc, &cast(uint32_t, data[ACK_CHAR_INDEX]), l);//remove len from packet

	if (crc != cast(uint16_t, data[l+2]))
		return 0;

	serialAct();


	if (data[ACK_CHAR_INDEX] == '\a')
	{
		if (handlers[data[ACK_HANDLER_ID_INDEX]] != 0 && len < 1024)
		{
			serialComHandler_t *ntemp = handlers[data[ACK_HANDLER_ID_INDEX]];

			if (ntemp->txPacketId != cast(unsigned short int, data[ACK_PACKET_ID_INDEX]))
				return 0;

			if(ntemp->event.status == wait && ntemp->event.blockedThread != 0)
			{
				TaskHandle_t temp = ntemp->event.blockedThread;
				ntemp->event.len = len - ACK_PAYLOAD_INDEX;

				memcpy(ntemp->rxPayload, &data[ACK_PAYLOAD_INDEX], l);

				ntemp->event.status = free_;
				xTaskNotifyGive(temp);
			}
			return 0;
		}
	}
	else if (handlers[data[DATA_HANDLER_ID_INDEX]] != 0)
	{
		serialComHandler_t *ntemp = handlers[data[DATA_HANDLER_ID_INDEX]];
		if (ntemp->handler != 0)
		{
			if (cast(unsigned short int, data[DATA_PACKET_ID_INDEX]) != ntemp->rxPacketId)
			{
				ntemp->respLen = ntemp->handler(ntemp, &data[DATA_PAYLOAD_INDEX], len-DATA_PAYLOAD_INDEX);
				ntemp->rxPacketId = cast(unsigned short int, data[DATA_PACKET_ID_INDEX]);
			}

			if (ntemp->respLen == 0)
				return 0;

			data[ACK_CHAR_INDEX] = '\a';
			data[ACK_HANDLER_ID_INDEX] = ntemp->ID;
			cast(unsigned short int, data[ACK_PACKET_ID_INDEX]) = ntemp->rxPacketId;

			memcpy(&data[ACK_PAYLOAD_INDEX], ntemp->respPayload, ntemp->respLen);

			l = ntemp->respLen + 4;

			while (hcrc.State == HAL_CRC_STATE_BUSY) osThreadYield();
			crc = HAL_CRC_Calculate(&hcrc, &cast(uint32_t, data[ACK_CHAR_INDEX]), l);

			cast(unsigned short int, data[PACKET_LEN_INDEX]) = l;
			cast(unsigned short int, data[l+ACK_CHAR_INDEX]) = crc;

			return l + 4;
		}


	}

	return 0;
}

int serialComHandler_t::writeAckPayload(uint8_t *data, unsigned int len)
{
	memcpy(respPayload, data, len);
	respLen = len;
	return 1;
}


int serialComHandler_t::sendData(const uint8_t *data, unsigned int len, unsigned char retry, unsigned int timout, unsigned char loop)
{


	if (retry && timout == 0) timout = 100;


	unsigned int i = 0, j;
	i += 2; //place for len
	txPayload[i++] = ID;
	cast(unsigned short int, txPayload[i]) = txPacketId;
	i += 2;

	memcpy(&txPayload[i], data, len);
	i+=len;


	while (hcrc.State == HAL_CRC_STATE_BUSY) osThreadYield();
	cast(uint16_t, txPayload[i]) = HAL_CRC_Calculate(&hcrc, &cast(uint32_t, txPayload[2]), len+3);
	i += sizeof(uint16_t);
	cast(uint16_t, txPayload[0]) = len+3;

	//GLOG.print("len = %d, CRC = 0x%X\n",len, cast(uint16_t, txPayload[i-2]));


	txPayload[i++] = '\n';
	event.payload = (uint8_t*)txPayload;
	event.len = i;

	if (retry) event.status = wait;
	else event.status = busy;


	event.sig = (loop) ? LOOP_BACK_COMP : SEND_TO_COMP;

	comEvt_t *evptr = &event;

	event.blockedThread = osThreadGetId();
	osMessageQueuePut(serialComHandler_t::eventQueue, &evptr, 0, 0);
	if (!retry || xTaskNotifyWait(0, UINT32_MAX, 0, timout) == pdTRUE)
	{
		txPacketId++;
		return len;
	}

	if (retry)
	{

		event.blockedThread = osThreadGetId();
		while (--retry)
		{
			osMessageQueuePut(serialComHandler_t::eventQueue, &evptr, 0, 0);
			if (xTaskNotifyWait(0, UINT32_MAX, 0, timout) == pdTRUE)
			{
				txPacketId++;
				return len;
			}
		}
	}

	txPacketId++;
	return 0;
}



