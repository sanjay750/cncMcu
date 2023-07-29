/*
 * serialComHandlerV2.cpp
 *
 *  Created on: 09-May-2023
 *      Author: sanjay
 */




#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stdio.h"
#include "string.h"

#include "serialComHandlerV2.h"




#define PRI_BUF_SIZ		(1024*2)
#define SEC_BUF_SIZ		(1024)
#define RXIND			(priRxIndex & (PRI_BUF_SIZ - 1))
#define WRIT_BUF(ch) if (rxBufIndex < SEC_BUF_SIZ) rxBuffer[rxBufIndex++] = ch

namespace ioHandler
{



uint8_t priRxBuffer[PRI_BUF_SIZ];
uint8_t rxBuffer[1024*2];
uint16_t rxCnt, priRxIndex, rxBufIndex;

uint8_t priTxBuffer[512], txBuffer[1024];
uint8_t respTxBuffer[512];

DMA_HandleTypeDef txDma, rxDma;

rxState_t rxState;

reqComHandler_t *reqHandlerList[16];
uint8_t reqListLen;

comHandler_t *comHandlerList[16];
uint8_t comHandlerListLen;


ackStatus_t ackstatus = IDLE;
osThreadId_t blockedAckThread = NULL;



osMessageQueueId_t txPacketQueue;

SemaphoreHandle_t txSemaphore = NULL;
StaticSemaphore_t txMutexBuffer;

void serialComInit()
{

	txSemaphore = xSemaphoreCreateBinaryStatic( &txMutexBuffer );

	xSemaphoreGive(txSemaphore);
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

	txPacketQueue = osMessageQueueNew(8, sizeof(sendPacketReq_t), 0);


	rxDma.DmaBaseAddress = DMA2;
	rxDma.ChannelIndex = 3;
	rxDma.Instance = DMA2_Channel3;

	rxDma.Init.Mode = DMA_CIRCULAR;
	rxDma.Init.PeriphInc = DMA_PINC_DISABLE;
	rxDma.Init.MemInc = DMA_MINC_ENABLE;
	rxDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	rxDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	rxDma.Init.Priority = DMA_PRIORITY_MEDIUM;


	HAL_DMA_Init(&rxDma);
	HAL_DMA_Start(&rxDma, (uint32_t) &UART4->RDR, (uint32_t) priRxBuffer, PRI_BUF_SIZ);
	rxCnt = PRI_BUF_SIZ;

	UART4->CR3 |= USART_CR3_DMAR;


}


txState_t txState = TX_WAIT_PACKET;

void addReqHandler(reqComHandler_t *reqHandler)
{
    if (reqListLen < 16)
    {
        reqHandlerList[reqListLen++] = reqHandler;
    }
}

extern "C" void txHandler()
{

	sendPacketReq_t pack;

	switch (txState)
	{
	case TX_WAIT_PACKET:
	{
		if (osMessageQueueGet(txPacketQueue, &pack, 0, 0) == osOK)
		{

			if (pack.len > 500) break;


			//create packet
			int i = 0, temp;
			//space for packet len
			i += 2;
	        memcpy(&priTxBuffer[i], &pack.psno, sizeof(pack.psno));
	        i += sizeof(pack.psno);

			memcpy(&priTxBuffer[i], pack.packet, pack.len);
			i += pack.len;

			temp = i - 2;

			memcpy(&priTxBuffer[0], &temp, 2);

			while (hcrc.State == HAL_CRC_STATE_BUSY) osThreadYield();
			uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)priTxBuffer, temp);

			memcpy(&priTxBuffer[i], &crc, 2);
			i += 2;


			// encode packet
			uint16_t j, k;
			for (j = 0, k = 0; k < i; j++, k++)
			{
				txBuffer[j] = priTxBuffer[k];
				if (priTxBuffer[k] == '\r')
					txBuffer[++j] = '\r';
			}
			txBuffer[j++] = '\r';
			txBuffer[j++] = '\n';

			//send packet

			txDma.Instance->CCR &= ~DMA_CCR_EN;
			txDma.Instance->CMAR = (uint32_t) txBuffer;
			txDma.Instance->CPAR = (uint32_t) &UART4->TDR;
			txDma.Instance->CNDTR = j;

			UART4->CR3 |= USART_CR3_DMAT;
			txDma.DmaBaseAddress->IFCR = DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CGIF5;
			txDma.Instance->CCR |= DMA_CCR_EN | DMA_CCR_TCIE;

			txState = TX_WAIT_DMA_TCOMP;

		}
		break;
	}

	case TX_WAIT_DMA_TCOMP:
	{
		if (txDma.DmaBaseAddress->ISR & DMA_ISR_TCIF5)
		{
			txState = TX_WAIT_PACKET;
		}
		break;
	}
	}
}

void sendAckResp(uint16_t psno, const uint8_t* ack, int len)
{
	if (len > 512) return;


	uint16_t i = 0;
	respTxBuffer[i++] = '\a';

	memcpy(&respTxBuffer[i], ack, len);
	i += len;

	sendPacket(psno, respTxBuffer, i);
}

void sendPacket(uint16_t psno, uint8_t *packet, int len)
{
	sendPacketReq_t pack = {psno, packet, len};
//	osMessageQueuePut(txPacketQueue, &pack, 0, portMAX_DELAY);

	if (pack.len > 500) return;

	xSemaphoreTake(txSemaphore, portMAX_DELAY);




	//create packet
	int i = 0, temp;
	//space for packet len
	i += 2;
    memcpy(&priTxBuffer[i], &pack.psno, sizeof(pack.psno));
    i += sizeof(pack.psno);

	memcpy(&priTxBuffer[i], pack.packet, pack.len);
	i += pack.len;

	temp = i - 2;

	memcpy(&priTxBuffer[0], &temp, 2);

	while (hcrc.State == HAL_CRC_STATE_BUSY) osThreadYield();
	uint16_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)priTxBuffer, temp);

	memcpy(&priTxBuffer[i], &crc, 2);
	i += 2;


	// encode packet
	uint16_t j, k;
	for (j = 0, k = 0; k < i; j++, k++)
	{
		txBuffer[j] = priTxBuffer[k];
		if (priTxBuffer[k] == '\r')
			txBuffer[++j] = '\r';
	}
	txBuffer[j++] = '\r';
	txBuffer[j++] = '\n';

	//send packet

	txDma.Instance->CCR &= ~DMA_CCR_EN;
	txDma.Instance->CMAR = (uint32_t) txBuffer;
	txDma.Instance->CPAR = (uint32_t) &UART4->TDR;
	txDma.Instance->CNDTR = j;

	UART4->CR3 |= USART_CR3_DMAT;
	txDma.DmaBaseAddress->IFCR = DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CGIF5;
	txDma.Instance->CCR |= DMA_CCR_EN | DMA_CCR_TCIE;

	while (!(txDma.DmaBaseAddress->ISR & DMA_ISR_TCIF5))
	{
		osDelay(2);
	}


	xSemaphoreGive(txSemaphore);

}





void packetHandler(const uint8_t *data, int len)
{
	int flen = len;
	int l = cast(uint16_t,data[PACKET_LEN_INDEX]);

	if (l > len) return;

	while (hcrc.State == HAL_CRC_STATE_BUSY) osThreadYield();
	uint16_t crc = HAL_CRC_Calculate(&hcrc, &cast(uint32_t, data[0]), l);



	if (memcmp(&crc, &data[l+2], 2) != 0)
		return;



	uint16_t psno;
	memcpy(&psno, &data[PACKET_SNO_INDEX], sizeof(psno));
	uint8_t packetId = data[PACKET_ID_INDEX];
	reqComHandler_t *reqHandler = 0;
	comHandler_t *comHandler = 0;


	if (packetId == '\a')
	{
		uint8_t i;
		packetId = data[ACK_PACKET_ID];
		for (i = 0; i < comHandlerListLen; i++)
			if (packetId == comHandlerList[i]->ID)
			{
				comHandler = comHandlerList[i];
				break;
			}

		if (comHandler != 0)
		{
			if (comHandler->status == wait_for_ack && comHandler->blockedThread != 0 && psno == comHandler->packetSno)
			{
				memcpy(comHandler->respPayload, &data[ACK_PACKET_ID+1], l - 4);
				comHandler->respLen = l-4;
				comHandler->status = got_ack;
				xTaskNotifyGive(comHandler->blockedThread);
			}
			else
			{
				printF("sno err %d, %d\n", (int)psno, (int)comHandler->packetSno);
			}
		}

	}
	else
	{
		uint8_t i;
		for (i = 0; i < reqListLen; i++)
			if (packetId == reqHandlerList[i]->ID)
			{
				reqHandler = reqHandlerList[i];
				break;
			}

		if (reqHandler != 0 && reqHandler->handler != 0)
		{
			reqHandler->packetSno = psno;
			reqHandler->handler(reqHandler, &data[PACKET_ID_INDEX+1], l-3);
		}
	}





}

void comHandler_t::init(char ID, uint8_t *txPl, uint8_t *respPl)
{
	this->ID = ID;
	txPayload = txPl;
	respPayload = respPl;
	packetSno = 0;

	if (comHandlerListLen < 16)
		comHandlerList[comHandlerListLen++] = this;
}

int comHandler_t::sendReq(const uint8_t*req, int len, uint32_t timeout, int retry)
{
	if (respPayload == 0) return 0;
	if (len == 0) return 0;

	int i = 0;
	txPayload[i++] = ID;
	memcpy(&txPayload[i], req, len);
	i += len;

	packetSno++;

	do
	{
		blockedThread = osThreadGetId();
		status = wait_for_ack;
		sendPacket(packetSno, txPayload, i);
		if (timeout == 0) return 1;

		BaseType_t temp = xTaskNotifyWait(0, UINT32_MAX, 0, timeout);
		if (temp == pdTRUE && status == got_ack)
		{
			return respLen;
		}
		if (retry == 0)
			break;
		retry--;

	}while (1);


	return 0;

}


int readSeral()
{
	int ch = -1;
	if (DMA2_Channel3->CNDTR != rxCnt)
	{
		ch = priRxBuffer[RXIND];
		priRxIndex++;

		rxCnt--;
		if (rxCnt == 0)
			rxCnt = PRI_BUF_SIZ;
	}
	return ch;
}



void rxHandler()
{

	int ch = readSeral();
	if (ch != -1)
	{
        switch (rxState)
        {
        case RX_STATE_CR:
        {
            if (ch == '\r')
            {
                WRIT_BUF(ch);
                rxState = RX_STATE_CHR;
            }
            else if (ch == '\n')
            {
                WRIT_BUF('\0');
                if (rxBufIndex < 1024 - 4)
                {
                	packetHandler(rxBuffer, rxBufIndex);
                }
                rxBufIndex = 0;
                rxState = RX_STATE_CHR;
            }
        }
        break;

        case RX_STATE_CHR:
        {
            if(ch == '\r')
            {
                rxState = RX_STATE_CR;
            }
            else
            {
                WRIT_BUF(ch);
                rxState = RX_STATE_CHR;
            }
        }
        break;

        default:
            break;
        }
	}

}

void serialComHandler()
{
	rxHandler();
	txHandler();
}


}
