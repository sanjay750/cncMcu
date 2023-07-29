/*
 * serialComHandler.h
 *
 *  Created on: Aug 26, 2022
 *      Author: sanjay
 */

#ifndef SERIALCOMHANDLER_SERIALCOMHANDLER_H_
#define SERIALCOMHANDLER_SERIALCOMHANDLER_H_

#include "cmsis_os.h"
#include "queue.h"
#include "task.h"


enum serialPrintEventStatus
{
	busy,
	free_,
	wait,
	com_error,
};

enum serialPacketEvents
{

	SEND_TO_COMP,
	OUTPUT_FROM_COMP,
	LOOP_BACK_COMP,
};


struct comEvt_t
{
	uint8_t sig;
	uint8_t status;
	uint8_t *payload;
	uint32_t len;
	osThreadId_t blockedThread;
};

#define SERIAL_COM_GPIO		GPIOC
#define SERIAL_COM_PIN		GPIO_PIN_9


enum bufferIndex
{

	PACKET_LEN_INDEX = 0,

	DATA_HANDLER_ID_INDEX= PACKET_LEN_INDEX + 2,
	DATA_PACKET_ID_INDEX = DATA_HANDLER_ID_INDEX+1,
	DATA_PAYLOAD_INDEX = DATA_PACKET_ID_INDEX + 2,

	ACK_CHAR_INDEX = PACKET_LEN_INDEX + 2,
	ACK_HANDLER_ID_INDEX = ACK_CHAR_INDEX + 1,
	ACK_PACKET_ID_INDEX = ACK_HANDLER_ID_INDEX + 1,
	ACK_PAYLOAD_INDEX = ACK_PACKET_ID_INDEX + 2,

};




typedef unsigned char* comStr;

#define getComStr(name, size) \
unsigned char name ## _head [size] = {'<', 0, 0, 0};\
comStr name = (name ## _head + 4)


void serialComHandlerInit();

void serialComHandlerTask(void *args);
void serialActLedHandler();


struct serialComHandler_t
{
	static serialComHandler_t* handlers[256];

	static QueueHandle_t eventQueue;
	static int comHandler(unsigned char *data, unsigned int len);

	static int comInit(serialComHandler_t *handler, unsigned char *rxPayload, unsigned char *txPayload, unsigned char *ackPayload);
	int writeAckPayload(uint8_t* data, unsigned int len);
	int sendData(const uint8_t *data, unsigned int len, unsigned char retry = 0, unsigned int timout = 100, unsigned char loop = 0);


	unsigned char ID;
	int (*handler)(serialComHandler_t* com, uint8_t *data, unsigned int len);
	comEvt_t event;
	uint16_t rxPacketId;
	uint16_t txPacketId;
	unsigned char respLen;
	unsigned char *txPayload;
	unsigned char *rxPayload;
	unsigned char *respPayload;

};

extern serialComHandler_t comWatchDog;
#endif /* SERIALCOMHANDLER_SERIALCOMHANDLER_H_ */
