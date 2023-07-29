/*
 * cncInsHandler.cpp
 *
 *  Created on: Aug 26, 2022
 *      Author: sanjay
 */

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "qpc.h"

#include "serialDebug.h"
#include "serialEvent.h"
#include "servo.h"
#include "X_servo.h"
#include "Y_servo.h"
#include "cncStateMachine.h"
#include "cncInsHandler.h"
#include "serialComHandlerV2.h"
#include "serialComHandler.h"

#include "PLC.h"
#include "cnc.h"
#include "printIns.h"

namespace isoFree
{
typedef struct QueueDefinition
{
	int8_t *pcHead;					/*< Points to the beginning of the queue storage area. */
	int8_t *pcTail;					/*< Points to the byte at the end of the queue storage area.  Once more byte is allocated than necessary to store the queue items, this is used as a marker. */
	int8_t *pcWriteTo;				/*< Points to the free next place in the storage area. */

	union							/* Use of a union is an exception to the coding standard to ensure two mutually exclusive structure members don't appear simultaneously (wasting RAM). */
	{
		int8_t *pcReadFrom;			/*< Points to the last place that a queued item was read from when the structure is used as a queue. */
		UBaseType_t uxRecursiveCallCount;/*< Maintains a count of the number of times a recursive mutex has been recursively 'taken' when the structure is used as a mutex. */
	} u;

	List_t xTasksWaitingToSend;		/*< List of tasks that are blocked waiting to post onto this queue.  Stored in priority order. */
	List_t xTasksWaitingToReceive;	/*< List of tasks that are blocked waiting to read from this queue.  Stored in priority order. */

	volatile UBaseType_t uxMessagesWaiting;/*< The number of items currently in the queue. */
	UBaseType_t uxLength;			/*< The length of the queue defined as the number of items it will hold, not the number of bytes. */
	UBaseType_t uxItemSize;			/*< The size of each items that the queue will hold. */

	volatile int8_t cRxLock;		/*< Stores the number of items received from the queue (removed from the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */
	volatile int8_t cTxLock;		/*< Stores the number of items transmitted to the queue (added to the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */

	#if( ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
		uint8_t ucStaticallyAllocated;	/*< Set to pdTRUE if the memory used by the queue was statically allocated to ensure no attempt is made to free the memory. */
	#endif

	#if ( configUSE_QUEUE_SETS == 1 )
		struct QueueDefinition *pxQueueSetContainer;
	#endif

	#if ( configUSE_TRACE_FACILITY == 1 )
		UBaseType_t uxQueueNumber;
		uint8_t ucQueueType;
	#endif

} xQUEUE;

typedef xQUEUE Queue_t;

BaseType_t xQueuePeekn( QueueHandle_t xQueue, void * const pvBuffer, int index)
{

	Queue_t * const pxQueue = ( Queue_t * ) xQueue;
	BaseType_t ret = pdFALSE;

	/* Check the pointer is not NULL. */
	configASSERT( ( pxQueue ) );

	/* The buffer into which data is received can only be NULL if the data size
	is zero (so no data is copied into the buffer. */
	configASSERT( !( ( ( pvBuffer ) == NULL ) && ( ( pxQueue )->uxItemSize != ( UBaseType_t ) 0U ) ) );

	if (index > pxQueue->uxLength)
		return pdFALSE;


	/* This function relaxes the coding standard somewhat to allow return
	statements within the function itself.  This is done in the interest
	of execution time efficiency. */


	taskENTER_CRITICAL();
	{
		const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;

		if (index < uxMessagesWaiting)
		{
			int8_t * readFrom = pxQueue->u.pcReadFrom;
			int8_t * tail = pxQueue->pcTail;

			readFrom += (index + 1) * pxQueue->uxItemSize;
			if (readFrom >= pxQueue->pcTail)
			{
				readFrom -= pxQueue->uxLength * pxQueue->uxItemSize;
			}

			( void ) memcpy( ( void * ) pvBuffer, ( void * ) readFrom, ( size_t ) pxQueue->uxItemSize );
			ret = pdTRUE;
		}
	}
	taskEXIT_CRITICAL();


	return ret;
}

}




uint8_t* cncInsComhandler(ioHandler::reqComHandler_t* com, const uint8_t *data, uint32_t len);

ioHandler::comHandler_t insFeedCom;
ioHandler::reqComHandler_t insFeedReqCom
{
	'>',
	cncInsComhandler,
};


uint32_t resPuledInsId = 0xFFFFFFFF, resExecutedInsId = 0xFFFFFFFF, loopInsId = 0, lastInsId = 0;

const word_t resPuledInsIdPD = D0;
const word_t resExecutedInsIdPD = D1;

// instruction queue -----------------------------------------------------------------
cncInsUn_t insQueueBuf[32];
StaticQueue_t insQueueSto;
QueueHandle_t insQueue;


StaticTask_t insTask;
StackType_t insTaskStack[128*4];
QueueHandle_t insTaskEventQueue;

void *insTaskEvtBuff[32];
StaticQueue_t insTaskEvtQueueSto;

uint8_t insFeedTxPayload[64], insFeedRespPaylad[1024];
uint8_t tempTxBuff[64];
uint8_t cncInsAckBuf[256];


void cncInsHandlerInit()
{
	insQueue = xQueueCreateStatic(32, sizeof(cncInsUn_t), (uint8_t*) &insQueueBuf[0], &insQueueSto);

	insTaskEventQueue = xQueueCreateStatic(32, sizeof(void *), (uint8_t*) &insTaskEvtBuff[0], &insTaskEvtQueueSto);
	xTaskCreateStatic(cncInsWorkerTaskV2, "insTask", 128*4, 0, osPriorityNormal, insTaskStack, &insTask);

	ioHandler::addReqHandler(&insFeedReqCom);
}



void cncInsWorkerTaskV2(void *arg)
{
	insTaskEvt_t *ev;

	insFeedCom.init('F', insFeedTxPayload, insFeedRespPaylad);

	while(1)
	{

		if (osMessageQueueGet(insTaskEventQueue, &ev, 0, 1000) != osOK)
			continue;



		switch(ev->sig)
		{

		case INS_EXECUTED:
		{
		} break;

		case PULL_INS_REQ:
		{
			uint32_t nextInsId;
			if (resPuledInsId == lastInsId)
				nextInsId = loopInsId;
			else
				nextInsId = lastInsId + 1;

			int i = 0;
			memcpy(&tempTxBuff[i], "PUL", 3);
			i+=3;

			memcpy(&tempTxBuff[i], &nextInsId, sizeof(nextInsId));
			i+= sizeof(nextInsId);

			uint32_t noOfIns = osMessageQueueGetSpace(insQueue);
			if (noOfIns > 5) noOfIns = 5;

			memcpy(&tempTxBuff[i], &noOfIns, sizeof(noOfIns));
			i+= noOfIns;

			if (insFeedCom.sendReq(tempTxBuff, i, 200, 3))
			{
				i = 0;
				if (tempTxBuff[0] == 'T')
				{
					print("got txt ins\n");

					xTaskNotify(ev->task, 1, eSetBits);
					break;
				}

			}

			xTaskNotify(ev->task, 0, eNoAction);


			break;
		}


		case START_FEED_REQ:
		{

			if (insFeedCom.sendReq((uint8_t*)"GETFEED", sizeof("GETFEED"), 1000, 3))
			{
				const uint8_t *resp = insFeedCom.respPayload;
				int i = 0;
				uint8_t startWithZero;
				if (memcmp(resp, "FRDY", 4) == 0)
				{
					i = 4;
					resPuledInsId = 0;

					memcpy(&resExecutedInsId, &resp[i], sizeof(resExecutedInsId));
					i+= sizeof(resExecutedInsId);
					resPuledInsId = resExecutedInsId;

					memcpy(&loopInsId, &resp[i], sizeof(loopInsId));
					i+= sizeof(loopInsId);

					memcpy(&lastInsId, &resp[i], sizeof(lastInsId));
					i+= sizeof(lastInsId);

					startWithZero = resp[i];
					i++;

					xTaskNotify( ev->task, 1, eSetBits);
				}
				else if (memcmp(resp, "FNRD", 4) == 0)
				{
					xTaskNotify( ev->task, 0, eNoAction);
				}
			}
			break;
		}

		case STOP_FEED_REQ:
		{

			break;
		}
		default:
		{
			break;
		}
		}

	}
}


void queueInsHandler(ioHandler::reqComHandler_t* com, const uint8_t *data, uint32_t len)
{
	uint32_t i = 0, privInsId;
	cncInsUn_t ins;

	privInsId = unPackData<uint32_t>(data, i);
	ins = unPackData<cncInsUn_t>(data, i);

	if (privInsId != resPuledInsId && privInsId != (uint32_t)-1)
		return;

	switch(ins.type)
	{
	case START_STITCH:
	case STITCH_INS:
	case START_JUMP:
	case JUMP_INS:
	case ZERO_INS:
	case SET_VALVE_INS:
	case STOP_INS:
		if (osMessageQueuePut(insQueue, &ins, 0, 0) == osOK)
		{
			resPuledInsId = ins.cncIns.id;
			PLC::write(resPuledInsIdPD, resPuledInsId);
		}
		break;

	default:
	 return;
	}

}


uint8_t* cncInsComhandler(ioHandler::reqComHandler_t* com, const uint8_t *data, uint32_t len)
{
	uint32_t l = 0, temp;

	if (memcmp(&data[0], "STA", 3) == 0) {
		packData<char>(com->ID, cncInsAckBuf, l);
		packData<uint32_t>(resExecutedInsId, cncInsAckBuf, l);
		packData<uint32_t>(resPuledInsId, cncInsAckBuf, l);

		temp = osMessageQueueGetSpace(insQueue);
		packData<uint32_t>(temp, cncInsAckBuf, l);

		ioHandler::sendAckResp(com->packetSno, cncInsAckBuf, l);
	}
	else if (memcmp(&data[0], "QIN", 3) == 0) {
		queueInsHandler(com, &data[3], len-3);
	}
	else if (memcmp(&data[0], "RST", 3) == 0) {
		resExecutedInsId = resPuledInsId = (uint32_t) -1;
		osMessageQueueReset(insQueue);
		ioHandler::sendAckResp(com->packetSno, (uint8_t*)">RST", sizeof(">RST"));
	}
	else if (memcmp(&data[0], "TST", 3) == 0) {
		ioHandler::sendAckResp(com->packetSno, (uint8_t*)">test", sizeof(">test"));
		print("ins test com\n");
	}
	return NULL;
}






int unpackTxtIns(const char *insBuf)
{
	uint32_t i, j, unIns;
	cncInsUn_t ins;
	unsigned int valve = 0;

	for (i = 0, j = 0; insBuf[i] != '\0' && j < 32; j++, i++)
	{
		unIns = 1;

		if (insBuf[i] == 'S')
		{
			unIns = 0;
			ins.type = STITCH_INS;
			sscanf(&insBuf[i], "S %X X:%d Y:%d Z:%c T:%d\n", &ins.stitchIns.id, &ins.stitchIns.xStep, &ins.stitchIns.yStep, &ins.stitchIns.vfdSpeed, &ins.stitchIns.stepTime);
			if(ins.cncIns.id >= 120)
			{printF("type = STITCH_INS, id = %d,vs = %c, x = %d, y = %d, time = %d\n",ins.stitchIns.id, ins.stitchIns.vfdSpeed, ins.stitchIns.xStep, ins.stitchIns.yStep, ins.stitchIns.stepTime);
			}
		}
		else if (insBuf[i] == 'Z')
		{
			unIns = 0;
			ins.type = ZERO_INS;
			sscanf(&insBuf[i], "Z %X X:%d Y:%d Z:%c T:%d\n", &ins.stitchIns.id, &ins.stitchIns.xStep, &ins.stitchIns.yStep, &ins.stitchIns.vfdSpeed, &ins.stitchIns.stepTime);
//			printF("type = ZERO_INS, id = %d,vs = %c, x = %d, y = %d, time = %d\n", ins.zeroIns.id, ins.zeroIns.vfdSpeed, ins.zeroIns.xStep, ins.zeroIns.yStep, ins.zeroIns.stepTime);

		}
		else if (insBuf[i] == 'J')
		{
			unIns = 0;
			ins.type = JUMP_INS;
			sscanf(&insBuf[i], "J %X X:%d Y:%d V:%X T:%d\n", &ins.jumpIns.id, &ins.jumpIns.xStep, &ins.jumpIns.yStep, &valve, &ins.jumpIns.stepTime);
			ins.jumpIns.valveMask = valve;
//			printF("type = JUMP_INS, id = %d, vmask = %x, x = %d, y = %d, time = %d\n", ins.jumpIns.id, ins.jumpIns.valveMask, ins.jumpIns.xStep, ins.jumpIns.yStep, ins.jumpIns.stepTime);

		}
		else if (insBuf[i] == 'V')
		{
			unIns = 0;
			ins.type = SET_VALVE_INS;
			sscanf(&insBuf[i], "V %X V:%X A:%c D:%u\n", &ins.valveIns.id, &valve, &ins.valveIns.action, &ins.valveIns.delay);
			ins.valveIns.valve = valve;
//			printF("type = SET_VALVE_INS, id = %d, vmask = %x, act = %x, delay = %d\n", ins.valveIns.id, ins.valveIns.valve, ins.valveIns.action, ins.valveIns.delay);

		}

		if (unIns) return j;
		if (ins.cncIns.type != NONE_INS)
		{
			if (osMessageQueuePut(insQueue, &ins, 0, portMAX_DELAY) != osOK)
			{
				return j;
			}
			resPuledInsId = ins.cncIns.id;
		}

		while (insBuf[++i] != '\n');



	}


	return j;

}





int unpackRawIns(const uint8_t *rawInsBuffer)
{
	uint32_t i, j, k;
	uint8_t insType, queueIns = 0;
	cncInsUn_t ins;

	for (i = 0, j = 0; rawInsBuffer[i] != 0 && j < 32; i += rawInsBuffer[i]+1, j++)
	{
		k = i+1;
		queueIns = 0;
		insType = rawInsBuffer[k++];

		if (insType == ZERO_INS)
		{
			queueIns = 1;
			ins.type = ZERO_INS;
			ins.zeroIns.id = cast(uint32_t, rawInsBuffer[k]), k+=4;
			ins.zeroIns.vfdSpeed = cast(uint8_t, rawInsBuffer[k]), k++;
			ins.zeroIns.xStep = cast(int32_t, rawInsBuffer[k]), k+=4;
			ins.zeroIns.yStep = cast(int32_t, rawInsBuffer[k]), k+=4;
			ins.zeroIns.stepTime = cast(uint32_t, rawInsBuffer[k]), k+=4;
//			printF("type = ZERO_INS, id = %d,vs = %c, x = %d, y = %d, time = %d\n", ins.zeroIns.id, ins.zeroIns.vfdSpeed, ins.zeroIns.xStep, ins.zeroIns.yStep, ins.zeroIns.stepTime);

		}
		else if (insType == STITCH_INS)
		{
			queueIns = 1;
			ins.type = STITCH_INS;
			ins.stitchIns.id = cast(uint32_t, rawInsBuffer[k]), k+=4;
			ins.stitchIns.vfdSpeed = cast(uint8_t, rawInsBuffer[k]), k++;
			ins.stitchIns.xStep = cast(int32_t, rawInsBuffer[k]), k+=4;
			ins.stitchIns.yStep = cast(int32_t, rawInsBuffer[k]), k+=4;
			ins.stitchIns.stepTime = cast(uint32_t, rawInsBuffer[k]), k+=4;
//			printF("type = STITCH_INS, id = %d,vs = %c, x = %d, y = %d, time = %d\n",ins.stitchIns.id, ins.stitchIns.vfdSpeed, ins.stitchIns.xStep, ins.stitchIns.yStep, ins.stitchIns.stepTime);
		}
		else if (insType == JUMP_INS)
		{
			queueIns = 1;
			ins.type = JUMP_INS;
			ins.jumpIns.id = cast(uint32_t, rawInsBuffer[k]), k+=4;
			ins.jumpIns.valveMask = cast(uint8_t, rawInsBuffer[k]), k++;
			ins.jumpIns.xStep = cast(int32_t, rawInsBuffer[k]), k+=4;
			ins.jumpIns.yStep = cast(int32_t, rawInsBuffer[k]), k+=4;
			ins.jumpIns.stepTime = cast(uint32_t, rawInsBuffer[k]), k+=4;
//			printF("type = JUMP_INS, id = %d, vmask = %x, x = %d, y = %d, time = %d\n", ins.jumpIns.id, ins.jumpIns.valveMask, ins.jumpIns.xStep, ins.jumpIns.yStep, ins.jumpIns.stepTime);
		}
		else if (insType == SET_VALVE_INS)
		{
			queueIns = 1;
			ins.type = SET_VALVE_INS;
			ins.valveIns.id = cast(uint32_t, rawInsBuffer[k]), k+=4;
			ins.valveIns.valve = cast(uint8_t, rawInsBuffer[k]), k++;
			ins.valveIns.action = cast(uint8_t, rawInsBuffer[k]), k++;
			ins.valveIns.delay = cast(int32_t, rawInsBuffer[k]), k+=4;
//			printF("type = SET_VALVE_INS, id = %d, vmask = %x, act = %x, delay = %d\n", ins.valveIns.id, ins.valveIns.valve, ins.valveIns.action, ins.valveIns.delay);
		}

		if (queueIns)
		{
			if (osMessageQueuePut(insQueue, &ins, 0, 0) == osOK)
			{
				resPuledInsId = ins.cncIns.id;
			}
			else
				break;
		}
	}
	return j;
}




int getQueueSpace()
{
	return osMessageQueueGetSpace(insQueue);
}


startFeeder_t startFeedReq;
static uint32_t refHartBeat;

void insHandlerHartBeatCheck()
{
	if (PLC::read(FDR_HART_BEAT, CHANGE_READ))
	{
		PLC::write(FDR_ALIVE, R_CLOSE);
		refHartBeat = HAL_GetTick();
	}

	if (PLC::read(FDR_ALIVE))
	if (HAL_GetTick() - refHartBeat > 2000)
	{
		PLC::write(FDR_ALIVE, R_OPEN);
	}
}

int isFeederAlive()
{
	if (PLC::read(FDR_ALIVE)) return 1;
	else return 0;
}

int isFeederRunning()
{
	if (PLC::read(FDR_RING)) return 1;
	else return 0;
}

void runFeeder()
{
	PLC::write(RUN_FDR, R_CLOSE);
}

void stopFeederv2()
{
	PLC::write(RUN_FDR, R_OPEN);
}

int startFeeder()
{
	startFeedReq.ev.sig = START_FEED_REQ;
	startFeedReq.sender = "STFEED";
	startFeedReq.ev.task = osThreadGetId();
	void *ev = &startFeedReq;
	xQueueReset(insQueue);
	if (osMessageQueuePut(insTaskEventQueue, &ev, 0, 1000) == osOK)
	{
		uint32_t ret = 0;
		xTaskNotifyWait(-1, -1, &ret, 5000);
		if (ret)
		{
			return ret;
		}

	}

	return 0;
}

startFeeder_t stopev;

int stopFeeder()
{

	stopev.ev.sig = STOP_FEED_REQ;
	stopev.sender = "STOP_FEED";
	void *ev = &stopev;

	if (osMessageQueuePut(insTaskEventQueue, &ev, 0, 1000) == osOK)
	{
		print("stoping feed... %d\n");
		return 1;
	}
	print("ERR feed not stop\n");
	return 0;
}

int pullIns(uint32_t noOfins)
{
	startFeedReq.ev.sig = PULL_INS_REQ;
	startFeedReq.sender = "PULL_INS";
	startFeedReq.ev.noOfIns = noOfins;
	startFeedReq.ev.task = osThreadGetId();


	void *ev = &startFeedReq;
	osMessageQueuePut(insTaskEventQueue, &ev, 0, 0);

	uint32_t ret = 0;
	xTaskNotifyWait(-1, -1, &ret, 5000);

	return ret;


}


int peakNextIns(void *ins)
{
	return xQueuePeek(insQueue, ins, 0);
}

int getNextIns(void *ins)
{
	uint8_t ret = xQueueReceive(insQueue, ins, 0);
	if (ret) {
		resExecutedInsId = ((cncIns_t*)ins)->id;
		PLC::write(resExecutedInsIdPD, resExecutedInsId);
	}

	return ret;
}




