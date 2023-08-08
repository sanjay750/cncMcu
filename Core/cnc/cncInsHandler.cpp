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


const char passward[256] = "isha-textile@112";
const char devideID[256] = "ISHA1234";
const char passcheck[256]  __attribute__((section(".seeprom"))) = "";


#define UNLOCK_FLAG	(*(__IO uint32_t *)passcheck)
#define IS_UNLOCK()	((UNLOCK_FLAG == 0xABCD) ? (true) :(false))


uint32_t unlockDevice(const char *pass) {

	if (IS_UNLOCK())
		return 2;

	if (strcmp(passward, pass) == 0) {

		FLASH_EraseInitTypeDef ers;
		HAL_FLASH_Unlock();

		ers.TypeErase = FLASH_TYPEERASE_PAGES;
		ers.PageAddress = (uint32_t)&UNLOCK_FLAG;
		ers.NbPages = 1;

		uint32_t ferr;

		if (HAL_FLASHEx_Erase(&ers, &ferr)) {
			HAL_FLASH_Lock();
			return 0;
		}

		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&UNLOCK_FLAG, 0xABCD)) {
			HAL_FLASH_Lock();
			return 0;
		}

		HAL_FLASH_Lock();



		return 0xABCD;
	}

	return 0xEEEE;
}



uint8_t* cncInsComhandler(ioHandler::reqComHandler_t* com, const uint8_t *data, uint32_t len);

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



void *insTaskEvtBuff[32];
StaticQueue_t insTaskEvtQueueSto;

uint8_t insFeedTxPayload[64], insFeedRespPaylad[1024];
uint8_t tempTxBuff[64];
uint8_t cncInsAckBuf[256];


void cncInsHandlerInit() {
	insQueue = xQueueCreateStatic(32, sizeof(cncInsUn_t), (uint8_t*) &insQueueBuf[0], &insQueueSto);
	ioHandler::addReqHandler(&insFeedReqCom);
}


void queueInsHandler(ioHandler::reqComHandler_t* com, const uint8_t *data, uint32_t len) {

	uint32_t i = 0, privInsId;
	cncInsUn_t ins;

	privInsId = unPackData<uint32_t>(data, i);
	ins = unPackData<cncInsUn_t>(data, i);

	if (privInsId != resPuledInsId && privInsId != (uint32_t)-1)
		return;

	switch(ins.type) {
	case START_STITCH:
	case STITCH_INS:
	case START_JUMP:
	case JUMP_INS:
	case ZERO_INS:
	case SET_VALVE_INS:
	case STOP_INS:
		if (osMessageQueuePut(insQueue, &ins, 0, 0) == osOK) {
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
		unlockDevice("STA");
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

		if (queueIns) {
			if (osMessageQueuePut(insQueue, &ins, 0, 0) == osOK) {
				resPuledInsId = ins.cncIns.id;
			}
			else
				break;
		}
	}
	return j;
}




int getQueueSpace() {
	return osMessageQueueGetSpace(insQueue);
}


startFeeder_t startFeedReq;
static uint32_t refHartBeat;

void insHandlerHartBeatCheck() {
	if (PLC::read(FDR_HART_BEAT, CHANGE_READ)) {
		PLC::write(FDR_ALIVE, R_CLOSE);
		refHartBeat = HAL_GetTick();
	}

	if (PLC::read(FDR_ALIVE) && HAL_GetTick() - refHartBeat > 2000) {
		PLC::write(FDR_ALIVE, R_OPEN);
	}
}

int isFeederAlive() {
	if (PLC::read(FDR_ALIVE)) return 1;
	else return 0;
}

int isFeederRunning() {
	if (PLC::read(FDR_RING)) return 1;
	else return 0;
}

void runFeeder() {
	PLC::write(RUN_FDR, R_CLOSE);
}

void stopFeederv2() {
	PLC::write(RUN_FDR, R_OPEN);
}

int peakNextIns(void *ins) {
	return xQueuePeek(insQueue, ins, 0);
}

int getNextIns(void *ins) {
	uint8_t ret = xQueueReceive(insQueue, ins, 0);
	if (ret) {
		resExecutedInsId = ((cncIns_t*)ins)->id;
		PLC::write(resExecutedInsIdPD, resExecutedInsId);
	}

	return ret;
}




