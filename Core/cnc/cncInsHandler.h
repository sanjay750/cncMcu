/*
 * cncInsHandler.h
 *
 *  Created on: Aug 26, 2022
 *      Author: sanjay
 */

#ifndef CNC_CNCINSHANDLER_H_
#define CNC_CNCINSHANDLER_H_





#include "cnc.h"
#include "cmsis_os.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif



#define bit_(n) (1<<n)


//feed status bits
#define FEED_STARTING_BIT	bit_(1)
#define FEED_RUNNING_BIT	bit_(0)

//mcu status bits
#define RUN_FEEDER_BIT 	bit_(1)
#define QUILT_SIG_BIT	bit_(0)




typedef enum
{
	PULL_INS_REQ = 1,
	START_FEEDER_REQ = 2,
	INS_EXECUTED = 3,
	START_FEED_REQ,
	STOP_FEED_REQ,
}insTaskEvtType_t;

typedef enum
{
    FEED_NOT_READY,
    FEED_READY,
    TXT_INS,
    RAW_INS,
}insFeedResp_t;

typedef struct
{
	insTaskEvtType_t sig;
	TaskHandle_t task;
	uint32_t noOfIns;
}insTaskEvt_t;

typedef struct
{
	insTaskEvt_t ev;
	const char *sender;
}startFeeder_t;



void cncInsHandlerInit();
void cncInsWorkerTask(void *args);
void cncInsWorkerTaskV2(void *args);


int unpackRawIns(const uint8_t*);
int unpackTxtIns(const char *insBuf);

void insHandlerHartBeatCheck();
void runFeeder();
void stopFeederv2();

int startFeeder();
int stopFeeder();



int pullIns(uint32_t noOfins);
int putRawIns(unsigned char *rawIns);
int peakNextIns(void *ins);
int getNextIns(void *ins);

int getQueueSpace();

void dbgStep();

void setQueueFlags();


void getInsStr(cncInsUn_t *ins, char *buf);


#ifdef __cplusplus
}
#endif

#endif /* CNC_CNCINSHANDLER_H_ */
