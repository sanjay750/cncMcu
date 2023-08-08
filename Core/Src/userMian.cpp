/*
 * userMian.cpp
 *
 *  Created on: Aug 22, 2022
 *      Author: sanjay
 */



#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "cmsis_os.h"
#include "task.h"
#include "serialDebug.h"

#include "stm32f3xx_hal_flash.h"

#include "serialEvent.h"
#include "servo.h"
#include "X_servo.h"
#include "Y_servo.h"
#include "servoHelpertTim.h"
#include "timer2ServoHelperT.h"



#include "cncStateMachine.h"
#include "cncInsHandler.h"
#include "serialComHandler.h"


#include "cnc.h"
#include "PLC.h"
#include "PLC_DEF.h"

#include "quiltSim.h"

#include "seeprom.h"

extern cncState cncStateMachine;

unsigned char userTestEnabled = 0;

const QEvt neddleTopComEvt =
{
	.sig = SET_NEEDLE_TOP_POS_SIG,
};

const QEvt neddleDownComEvt =
{
	.sig = SET_NEEDLE_DOWN_POS_SIG,
};

const QEvt neddleLoopComEvt =
{
	.sig = SET_LOOP_POS_SIG,
};

machineTestEvt_t testUserEvt =
{
};

int tst = 0;
void userTestHandler()
{
	userEvent_t ev;
	unsigned int i;
	int ival;


	if (userEventAvailable())
	{
		if (getUserEvent(&ev))
		{
			if (ev.type == USER_EVT_ENABLE)
			{
				userTestEnabled = !userTestEnabled;
			}
			else if (userTestEnabled)
			switch(ev.type)
			{
			case TEST_JUMP_EVT:
				ival = ev.value;
				testUserEvt.ev.sig = TEST_JUMP_SIG;
				postEvent(&testUserEvt);

				break;

			case X_STEP_EVT:
				ival = ev.value;

				testUserEvt.ev.sig = TEST_SERVO_SIG;
				testUserEvt.testType = 'x';
				testUserEvt.steps.stepCount = ival;
				testUserEvt.steps.stepTime = 31000*abs(ival);

				postEvent(&testUserEvt);
				break;


			case Y_STEP_EVT:
				ival = ev.value;

				testUserEvt.ev.sig = TEST_SERVO_SIG;
				testUserEvt.testType = 'y';
				testUserEvt.steps.stepCount = ival;
				testUserEvt.steps.stepTime = 31000*abs(ival);

				postEvent(&testUserEvt);
			break;

			case VFD_SPEED_EVT:
				if (ev.value == 0)
					stopVfd();
				else if (ev.value < 16)
				{
    				resetBreak();
    				osDelay(100);
    				startVfd(ev.value);
				}
				break;

			case TOGGLE_BREAK_EVT:
				RtoggleBreak();
				break;

			case VFD_STOP_EVT:
				stopVfd();
				break;

			case TOGGLE_VALVE_EVT:
				toggleValve(ev.value);
				break;

			case UNLATCH_START_EVT:
				unlatchQuiltSig();
				break;

			case NEEDLE_UP_POS_COM:
				QActive_post_(&cncStateMachine.super, &neddleTopComEvt, QF_NO_MARGIN, 0);
				break;

			case NEEDLE_DOWN_POS_COM:
				QActive_post_(&cncStateMachine.super, &neddleDownComEvt, QF_NO_MARGIN, 0);
				break;

			case NEEDLE_LOOP_COM:
				QActive_post_(&cncStateMachine.super, &neddleLoopComEvt, QF_NO_MARGIN, 0);
				break;

			case TSTEVT:
				ival = ev.value;
				ival = abs(ival);
				if (ival > 6) ival = 6;
				printF("sp = %d\n", ival);
				setVfdRunnignSpeed(ival);
				break;

			}
		}
	}
}


extern "C" void postTestDone();

extern servoHandler_t xServoHandler;
extern servoHandler_t yServoHandler;
unsigned int refDelay,xy = 1;


extern serialComHandler_t comWatchDog;
extern serialComHandler_t cncStateComHandler;

void writeEVTEST(uint8_t ev);

#include "serialComHandlerV2.h"


uint8_t * McuReqHandler(ioHandler::reqComHandler_t* com, const uint8_t *data, uint32_t len)
{
	if (memcmp(data, "RST", 3) == 0)
	{
		print("system reset\n");
		NVIC_SystemReset();
	}
	return NULL;
}

ioHandler::reqComHandler_t mcuReq
{
	'M',
	McuReqHandler
};

#include "semphr.h"


SemaphoreHandle_t xSemaphore = NULL;
StaticSemaphore_t xMutexBuffer;

uint8_t blink = 0;
extern QueueHandle_t insQueue;

static uint32_t w;
static uint64_t dw;

namespace isoFree
{
BaseType_t xQueuePeekn( QueueHandle_t xQueue, void * const pvBuffer, int index);

}

extern servoHandler_t xServoHandler;
extern servoHandler_t yServoHandler;
extern TaskHandle_t eventGeneratorTaskHand;
extern osMessageQueueId_t eventGeneratorQueue;

const char passcheck__[256]  __attribute__((section(".seeprom"))) = "passcheck\n";
char tempBuf[256];
const int eeprom_int __attribute__((section(".seeprom"))) = 123;

int eeprom_int__ = 0;
extern "C" void userMainThread(void *arg)
{

	cncInsUn_t ins;
	xServoInit();
	yServoInit();


	cncInit();
	cncInsHandlerInit();
	userEventHandlerInit();
	ioHandler::serialComInit();
	ioHandler::addReqHandler(&mcuReq);
	PLC::plcComInit();
	startStateMachine();


	PLC::write(Y11, R_CLOSE);
	osDelay(100);
	PLC::write(Y11, R_OPEN);
	osDelay(100);
	PLC::write(Y11, R_CLOSE);
	osDelay(100);
	PLC::write(Y11, R_OPEN);
	osDelay(2000-3*100);

	xSemaphore = xSemaphoreCreateBinaryStatic( &xMutexBuffer );

	xSemaphoreGive(xSemaphore);

    for(;;)
    {
    	ioHandler::rxHandler();

    	cncButtonAndSigHandler();
    	userTestHandler();


    	timeOutHandler();
    	insHandlerHartBeatCheck();

    	QUILT_SIM::checkForStartSig();

    	if (HAL_GetTick() - refDelay >= 1000)
    	{
    		if (PLC::read(CNC_IDEL_STATE) == R_CLOSE) {

    			PLC::write(Y11, R_TOGGLE);
    		}
    		refDelay = HAL_GetTick();
    	}

    	osThreadYield();
    }
}




extern "C" void testTask(void *arg)
{
	while(1)
	{

	}
}

