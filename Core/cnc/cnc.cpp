/*
 * cnc.cpp
 *
 *  Created on: Aug 25, 2022
 *      Author: sanjay
 */



#include "main.h"
#include "stm32f3xx_hal_flash.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"
#include "serialDebug.h"


#include "serialEvent.h"
#include "servo.h"
#include "X_servo.h"
#include "Y_servo.h"
#include "servoHelpertTim.h"
#include "timer2ServoHelperT.h"

#include "qpc.h"
#include <cnc.h>
#include "cncStateMachine.h"
#include "serialComHandler.h"
#include "PLC.h"



uint32_t eventGeneratorTaskBuffer[ 256 ];
StaticTask_t eventGeneratorTaskControlBlock;
const osThreadAttr_t eventGeneratorTask_attributes = {
  .name = "eventGenerator",
  .cb_mem = &eventGeneratorTaskControlBlock,
  .cb_size = sizeof(eventGeneratorTaskControlBlock),
  .stack_mem = &eventGeneratorTaskBuffer[0],
  .stack_size = sizeof(eventGeneratorTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

TaskHandle_t eventGeneratorTaskHand;
osMessageQueueId_t eventGeneratorQueue;





const QEvt sensorTopEvt = {.sig = SENSOR_TOP_SIG,};
const QEvt sensorDownEvt = {.sig = SENSOR_DOWN_SIG,};
const QEvt sensorHookEvt = {.sig = SENSOR_HOOK_SIG,};
const QEvt sensorClothOutEvt = {.sig = SENSOR_CLT_OUT_SIG,};
const QEvt startEvt = {.sig = START_SIG,};
const QEvt quiltRunEvt = {.sig = QUILT_RUN_SIG,};
const QEvt stopEvt = {.sig = STOP_SIG,};

machineTestEvt_t testEvt = {};


uint8_t vfdRunSpeedLim = 16;
uint8_t vfdRunSpeed = 2;
uint8_t vfdStopSpeed = 1;

extern cncState cncStateMachine;
unsigned int speedDiskCunt = 0;
servoHandler_t xServoHandler, yServoHandler;

uint8_t cncComTxPayload[32], cncComRxPayload[32], cncComRespPayload[32];


const QEvt ctrlRegChange = {.sig = CTRL_REG_SIG,};


SemaphoreHandle_t plcIoSemaphore = NULL;
StaticSemaphore_t plcIoMutexBuffer;

static uint8_t quiltCommand = 0;


void setVfdRunnignSpeed(uint8_t sp) {
	vfdRunSpeed = sp;
}

void servoTask(void *args);

void servoInit(const char *name, servoHandler_t*srv, void* ser)
{
	srv->ser = ser;

    srv->servoQueueHandler = xQueueCreateStatic(
    		8,
			sizeof(servoStepsUn_t),
			(uint8_t *)srv->stepQueueBuf,
			&srv->servoStepQueue
			);

    srv->taskHandle = xTaskCreateStatic(&servoTask, name, 256, srv, osPriorityHigh, srv->stackBuf, &srv->servoTask);
}

void resetServoTask()
{
	osThreadTerminate(xServoHandler.taskHandle);
	osMessageQueueReset(xServoHandler.servoQueueHandler);

	osThreadTerminate(yServoHandler.taskHandle);
	osMessageQueueReset(yServoHandler.servoQueueHandler);

	osThreadTerminate(eventGeneratorTaskHand);
	osMessageQueueReset(eventGeneratorQueue);

	xServoReset();
	yServoReset();
	resetServoHelper();

	xServoHandler.ser = &xServo;
	xServoHandler.taskHandle = xTaskCreateStatic(&servoTask, "xServoTask", 256, &xServoHandler, osPriorityHigh, xServoHandler.stackBuf, &xServoHandler.servoTask);

	yServoHandler.ser = &yServo;
	yServoHandler.taskHandle = xTaskCreateStatic(&servoTask, "yServoTask", 256, &yServoHandler, osPriorityHigh, yServoHandler.stackBuf, &yServoHandler.servoTask);

	eventGeneratorTaskHand = osThreadNew(eventGeneratorTask, NULL, &eventGeneratorTask_attributes);

}



void cncInit()
{
	__HAL_RCC_TIM20_CLK_ENABLE();
	NVIC_EnableIRQ(TIM20_CC_IRQn);
	TIM20->CR1 |= TIM_CR1_CEN;


	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_TSC_IRQn);

	plcIoSemaphore = xSemaphoreCreateBinaryStatic( &plcIoMutexBuffer );
	xSemaphoreGive(plcIoSemaphore);

	servoInit("xServoTask", &xServoHandler, &xServo);
	servoInit("yServoTask", &yServoHandler, &yServo);

	servoHelperInit();
	eventGeneratorQueue = osMessageQueueNew(8, sizeof(helperEvt_t), 0);
	eventGeneratorTaskHand = osThreadNew(eventGeneratorTask, NULL, &eventGeneratorTask_attributes);

}






uint32_t getComIOBits()
{
	uint32_t val;
	val = 0;

	val |= isXonZero()? (1<<ZERO_SIG_POS):(0);
	val |= isNeedleOnTop()?(1U<<NEEDLE_TOP_SIG_POS):(0);
	val |= isNeedleOnDown()?(1U<<NEEDLE_DOWN_SIG_POS):0;
	val |= readSpeedSensor()?(1U<<SPEED_SENSOR_SIG_POS):(0);
//	val |= readQuiltSig()?(1U<<QUILT_SIG_POS):(0);


	val |= PLC::read(MEG_BREAK)?(1U<<BREAK_SIG_POS):(0);
	val |= PLC::read(LOOPER_VALVE)?(1U<<LOOPER_VALVE_POS):(0);
	val |= PLC::read(LOWER_THREAD_VALVE)?(1U<<LOWER_THREAD_LOOSE_VALVE_POS):(0);
	val |= PLC::read(UPPER_THREAD_VALVE)?(1U<<UPPER_THREAD_LOOSE_VALVE_POS):(0);

	val |= PLC::read(START_BUTTON)?(1U<<START_BUTTON_POS):(0);
	val |= PLC::read(STOP_BUTTON)?(1U<<STOP_BUTTON_POS):(0);

	return val;
}




unsigned char isBreakSet()
{
	return PLC::read(MEG_BREAK);
}



void RsetBreak()
{
	print("set break\n");
	PLC::write(TEST_BREAK, R_CLOSE);
}
void RresetBreak()
{
	print("reset break\n");
	PLC::write(TEST_BREAK, R_OPEN);
}

void RtoggleBreak()
{
	PLC::write(TEST_BREAK, R_TOGGLE);
}

void setBreak()
{
	stopVfd();
	PLC::write(MEG_BREAK, R_CLOSE);
}

void resetBreak()
{
	PLC::write(MEG_BREAK, R_OPEN);
}

void toggleBreak()
{
	PLC::write(MEG_BREAK, R_TOGGLE);
}


void startVfd(unsigned char speedIndex)
{
	stopVfd();

	if (PLC::read(MEG_BREAK) == R_OPEN)
	{
		PLC::write(VFD_M1, (speedIndex & 1) ? R_CLOSE : R_OPEN);
		PLC::write(VFD_M2, (speedIndex & 2) ? R_CLOSE : R_OPEN);
		PLC::write(VFD_M3, (speedIndex & 4) ? R_CLOSE : R_OPEN);
		PLC::write(VFD_M4, (speedIndex & 8) ? R_CLOSE : R_OPEN);
		PLC::write(VFD_RUN, R_CLOSE);
	}
}
void stopVfd()
{
	PLC::write(VFD_RUN, R_OPEN);
	PLC::write(VFD_M1, R_OPEN);
	PLC::write(VFD_M2, R_OPEN);
	PLC::write(VFD_M3, R_OPEN);
	PLC::write(VFD_M4, R_OPEN);
}


unsigned char isValveSet(unsigned char valve)
{
	switch (valve) {
		case UPPER_THREAD_LOOSE_VALVE:
			return PLC::read(UPPER_THREAD_VALVE);
		case LOWER_THREAD_LOOSE_VALVE:
			return PLC::read(LOWER_THREAD_VALVE);
		case THREAD_CUT_VALVE:
			return PLC::read(LOOPER_VALVE);

		default:
			break;
	}
	return 0;
}

void executeValveIns(setValveIns_t ins){
	if (ins.type != insTypes::SET_VALVE_INS)
		return;

	if (ins.action == ACTION_SET) {
		if (ins.valve & (UPPER_THREAD_LOOSE_VALVE)) PLC::write(UPPER_THREAD_VALVE, R_CLOSE);
		if (ins.valve & (LOWER_THREAD_LOOSE_VALVE)) PLC::write(LOWER_THREAD_VALVE, R_CLOSE);
		if (ins.valve & (THREAD_CUT_VALVE)) PLC::write(LOOPER_VALVE, R_CLOSE);
	}
	else if (ins.action == ACTION_RESET) {
		if (ins.valve & (UPPER_THREAD_LOOSE_VALVE)) PLC::write(UPPER_THREAD_VALVE, R_OPEN);
		if (ins.valve & (LOWER_THREAD_LOOSE_VALVE)) PLC::write(LOWER_THREAD_VALVE, R_OPEN);
		if (ins.valve & (THREAD_CUT_VALVE)) PLC::write(LOOPER_VALVE, R_OPEN);
	}
	else if (ins.action == ACTION_SET) {
		if (ins.valve & (UPPER_THREAD_LOOSE_VALVE)) PLC::write(UPPER_THREAD_VALVE, R_TOGGLE);
		if (ins.valve & (LOWER_THREAD_LOOSE_VALVE)) PLC::write(LOWER_THREAD_VALVE, R_TOGGLE);
		if (ins.valve & (THREAD_CUT_VALVE)) PLC::write(LOOPER_VALVE, R_TOGGLE);
	}

}

void writeValves(uint8_t valveMask)
{
	PLC::write(UPPER_THREAD_VALVE,  (valveMask & (UPPER_THREAD_LOOSE_VALVE))?(R_CLOSE):(R_OPEN));
	PLC::write(LOWER_THREAD_VALVE,  (valveMask & (LOWER_THREAD_LOOSE_VALVE))?(R_CLOSE):(R_OPEN));
	PLC::write(LOOPER_VALVE,  (valveMask & (THREAD_CUT_VALVE))?(R_CLOSE):(R_OPEN));
}

void setValve(unsigned char valve)
{
	switch (valve) {
		case UPPER_THREAD_LOOSE_VALVE:
			PLC::write(UPPER_THREAD_VALVE, R_CLOSE);
			break;
		case LOWER_THREAD_LOOSE_VALVE:
			PLC::write(LOWER_THREAD_VALVE, R_CLOSE);
			break;
		case THREAD_CUT_VALVE:
			PLC::write(LOOPER_VALVE, R_CLOSE);
			break;

		default:
			break;
	}
}

void resetValve(unsigned char valve)
{
	switch (valve) {
		case UPPER_THREAD_LOOSE_VALVE:
			PLC::write(UPPER_THREAD_VALVE, R_OPEN);
			break;
		case LOWER_THREAD_LOOSE_VALVE:
			PLC::write(LOWER_THREAD_VALVE, R_OPEN);
			break;
		case THREAD_CUT_VALVE:
			PLC::write(LOOPER_VALVE, R_OPEN);
			break;

		default:
			break;
	}
}

void toggleValve(unsigned char valve)
{

	switch (valve) {
		case UPPER_THREAD_LOOSE_VALVE:
			PLC::write(UPPER_THREAD_VALVE, R_TOGGLE);
			break;
		case LOWER_THREAD_LOOSE_VALVE:
			PLC::write(LOWER_THREAD_VALVE, R_TOGGLE);
			break;
		case THREAD_CUT_VALVE:
			PLC::write(LOOPER_VALVE, R_TOGGLE);
			break;

		default:
			break;
	}
}

int readQuiltCommand()
{
	if (quiltCommand) return 1;
	return 0;
}

uint8_t readQuiltSig()
{
	return PLC::read(QUILT_SIGNAL);
}
unsigned int refUnlatchRel;

void unlatchQuiltSig()
{
	PLC::write(UNLATCH_QUILT_OUT, R_CLOSE);
	PLC::write(LATCH_QUILT_OUT, R_OPEN);
}

void latchQuiltSig()
{
	PLC::write(LATCH_QUILT_OUT, R_CLOSE);
	PLC::write(UNLATCH_QUILT_OUT, R_OPEN);
}


void unlatchAutoRelHandler()
{
	using namespace PLC;

	if (read(LATCH_QUILT_OUT, CHANGE_READ) || read(UNLATCH_QUILT_OUT, CHANGE_READ))
		refUnlatchRel = HAL_GetTick();

	if (read(LATCH_QUILT_OUT) || read(UNLATCH_QUILT_OUT))
	{
		if (HAL_GetTick() - refUnlatchRel >= 200)
		{
			write(LATCH_QUILT_OUT, R_OPEN);
			write(UNLATCH_QUILT_OUT, R_OPEN);
		}
	}

	if (read(STOP_BUTTON))
	{
		unlatchQuiltSig();
	}
	else if (read(STOP_BUTTON))
	{
		latchQuiltSig();
	}
}





unsigned char isNeedleOnTop()
{
	return PLC::read(NEEDLE_TOP_SENSOR, NEEDLE_TOP_NPN_PNP);
}

unsigned char isNeedleOnDown()
{
	return PLC::read(NEEDLE_DOWN_SENSOR, NEEDLE_DOWN_NPN_PNP);
}

unsigned char readSpeedSensor()
{
	return PLC::read(NEEDLE_POS_SENSOR, NEEDLE_POS_NPN_PNP);
}

unsigned char isXonZero()
{
	return PLC::read(ZERO_SENSOR, ZERO_SENSOR_NPN_PNP);
}


#define START_SIG_MASK		1

unsigned int refDebounce;
unsigned int resVal, stableVal;
unsigned char newValAvailable, unStable = 0;

static const QEvt neddleTopComEvt =
{
	.sig = SET_NEEDLE_TOP_POS_SIG,
};

static const QEvt neddleDownComEvt =
{
	.sig = SET_NEEDLE_DOWN_POS_SIG,
};

static const QEvt neddleLoopComEvt =
{
	.sig = SET_LOOP_POS_SIG,
};


void enterIdleState()
{
	xSemaphoreTake(plcIoSemaphore, portMAX_DELAY);
	PLC::write(CNC_IDEL_STATE, R_CLOSE);
	xSemaphoreGive(plcIoSemaphore);
}

void exitIdleState()
{
	xSemaphoreTake(plcIoSemaphore, portMAX_DELAY);
	PLC::write(CNC_IDEL_STATE, R_OPEN);
	xSemaphoreGive(plcIoSemaphore);
}

void cncButtonAndSigHandler()
{

	using namespace PLC;

	if (read(QUILT_SIGNAL, CHANGE_READ))
	{
		unStable = 1;
		refDebounce = HAL_GetTick();
	}

	if (unStable)
	{
		if (HAL_GetTick() - refDebounce >= 100)
		{
			unStable = 0;
			if (read(QUILT_SIGNAL))
				QActive_post_(&cncStateMachine.super, &startEvt, QF_NO_MARGIN, 0);

		}
	}




	xSemaphoreTake(plcIoSemaphore, portMAX_DELAY);

if (read(CNC_IDEL_STATE))
{

	if (read(MOVE_RIGHT, RISE_READ))
	{
		moveServoContStart(200, 0);
	}
	else if (read(MOVE_LEFT, RISE_READ))
	{
		moveServoContStart(-200, 0);
	}
	else if (read(MOVE_UP, RISE_READ))
	{
		moveServoContStart(0, 200);
	}
	else if (read(MOVE_DOWN, RISE_READ))
	{
		moveServoContStart(0, -200);
	}
	else if (read(MOVE_RIGHT, NC_READ)&&
			read(MOVE_LEFT, NC_READ) &&
			read(MOVE_UP, NC_READ) &&
			read(MOVE_DOWN, NC_READ))
	{
		moveServoContStop();
	}


	if (read(NEEDLE_UP, RISE_READ))
	{
		QActive_post_(&cncStateMachine.super, &neddleTopComEvt, QF_NO_MARGIN, 0);
	}
	else if (read(NEEDLE_DOWN, RISE_READ))
	{
		QActive_post_(&cncStateMachine.super, &neddleDownComEvt, QF_NO_MARGIN, 0);
	}
	else if (read(NEEDLE_LOOP, RISE_READ))
	{
		QActive_post_(&cncStateMachine.super, &neddleLoopComEvt, QF_NO_MARGIN, 0);
	}

	if (read(TEST_BREAK, RISE_READ))
		setBreak();
	else if (read(TEST_BREAK, NC_READ))
		resetBreak();

}
else
{

	write(MOVE_RIGHT, R_OPEN);
	write(MOVE_LEFT, R_OPEN);
	write(MOVE_UP, R_OPEN);
	write(MOVE_DOWN, R_OPEN);
	write(NEEDLE_UP, R_OPEN);
	write(NEEDLE_DOWN, R_OPEN);
	write(NEEDLE_LOOP, R_OPEN);
	write(TEST_BREAK, read(MEG_BREAK));
}

	xSemaphoreGive(plcIoSemaphore);


	if (read(FDR_RING) && read(FDR_ALIVE) && read(QUILT_SIGNAL))
	{
		if (quiltCommand == 0)
		{
			quiltCommand = 1;
			QActive_post_(&cncStateMachine.super, &quiltRunEvt, QF_NO_MARGIN, 0);
		}
	}
	else
	{
		if (quiltCommand == 1)
		{
			quiltCommand = 0;
			QActive_post_(&cncStateMachine.super, &stopEvt, QF_NO_MARGIN, 0);
		}

	}

}



void servoTask(void *args)
{

	servoHandler_t *srv = (servoHandler_t*) args;
	servo *Servo = (servo*)srv->ser;
	QueueHandle_t stepQueue = srv->servoQueueHandler;
	servoStepsUn_t steps;
	char name[20];
	const char *n = osThreadGetName(osThreadGetId());
	memcpy(name, n, strlen(n)+1);
	unsigned char dir;
	unsigned int cnt;
	uint32_t errF, err, nst, st;
	double temp;

	if (name[0] == 'x');
	print("servo online\n");

	while(1)
	{
		if (osMessageQueueGet(stepQueue, &steps, 0, portMAX_DELAY) == osOK)
		{

			//printF("%c = %d\n", name[0], steps.stepCount);

			switch(steps.type)
			{
			case SERVO_STITCH_STEPS:

				if (steps.stitchStep.stepCount == 0)
				{
					Servo->write_step(steps.stitchStep.stepTime, time_ch);
					Servo->write_pause_time(72000, event_EP_ch);
					break;
				}


				dir = (steps.stitchStep.stepCount < 0)?(negitive_ch):(positive_ch);
				cnt = (steps.stitchStep.stepCount < 0)?(-steps.stitchStep.stepCount):(steps.stitchStep.stepCount);

				nst = steps.stitchStep.stepTime;

				temp = (double)nst/cnt;
				st = temp;
			    errF = (temp - st) * 10000 + 1;
			    err = 0;
				for (unsigned int i = 0, t = 0; i < cnt; i++)
				{
					t = st;
					err += errF;

					if (err >= 10000)
					{
						t += 1;
						err -= 10000;
					}
					Servo->write_step(t , dir);
				}
				Servo->write_pause_time(72000, event_EP_ch);

				break;


			case SERVO_JUMP_STEPS:
				if (steps.stitchStep.stepCount == 0)
				{
					Servo->write_step(steps.jumpStep.stepTime, time_ch);
					break;
				}

				dir = (steps.jumpStep.stepCount < 0)?(negitive_ch):(positive_ch);
				cnt = (steps.jumpStep.stepCount < 0)?(-steps.jumpStep.stepCount):(steps.jumpStep.stepCount);

				nst = steps.jumpStep.stepTime;

				temp = (double)nst/cnt;
				st = temp;
			    errF = (temp - st) * 10000 + 1;
			    err = 0;
				for (unsigned int i = 0, t = 0; i < cnt; i++)
				{
					t = st;
					err += errF;

					if (err >= 10000)
					{
						t += 1;
						err -= 10000;
					}
					Servo->write_step(t , dir);
				}
				break;

			case SERVO_QUANT_PAUSE:
				Servo->write_pause_time(steps.jumpStep.stepTime, event_EP_ch);
				break;

			}

		}
	}
}

uint16_t trigEn = 0, stopingSteps = 0;

int servoTrigger()
{
//	while (trigEn == 0)
//	{
//		print("error\n");
//		osDelay(500);
//	}

	if ((TIM3->CR1 & TIM_CR1_CEN) || (TIM4->CR1 & TIM_CR1_CEN))
	while (1)
	{
		TIM3->CR1 &= ~TIM_CR1_CEN;
		TIM4->CR1 &= ~TIM_CR1_CEN;
		startVfd(1);
		print("error servo not paused\n");
		osDelay(500);
		startVfd(0);

		return 0;
	}

	//for timer 3 and 4 synchronization
	TIM3->CR2 = (0b001 << TIM_CR2_MMS_Pos);
	TIM3->SMCR = TIM_SMCR_MSM | (0b110 << TIM_SMCR_SMS_Pos);

	TIM4->SMCR = (0B0110<<TIM_SMCR_SMS_Pos) | (0b010 << TIM_SMCR_TS_Pos);

	xServo.servo_DMA->Instance->CCR |= DMA_CCR_EN;
	yServo.servo_DMA->Instance->CCR |= DMA_CCR_EN;


	trigEn--;
	xServo.trigger_timre(0);
	yServo.trigger_timre(0);

	return 1;
}


int moveServo(int32_t xStep, int32_t yStep, uint32_t stepTime, uint8_t Trig)
{
	servoStepsUn_t x, y;
	x.type = SERVO_STITCH_STEPS;
	x.stitchStep.stepCount = xStep;
	x.stitchStep.stepTime = stepTime;
	osMessageQueuePut(xServoHandler.servoQueueHandler, &x, 0, portMAX_DELAY);

	y.type = SERVO_STITCH_STEPS;
	y.stitchStep.stepCount = yStep;
	y.stitchStep.stepTime = stepTime;
	osMessageQueuePut(yServoHandler.servoQueueHandler, &y, 0, portMAX_DELAY);

	if (Trig)
	{
		osDelay(100);
		yServo.trigger_timre(0);
		xServo.trigger_timre(0);
	}
	return 0;
}




static uint8_t timerArmed = 0;
static uint32_t refTimeOut, timOUtDelay;
static const void *timeOutEvt;

void armTimer(uint32_t delay, const void *ev)
{
	refTimeOut = HAL_GetTick();
	timeOutEvt = ev;
	timerArmed = 1;
	timOUtDelay = delay;
}

void disArmTimer()
{
	timerArmed = 0;
}

void timeOutHandler()
{
	if (timerArmed)
	{
		if (HAL_GetTick() - refTimeOut >= timOUtDelay)
		{
			taskENTER_CRITICAL();
			timerArmed = 0;
			postEvent(timeOutEvt);
			taskEXIT_CRITICAL();
		}
	}
}



int getStopingSteps()
{
	return stopingSteps;
}
uint8_t vfdSpeed = 1;

int makeStitch(stitchIns_t *ins)
{

	uint8_t ret = 0;
	uint8_t temp;

	servoStepsUn_t x, y;
	x.type = SERVO_STITCH_STEPS;
	x.stitchStep.stepCount = ins->xStep;
	x.stitchStep.stepTime = ins->stepTime;
	osMessageQueuePut(xServoHandler.servoQueueHandler, &x, 0, portMAX_DELAY);

	y.type = SERVO_STITCH_STEPS;
	y.stitchStep.stepCount = ins->yStep;
	y.stitchStep.stepTime = ins->stepTime;
	osMessageQueuePut(yServoHandler.servoQueueHandler, &y, 0, portMAX_DELAY);

	trigEn++;


	if (ins->vfdSpeed == 'R')
	{
		temp = PLC::read(D2);
		if (temp == 0)
			PLC::write(D2, (uint32_t)1);

		stopingSteps = 0;
		if (vfdSpeed < temp)
			vfdSpeed++;
		else
		vfdSpeed = temp;

		if (vfdSpeed == 0)
		vfdSpeed = 1;

		ret = 1;
	}
	else if (ins->vfdSpeed == 'Z')
	{
		stopingSteps = 0;
		vfdSpeed = 2;
		ret = 1;
	}
	else if (ins->vfdSpeed == 'S')
	{
		if (vfdSpeed > 2)
			vfdSpeed = 2;
		else if (vfdSpeed > 1)
			vfdSpeed--;
		else
			vfdSpeed = 1;

		if (vfdSpeed == 1)
		{
			stopingSteps++;
			printF("stop = %d\n", stopingSteps);
		}

		if (stopingSteps >= 10)
		{
			stopingSteps = 10;
			ret = 2;
		}
	}
	else if (ins->vfdSpeed >= 1 && ins->vfdSpeed <= 15)
	{
		if (vfdSpeed > ins->vfdSpeed)
			vfdSpeed = ins->vfdSpeed;
	}
	startVfd(vfdSpeed);

	return ret;

}

static uint8_t valveCommandArray[8];
static uint8_t valveCommandIndexHead, valveCommandIndexTail;

void accJumpValve() {
	writeValves(valveCommandArray[valveCommandIndexTail++]);
}

int xmovecontSteps, ymovecontSteps;
uint8_t servoMoveRunning;

const QEvt moveServoEvt = {.sig = MOVE_SEROV_SIG,};

void moveServoContStart(int x, int y) {
	xmovecontSteps = x;
	ymovecontSteps = y;
	servoMoveRunning = 1;
	QActive_post_(&cncStateMachine.super, &moveServoEvt, QF_NO_MARGIN, 0);
}

void moveServoContStop() {
	xmovecontSteps = 0;
	ymovecontSteps = 0;
	servoMoveRunning = 0;
}

int moveSerovContinuous() {

	if (!servoMoveRunning)
		return 0;
	servoStepsUn_t Sx, Sy;

	Sx.type = SERVO_JUMP_STEPS;
	Sx.jumpStep.stepCount = (xmovecontSteps > 1000)?(100):(xmovecontSteps);
	Sx.jumpStep.stepTime = 100*1000*72;

	Sy.type = SERVO_JUMP_STEPS;
	Sy.jumpStep.stepCount = (ymovecontSteps > 1000)?(1000):(ymovecontSteps);
	Sy.jumpStep.stepTime = 100*1000*72;

	helperEvt_t hlp;
	hlp.type = HALPER_GEN_EV;
	hlp.time = 100*1000*72;

	osMessageQueuePut(xServoHandler.servoQueueHandler, &Sx, 0, 0);
	osMessageQueuePut(yServoHandler.servoQueueHandler, &Sy, 0, 0);
	osMessageQueuePut(eventGeneratorQueue, &hlp, 0, 0);
	return 1;
}



void makeJump(jumpIns_t *ins) {

	servoStepsUn_t x, y;

	valveCommandArray[valveCommandIndexHead++ & 0xF] = ins->valveMask;

	x.type = SERVO_JUMP_STEPS;
	x.jumpStep.stepCount = ins->xStep;
	x.jumpStep.stepTime = ins->stepTime;

	if (osMessageQueuePut(xServoHandler.servoQueueHandler, &x, 0, 0))
		while(1);

	y.type = SERVO_JUMP_STEPS;
	y.jumpStep.stepCount = ins->yStep;
	y.jumpStep.stepTime = ins->stepTime;
	if(osMessageQueuePut(yServoHandler.servoQueueHandler, &y, 0, 0))
		while(1);

	helperEvt_t hlp;
	hlp.type = HALPER_GEN_EV;
	hlp.time = ins->stepTime;
	if(osMessageQueuePut(eventGeneratorQueue, &hlp, 0, 0))
		while(1);

}

void terminateJump() {
	servoStepsUn_t x, y;

	x.type = SERVO_QUANT_PAUSE;
	x.jumpStep.stepTime = 72000;
	osMessageQueuePut(xServoHandler.servoQueueHandler, &x, 0, portMAX_DELAY);

	y.type = SERVO_QUANT_PAUSE;
	y.jumpStep.stepTime = 72000;
	osMessageQueuePut(yServoHandler.servoQueueHandler, &y, 0, portMAX_DELAY);

	helperEvt_t hlp;
	hlp.type = HALPER_PAUSE_EV;
	hlp.time = 72000*2;
	osMessageQueuePut(eventGeneratorQueue, &hlp, 0, portMAX_DELAY);
}


void helperTigger() {
	xyServoHelper.trigger(&xyServoHelper);
}


uint8_t xServoPaused() {
	return xServo.isServoPaused();
}
uint8_t yServoPaused() {
	return yServo.isServoPaused();
}
uint8_t xyHelperPaused() {
	return xyServoHelper.isHelperPaused();
}

void eventGeneratorTask(void *argument) {
	helperEvt_t ev;
	print("helper online\n");

	for(;;) {

		if (osMessageQueueGet(eventGeneratorQueue, &ev, 0, portMAX_DELAY) != osOK)
			continue;

		if (ev.type == HALPER_GEN_EV) {
			xyServoHelper.writeEventTime(ev.time, send_ev);
		}
		else if (ev.type == HALPER_PAUSE_EV) {
			xyServoHelper.writeEventTime(ev.time, event_EP_ch);
		}
	}
}



extern "C" void TIM20_CC_IRQHandler()
{
	if ((TIM20->DIER & TIM_DIER_CC1IE) && (TIM20->SR & TIM_SR_CC1IF))//NEDDLE_POS_SENSOR_PIN GPIOB0
	{
		TIM20->DIER &= ~TIM_DIER_CC1IE;
		TIM20->SR &= ~TIM_SR_CC1IF;


		if (readSpeedSensor())
		{
			if (speedDiskCunt < 50)
			{
				speedDiskCunt++;
				if (speedDiskCunt == 45)
				{
					__NVIC_SetPriority(TIM20_CC_IRQn,0x0D);
					QActive_postFromISR_(&cncStateMachine.super, &sensorClothOutEvt, QF_NO_MARGIN, 0, 0);
					__NVIC_SetPriority(TIM20_CC_IRQn,0x0C);
				}
				if (speedDiskCunt == 25)
				{
					__NVIC_SetPriority(TIM20_CC_IRQn,0x0D);
					QActive_postFromISR_(&cncStateMachine.super, &sensorDownEvt, QF_NO_MARGIN, 0, 0);
					__NVIC_SetPriority(TIM20_CC_IRQn,0x0C);
				}
			}
		}
	}

	if ((TIM20->DIER & TIM_DIER_CC2IE) && (TIM20->SR & TIM_SR_CC2IF)) //NEDDLE_TOP_SENSOR_PIN GPIOB1
	{
		TIM20->DIER &= ~TIM_DIER_CC2IE;
		TIM20->SR &= ~TIM_SR_CC2IF;

		if (isNeedleOnTop())
		{
			speedDiskCunt = 0;
			__NVIC_SetPriority(TIM20_CC_IRQn,0x0D);
			QActive_postFromISR_(&cncStateMachine.super, &sensorTopEvt, QF_NO_MARGIN, 0, 0);
			__NVIC_SetPriority(TIM20_CC_IRQn,0x0C);
		}
	}

	if ((TIM20->DIER & TIM_DIER_CC3IE) && (TIM20->SR & TIM_SR_CC3IF)) //NEDDLE_DONW_SENSOR_PIN GPIOB1
	{
		TIM20->DIER &= ~TIM_DIER_CC3IE;
		TIM20->SR &= ~TIM_SR_CC3IF;

		if (isNeedleOnDown())
		{
			__NVIC_SetPriority(TIM20_CC_IRQn,0x0D);
			QActive_postFromISR_(&cncStateMachine.super, &sensorDownEvt, QF_NO_MARGIN, 0, 0);
			__NVIC_SetPriority(TIM20_CC_IRQn,0x0C);
		}
	}
}


extern "C" void EXTI0_IRQHandler() //NEDDLE_POS_SENSOR_PIN GPIOB0
{
	EXTI->PR = EXTI_PR_PIF0;
	TIM20->CCR1 = TIM20->CNT + 500;
	TIM20->SR &= ~TIM_SR_CC1IF;
	TIM20->DIER |= TIM_DIER_CC1IE;
}


extern "C" void EXTI1_IRQHandler() //NEDDLE_TOP_SENSOR_PIN GPIOB1
{
	EXTI->PR = EXTI_PR_PIF1;
	TIM20->CCR2 = TIM20->CNT + 1000;
	TIM20->SR &= ~TIM_SR_CC2IF;
	TIM20->DIER |= TIM_DIER_CC2IE;
}

extern "C" void EXTI2_TSC_IRQHandler() //NEDDLE_DOWN_SENSOR_PIN GPIOB1
{
	EXTI->PR = EXTI_PR_PIF2;
	TIM20->CCR3 = TIM20->CNT + 1000;
	TIM20->SR &= ~TIM_SR_CC3IF;
	TIM20->DIER |= TIM_DIER_CC3IE;
}

