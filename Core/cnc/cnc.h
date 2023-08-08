/*
 * cncDef.h
 *
 *  Created on: Aug 25, 2022
 *      Author: sanjay
 */



#ifndef CNC_CNC_H_
#define CNC_CNC_H_

#include "PLC.h"
#include "qpc.h"

#ifdef __cplusplus
extern "C" {
#endif


enum outputEnum
{
    ENTER_TEST_STATE = '~',
    EXIT_TEST_STATE = '>',

	OUTPUT_X_STEP = 'x',
	OUTPUT_Y_STEP = 'y',

	OUTPUT_SET_VALVE_EVT = 'V',
	OUTPUT_RESET_VALVE_EVT = 'v',

	OUTPUT_SET_BREAK_EVT = 'B',
	OUTPUT_RESET_BREAK_EVT = 'b',
	OUTPUT_TOGGLE_BREAK_EVT = 'T',

	OUTPUT_UNLATCH_START_EVT = 's',

	OUTPUT_NEEDLE_UP_POS_COM = 'N',
	OUTPUT_NEEDLE_LOOP_COM = 'R',
	OUTPUT_NEEDLE_DOWN_POS_COM = 'n',
	OUTPUT_TEST_MODE_COM = '0',
	OUTPUT_TEST_MODE_EXIT_COM = '1',

	OUTPUT_TEST_QUILT_EVT = 'Q',

};

enum io_com_bit_pos
{
	ZERO_SIG_POS,
	NEEDLE_TOP_SIG_POS,
	NEEDLE_DOWN_SIG_POS,
	SPEED_SENSOR_SIG_POS,


	BREAK_SIG_POS,
	LOWER_THREAD_LOOSE_VALVE_POS,
	UPPER_THREAD_LOOSE_VALVE_POS,
	LOOPER_VALVE_POS,

	THREAD_BREACK_SIG_POS,

	QUILT_SIG_POS,
    START_BUTTON_POS,
    STOP_BUTTON_POS,
    PROTECTION_SIG_POS,
};




#define NEEDLE_TOP_NPN_PNP		NO_READ
#define NEEDLE_DOWN_NPN_PNP		NO_READ
#define NEEDLE_POS_NPN_PNP		NO_READ
#define ZERO_SENSOR_NPN_PNP		NO_READ
#define PROTECTION_NPN_PNP		NO_READ


enum valveNo
{

	UPPER_THREAD_LOOSE_VALVE = 1,
	LOWER_THREAD_LOOSE_VALVE = 2,
	THREAD_CUT_VALVE = 4,
};


enum servoStepType
{
	SERVO_STITCH_STEPS,
	SERVO_QUANT_STEPS,
	SERVO_QUANT_PAUSE,
	SERVO_JUMP_STEPS,
};

enum cncIoSig
{
	SENSOR_CHANGED = 1,
};

typedef struct
{
	unsigned char type;
	int stepCount;
	unsigned int stepTime;
}servoStitchSteps_t;

typedef struct
{
	unsigned char type;
	int stepCount;
	unsigned int initDeadTime;
	unsigned int stepTime;
	unsigned int endDeadTime;
}servoQuantSteps_t;

typedef struct
{
	unsigned char type;
	int stepCount;
	unsigned int stepTime;
}servoJumpSteps_t;


typedef union
{
	unsigned char type;
	servoStitchSteps_t stitchStep;
	servoQuantSteps_t servoQuantSteps;
	servoJumpSteps_t jumpStep;

}servoStepsUn_t;

typedef struct
{
	uint32_t stackBuf[256];
	StaticTask_t servoTask;
	TaskHandle_t taskHandle;

	servoStepsUn_t stepQueueBuf[8];
	StaticQueue_t servoStepQueue;
	QueueHandle_t servoQueueHandler;
	void* ser;
}servoHandler_t;


typedef struct
{
	QEvt ev;
	uint8_t testType;
	servoStitchSteps_t steps;
	uint8_t valve;
}machineTestEvt_t;

typedef struct
{
	QEvt ev;
	uint8_t testType;
	int x, y;
	uint32_t stepTime;
}servoMoveEvt_t;

enum dstInsTypes
{
	DST_STITCH_INS,
	DST_JUMP_INS,
	DST_ZERO_INS,
};

typedef struct
{
	uint8_t type;
	uint32_t index;
	int8_t x;
	int8_t y;
}dstIns_t;


enum helperEvType
{
	HALPER_GEN_EV,
	HALPER_PAUSE_EV
};

typedef struct
{
	uint8_t type;
	uint32_t time;
	void *ev;
}helperEvt_t;



enum insTypes
{
	NONE_INS = 0,
	ZERO_INS = 1,
	STITCH_INS,
	JUMP_INS,
	SET_VALVE_INS,
	PRINT_INS,
	TEST_INS,
	STOP_INS,

	START_JUMP,
	START_STITCH,
	INS_MAX,
};

static const char* insName[] =
{
"NONE_INS",
"ZERO_INS",
"STITCH_INS",
"JUMP_INS",
"SET_VALVE_INS",
"PRINT_INS",
"STOP_INS",
};

typedef struct
{
	unsigned char type;
	unsigned int id;

	uint8_t valveMask;
	int xStep;
	int yStep;
	unsigned int stepTime;
}jumpIns_t;

typedef struct
{
	unsigned char type;
	unsigned int id;

	uint8_t vfdSpeed;
	int xStep;
	int yStep;
	unsigned int stepTime;
}stitchIns_t;



typedef struct
{
	unsigned char type;
	unsigned int id;

	uint8_t vfdSpeed;
	int xStep;
	int yStep;
	unsigned int stepTime;
}zeroIns_t;



typedef struct
{
	unsigned char type;
	unsigned int id;
}cncIns_t;

enum action_t{
	ACTION_NONE = 0,
	ACTION_SET = 1,
	ACTION_RESET = 2,
	ACTION_TOGGLE = 3,
};

typedef struct
{
	unsigned char type;
	unsigned int id;
	unsigned char valve;
	unsigned char action;
	unsigned int delay;
}setValveIns_t;

typedef union
{
	unsigned char type;
	cncIns_t cncIns;
	stitchIns_t stitchIns;
	zeroIns_t zeroIns;
	jumpIns_t jumpIns;
	setValveIns_t valveIns;

}cncInsUn_t;



void servoInit(const char *name, servoHandler_t*, void*);
void resetServoTask();

void cncInit();
void setVfdRunnignSpeed(uint8_t sp);
void enterIdleState();
void exitIdleState();

unsigned char isBreakSet();

void RsetBreak();
void RresetBreak();
void RtoggleBreak();

void setBreak();
void resetBreak();
void toggleBreak();

void startVfd(unsigned char speedIndex);
void stopVfd();



unsigned char isValveSet(unsigned char valve);
void executeValveIns(setValveIns_t ins);
void writeValves(uint8_t valveMask);
void setValve(unsigned char valve);
void resetValve(unsigned char valve);
void toggleValve(unsigned char valve);



int readQuiltCommand();
uint8_t readQuiltSig();
void unlatchQuiltSig();
void latchQuiltSig();
void unlatchAutoRelHandler();


unsigned char isXonZero();
unsigned char isNeedleOnTop();
unsigned char isNeedleOnDown();



unsigned char isNeedleOnHook();
unsigned char readSpeedSensor();

void cncButtonAndSigHandler();


// manual servo move ---------------------------------------
void moveServoContStart(int x, int y);
void moveServoContStop();
int moveSerovContinuous();

// check if servo is paused ---------------------------------------------
uint8_t xServoPaused();
uint8_t yServoPaused();
uint8_t xyHelperPaused();

// get sensor reg value ------------------------------------------------------
uint32_t getComIOBits();

// stitch instruction function ------------------------------------------------------
int getStopingSteps();
int makeStitch(stitchIns_t *ins);
int servoTrigger();

// jump instruction functions --------------------------------------------------------
void makeJump(jumpIns_t *ins);
void terminateJump();
void helperTigger();


int moveServo(int32_t x, int32_t y, uint32_t stepTime, uint8_t Trig);
void eventGeneratorTask(void *argument);
void onChangeComSender();

// timer functions meant to be used in state machine
void armTimer(uint32_t delay, const void *ev);
void disArmTimer();
void timeOutHandler();

#ifdef __cplusplus
}
#endif

#endif /* CNC_CNC_H_ */
