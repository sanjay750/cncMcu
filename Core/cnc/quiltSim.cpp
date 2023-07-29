/*
 * quiltSim.cpp
 *
 *  Created on: Jul 4, 2023
 *      Author: sanjay
 */


#include "main.h"
#include "quiltSim.h"
#include "PLC.h"
#include "cncStateMachine.h"
#include "cnc.h"
#include "qpc.h"



namespace QUILT_SIM
{

#define RPM_RPS(s) (s/60.0)
#define RPS_FREQ(s)	(1.0/s)
#define RPM_Tms(s) ((uint32_t)(RPS_FREQ(RPM_RPS(s)) * 1000))

const uint32_t vfdSpeedIndexToTimeOut[] =
{
	RPM_Tms(10.0),
	RPM_Tms(50.0),
	RPM_Tms(100.0),
	RPM_Tms(300.0),
	RPM_Tms(500.0),
	RPM_Tms(600.0),
	RPM_Tms(800.0),
	RPM_Tms(900.0),
	RPM_Tms(1200.0),
};

const QEvt simStartEvent = {SIM_START_SIG};
const QEvt simStopEvent = {SIM_STOP_SIG};
const QEvt quiltRunEvt = {.sig = QUILT_RUN_SIG,};

unsigned char quiltCommand = 0;


void checkForStartSig()
{
	if (PLC::read(SIM_QUILT_SIG, CHANGE_READ) == R_CLOSE)
	{
		if (PLC::read(SIM_QUILT_SIG) == R_CLOSE)
		{
			postEvent(&simStartEvent);
			PLC::write(Y11, R_CLOSE);
		}
		else
		{
			postEvent(&simStopEvent);
			PLC::write(Y11, R_OPEN);
		}
	}

	if (PLC::read(SIM_QUILT_SIG) && PLC::read(FDR_ALIVE) && PLC::read(FDR_RING))
	{
		if (quiltCommand == 0)
		{
			quiltCommand = 1;
			postEvent(&quiltRunEvt);
		}
	}
	else
	{
		if (quiltCommand == 1)
		{
			quiltCommand = 0;
		}
	}
}

int simReadQuiltSig()
{
	return quiltCommand;
}

void simUnlat()
{
	PLC::write(SIM_QUILT_SIG, R_OPEN);
}
static char insStrBuf[256];
void simProcessIns(cncInsUn_t *ins)
{
	int32_t temp = PLC::read(D2);
	if (temp >= 9) temp = 8;
	if (ins->type == insTypes::START_STITCH || ins->type == insTypes::STITCH_INS)
		armTimer(vfdSpeedIndexToTimeOut[temp], &nextStateEvt);
	else if (ins->type == insTypes::START_JUMP || ins->type == insTypes::JUMP_INS)
	{
		armTimer(ins->jumpIns.stepTime/72000, &nextStateEvt);
		printF("time:%d\n",ins->jumpIns.stepTime/72000);
	}
	else if (ins->type == insTypes::SET_VALVE_INS)
		armTimer(500, &nextStateEvt);


	getInsStr(*ins, insStrBuf);
	print(insStrBuf);
	print("---------\n");
	PLC::write(Y12, R_TOGGLE);
}


void getInsStr(jumpIns_t &ins, char *buf){
    sprintf(buf, "jump: %u %u\nxStep:%d\nyStep:%d\nstepTime:%u\n",
    	ins.id, ins.type,
		ins.xStep, ins.yStep,
		ins.stepTime);
}

void getInsStr(stitchIns_t &ins, char *buf){
    sprintf(buf, "stitch: %u %u\nxStep:%d\nyStep:%d\nstepTime:%u\nvfdSpeed:%c\n",
    	ins.id, ins.type,
		ins.xStep, ins.yStep,
		ins.stepTime, ins.vfdSpeed);
}
void getInsStr(zeroIns_t &ins, char *buf){
    sprintf(buf, "zero: %u %u\nxStep:%d\nyStep:%d\nstepTime:%u\nvfdSpeed:%c\n",
    	ins.id, ins.type,
		ins.xStep, ins.yStep,
		ins.stepTime, ins.vfdSpeed);
}
void getInsStr(setValveIns_t &ins, char *buf){
    sprintf(buf, "valve: %u %u\naction:%u\nvalve:%u\ndelay:%u\n",
    	ins.id, ins.type,
		ins.action, ins.valve,
		ins.delay);
}




void getInsStr(cncInsUn_t &ins, char *buf)
{
    switch (ins.cncIns.type)
    {
    case insTypes::START_STITCH:
    case insTypes::STITCH_INS:
        getInsStr(ins.stitchIns, buf);
        break;

    case insTypes::START_JUMP:
    case insTypes::JUMP_INS:
        getInsStr(ins.jumpIns, buf);
        break;

    case insTypes::ZERO_INS:
        getInsStr(ins.zeroIns, buf);
        break;

    case insTypes::SET_VALVE_INS:
        getInsStr(ins.valveIns, buf);
        break;

    default:
        break;
    }
}


void getInsStr(cncInsUn_t *ins, char *buf)
{
	getInsStr(*ins, buf);
}

}
























