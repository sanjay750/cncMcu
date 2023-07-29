/*
 * mainMotor.cpp
 *
 *  Created on: 17-Jul-2023
 *      Author: sanjay
 */

#include "main.h"

#include "mainMotor.h"
#include "PLC.h"


// ---------------------------------------------------------------------------------------





uint16_t speedList[16] = {
0,
80,
160,
240,
320,
400,
480,
560,
640,
720,
800,
880,
960,
1040,
1120,
1200,

};




void vfdMainMotor::setSpeed(uint16_t speedRpm)
{
	uint8_t i;
	for (i = 0; i < 16; i++) {
		if (speedList[i] > speedRpm) {
			i--;
			break;
		}
		else if (speedList[i] == speedRpm) break;
	}

	if (i == 0) {
		stopMotor();
		speedIndex = 0;
	}
	else if (i < 16) {
		speedIndex = i;
		PLC::write(MEG_BREAK, R_OPEN);
		PLC::write(VFD_M1, (i & 1) ? R_CLOSE : R_OPEN);
		PLC::write(VFD_M2, (i & 2) ? R_CLOSE : R_OPEN);
		PLC::write(VFD_M3, (i & 4) ? R_CLOSE : R_OPEN);
		PLC::write(VFD_M4, (i & 8) ? R_CLOSE : R_OPEN);
		PLC::write(VFD_RUN, R_CLOSE);
	}
}

void vfdMainMotor::stopMotor()
{
	PLC::write(VFD_M1, R_OPEN);
	PLC::write(VFD_M2, R_OPEN);
	PLC::write(VFD_M3, R_OPEN);
	PLC::write(VFD_M4, R_OPEN);
	PLC::write(VFD_RUN, R_OPEN);
	PLC::write(MEG_BREAK, R_CLOSE);
}


void vfdMainMotor::setBreak(bool set)
{
	PLC::write(MEG_BREAK,(set)?(R_CLOSE):(R_OPEN));
}

void vfdMainMotor::resetBreak()
{
	PLC::write(MEG_BREAK, R_OPEN);
}

void vfdMainMotor::toggleBreak()
{
	PLC::write(MEG_BREAK, R_TOGGLE);
}

uint16_t vfdMainMotor::getSpeed()
{
	return speedList[speedIndex];
}



void vfdMainMotor::init(
	void (*needleTopCallBack)(),
	void (*needleDownCallBack)(),
	void (*needleClothOutCallBack)()
	)
{
	this->needleTopCallBack = needleTopCallBack;
	this->needleDownCallBack = needleDownCallBack;
	this->needleClothOutCallBack = needleClothOutCallBack;
}

void vfdMainMotor::dinit(){

}









