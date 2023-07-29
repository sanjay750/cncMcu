/*
 * quiltSim.h
 *
 *  Created on: Jul 4, 2023
 *      Author: sanjay
 */

#ifndef CNC_QUILTSIM_H_
#define CNC_QUILTSIM_H_

#include "cnc.h"
namespace QUILT_SIM
{


void checkForStartSig();

extern "C" int simReadQuiltSig();
extern "C" void simUnlat();
extern "C" void simProcessIns(cncInsUn_t *ins);

void getInsStr(jumpIns_t &ins, char *buf);
void getInsStr(stitchIns_t &ins, char *buf);
void getInsStr(zeroIns_t &ins, char *buf);
void getInsStr(setValveIns_t &ins, char *buf);
void getInsStr(cncInsUn_t &ins, char *buf);



};



#endif /* CNC_QUILTSIM_H_ */
