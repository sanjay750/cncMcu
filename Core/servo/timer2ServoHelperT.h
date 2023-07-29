/*
 * timer2ServoHelper.h
 *
 *  Created on: Sep 1, 2022
 *      Author: sanjay
 */

#ifndef SERVO_TIMER2SERVOHELPERT_H_
#define SERVO_TIMER2SERVOHELPERT_H_

#include <servoHelpertTim.h>


extern servoHalper_t xyServoHelper;

void servoHelperInit();
void resetServoHelper();

void writeBuffer(servoHalper_t*, uint32_t time, unsigned char type);
void triggerHelper(servoHalper_t*);
void xyPause(servoHalper_t *halper, uint8_t wait);
uint8_t xyPaused();





#endif /* SERVO_TIMER2SERVOHELPERT_H_ */
