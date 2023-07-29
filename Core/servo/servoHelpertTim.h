/*
 * servoHelperTim.h
 *
 *  Created on: Sep 1, 2022
 *      Author: sanjay
 */

#ifndef SERVO_SERVOHELPERTTIM_H_
#define SERVO_SERVOHELPERTTIM_H_


typedef struct
{
	unsigned char type;
	uint32_t time;
}evBuf_t;



class servoHalper_t
{
public:
	void init();
	void resetHelper();
	void writeDeadTime(uint64_t time, unsigned char type);
	void writeEventTime(uint64_t time, unsigned char type);
	uint8_t (*isHelperPaused)();
	void (*writeBuffer)(servoHalper_t*, uint32_t time, unsigned char type);
	void (*trigger)(servoHalper_t*);
	void (*pause)(servoHalper_t*, uint8_t wait);

//private:

	TIM_TypeDef *helperTim;
	evBuf_t evTimeBuf[16];
	unsigned char evTimeBufHead, evTimeBufTail;
	uint64_t timeAcc, time_ref;

};


#endif /* SERVO_SERVOHELPERTTIM_H_ */
