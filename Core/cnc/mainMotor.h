/*
 * mainMotor.h
 *
 *  Created on: 17-Jul-2023
 *      Author: sanjay
 */

#ifndef CNC_MAINMOTOR_H_
#define CNC_MAINMOTOR_H_

#include "main.h"


struct mainMotor_t
{
protected:


	void (*needleTopCallBack)();
	void (*needleDownCallBack)();
	void (*needleClothOutCallBack)();
public:

	virtual void init(
	void (*needleTopCallBack)(),
	void (*needleDownCallBack)(),
	void (*needleClothOutCallBack)()
			);

	virtual void dinit();
	virtual void setSpeed(uint16_t speed);
	virtual void stopMotor();
	virtual void setBreak(bool set);
	virtual void resetBreak();
	virtual void toggleBreak();
	virtual uint16_t getSpeed();

};

struct vfdMainMotor:public mainMotor_t
{

protected:

	uint16_t speedIndex;
public:

	virtual void init(
	void (*needleTopCallBack)(),
	void (*needleDownCallBack)(),
	void (*needleClothOutCallBack)()
			);

	virtual void dinit();
	virtual void setSpeed(uint16_t speed);
	virtual void stopMotor();
	virtual void setBreak(bool set);
	virtual void resetBreak();
	virtual void toggleBreak();
	virtual uint16_t getSpeed();


};

#endif /* CNC_MAINMOTOR_H_ */
