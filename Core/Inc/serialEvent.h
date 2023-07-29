/*
 * serialEvent.h
 *
 *  Created on: Aug 25, 2022
 *      Author: sanjay
 */

#ifndef INC_SERIALEVENT_H_
#define INC_SERIALEVENT_H_

enum userEventType
{
	USER_EVT_ENABLE = '~',
	X_STEP_EVT = 'x',
	Y_STEP_EVT = 'y',
	MOVE_SERVO_COM_EVT = 'S',
	NEEDLE_TOP_EVT = 't',
	NEEDLE_POS_EVT = 'p',
	TOGGLE_VALVE_EVT = 'v',
	TOGGLE_BREAK_EVT = 'B',
	VFD_SPEED_EVT = 'V',
	VFD_STOP_EVT = ' ',

	UNLATCH_START_EVT = 's',

	NEEDLE_UP_POS_COM = 'N',
	NEEDLE_LOOP_COM = 'R',
	NEEDLE_DOWN_POS_COM = 'n',

	TEST_JUMP_EVT = 'J',
	TSTEVT = 'S',

};

struct userEvent_t
{
	unsigned char type;
	int value;
	int value1;
	int value2;
};

void userEventHandlerInit();

extern "C" void userEventHandler();
unsigned char userEventAvailable();
unsigned char getUserEvent(userEvent_t *ev);



#endif /* INC_SERIALEVENT_H_ */
