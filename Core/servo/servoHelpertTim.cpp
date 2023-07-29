/*
 * servoHelperTim.cpp
 *
 *  Created on: Sep 1, 2022
 *      Author: sanjay
 */


#include "main.h"
#include "cmsis_os.h"

#include "servo.h"
#include "servoHelpertTim.h"


void servoHalper_t::resetHelper()
{
	timeAcc = 0;
	evTimeBufHead = 0;
	evTimeBufTail = 0;
}



void servoHalper_t::writeEventTime(uint64_t time, unsigned char type)
{
	timeAcc += time;

	if (timeAcc > 0xFFFFFFFF)
	{
		timeAcc -= 0xFFFFFFFF;

		if (timeAcc <= 0xFFFFFFFF)
		{
			if (timeAcc < 720000)
			{
				writeBuffer(this, timeAcc + 0xFFFFFFFF - 720000, time_ch);
				writeBuffer(this, 720000, type);
				timeAcc = 0;
				return;
			}
			else
			{
				writeBuffer(this, 0xFFFFFFFF, time_ch);
				writeBuffer(this, timeAcc, type);
				timeAcc = 0;
			}
			return;
		}
		writeBuffer(this, 0xFFFFFFFF, time_ch);
	}
	else
	{
		writeBuffer(this, timeAcc, type);
		timeAcc = 0;
	}
}
