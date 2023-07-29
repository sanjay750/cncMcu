/*
 * timer2ServoHelper.cpp
 *
 *  Created on: Sep 1, 2022
 *      Author: sanjay
 */


#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"
#include "servo.h"
#include "X_servo.h"
#include "Y_servo.h"

#include <servoHelpertTim.h>
#include <timer2ServoHelperT.h>
#include "cncStateMachine.h"
#include "qpc.h"

servoHalper_t xyServoHelper;

void servoHelperInit()
{

	__HAL_RCC_TIM2_CLK_ENABLE();
	xyServoHelper.writeBuffer = writeBuffer;
	xyServoHelper.trigger = triggerHelper;
	xyServoHelper.helperTim = TIM2;
	xyServoHelper.isHelperPaused = xyPaused;
	xyServoHelper.helperTim->CNT = -1;
	TIM2->DIER |= TIM_DIER_CC1IE;
	NVIC_EnableIRQ(TIM2_IRQn);

}

void resetServoHelper()
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->DIER &= ~TIM_DIER_CC1IE;
	TIM2->SR &= ~TIM2->SR;
	TIM2->CNT = 0;

	xyServoHelper.evTimeBufHead = 0;
	xyServoHelper.evTimeBufTail = 0;
	xyServoHelper.timeAcc = 0;
	xyServoHelper.time_ref = 0;

	TIM2->DIER |= TIM_DIER_CC1IE;

}

uint8_t xyPaused()
{
	return !(TIM2->CR1 & TIM_CR1_CEN);
}

void triggerHelper(servoHalper_t *halper)
{
	//for timer 3 and 4 synchronization

	TIM2->CR2 = (0b001 << TIM_CR2_MMS_Pos);
	TIM2->SMCR = TIM_SMCR_MSM | (0b110 << TIM_SMCR_SMS_Pos);


	TIM3->SMCR = (0B0101<<TIM_SMCR_SMS_Pos) | (0b001 << TIM_SMCR_TS_Pos);
	TIM4->SMCR = (0B0101<<TIM_SMCR_SMS_Pos) | (0b001 << TIM_SMCR_TS_Pos);


	xServo.servo_DMA->Instance->CCR |= DMA_CCR_EN;
	yServo.servo_DMA->Instance->CCR |= DMA_CCR_EN;

	xServo.servo_TIM->Instance->CR1 |= TIM_CR1_CEN;
	yServo.servo_TIM->Instance->CR1 |= TIM_CR1_CEN;

	TIM2->CCR1 = halper->evTimeBuf[halper->evTimeBufTail & 15].time;
	TIM2->DIER |= TIM_DIER_CC1IE;
	TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t refPrint__;

void writeBuffer(servoHalper_t *halper, uint32_t time, unsigned char type)
{
	if (time == 0)
		return;

	halper->time_ref += time;

	evBuf_t *buf = &halper->evTimeBuf[halper->evTimeBufHead & 15];
	buf->type = type;
	buf->time = halper->time_ref;

	halper->evTimeBufHead++;


	//printF("H = %d, T = %d\n", halper->evTimeBufHead, halper->evTimeBufTail);
	refPrint__ = HAL_GetTick();
	while(1)
	{

		if (HAL_GetTick() - refPrint__ >= 500)
		{
			//printF("H = %d, T = %d\n", halper->evTimeBufHead, halper->evTimeBufTail);
			refPrint__ += 500;
		}
		if ((halper->evTimeBufHead & 15) != (halper->evTimeBufTail & 15))
			break;
		else
		osThreadYield();
	}

}

static QEvt signal;


extern "C" void TIM2_IRQHandler()
{
	if ((TIM2->DIER & TIM_DIER_CC1IE) && (TIM2->SR & TIM_SR_CC1IF))
	{
		TIM2->SR &= ~TIM_SR_CC1IF;


		switch(xyServoHelper.evTimeBuf[xyServoHelper.evTimeBufTail & 15].type)
		{
		case time_ch:
			break;

		case send_ev:
			__NVIC_SetPriority(TIM2_IRQn,0x0D);
			signal.sig = HELPER_EVENT_SIG;
			postEventISR(&signal);
			__NVIC_SetPriority(TIM2_IRQn,0x0C);
			break;


		case event_EP_ch:
			//post event;
			TIM2->CR1 &= ~TIM_CR1_CEN;
			__NVIC_SetPriority(TIM2_IRQn,0x0D);
			signal.sig = HELPER_PAUSE_SIG;
			postEventISR(&signal);
			__NVIC_SetPriority(TIM2_IRQn,0x0C);
			break;
		}
//		TIM2->CR1 &= ~TIM_CR1_CEN;
		xyServoHelper.evTimeBufTail++;
		TIM2->CCR1 = xyServoHelper.evTimeBuf[xyServoHelper.evTimeBufTail & 15].time;
//		TIM2->CR1 |= TIM_CR1_CEN;

	}

	if ((TIM2->DIER & TIM_DIER_CC2IE) && (TIM2->SR & TIM_SR_CC2IF))
	{
		TIM2->CR1 &= ~TIM_CR1_CEN;
		TIM2->SR &= ~TIM_SR_CC2IF;
		TIM2->DIER &= ~TIM_DIER_CC2IE;
	}
}


