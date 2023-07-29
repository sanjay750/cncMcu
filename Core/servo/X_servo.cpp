/*
 * X_servo.cpp
 *
 *  Created on: 02-Jul-2021
 *      Author: sanjay
 */


#include "main.h"
#include "cmsis_os.h"
#include "serialDebug.h"
#include "servo.h"
#include "X_servo.h"
#include "cncStateMachine.h"
#include "qpc.h"



#define X_SERVO_POSITIVE_CH	TIM_CHANNEL_1
#define X_SERVO_NEGITIVE_CH	TIM_CHANNEL_2
#define X_SERVO_DMAT_CH		TIM_CHANNEL_3
#define X_SERVO_EVENT_CH	TIM_CHANNEL_4

#define X_SERVO_POSITIVE_CCR	CCR1
#define X_SERVO_NEGITIVE_CCR	CCR2
#define X_SERVO_DMAT_CCR		CCR3
#define X_SERVO_EVENT_CCR		CCR4


servo xServo;
DMA_HandleTypeDef x_servo_DMA;
TIM_HandleTypeDef x_servo_TIM;





void x_servo_buffer_wirte(servo *Servo, unsigned int time, unsigned char type);

void xServoInit()
{
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*TIM3 GPIO Configuration
	PA4     ------> TIM3_CH2
	PA6     ------> TIM3_CH1*/

	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	x_servo_TIM.Instance = TIM3;
	x_servo_DMA.Instance = DMA1_Channel2;
	x_servo_DMA.DmaBaseAddress = DMA1;
	x_servo_DMA.ChannelIndex = 2;

	xServo.servo_init(&x_servo_TIM, &x_servo_DMA, x_servo_buffer_wirte,
			X_SERVO_DMAT_CH, X_SERVO_POSITIVE_CH,
			X_SERVO_NEGITIVE_CH, X_SERVO_EVENT_CH);

	x_servo_TIM.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	x_servo_TIM.Instance->DIER = TIM_DIER_CC3DE | TIM_DIER_CC4IE;


	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void xServoReset()
{
	xServo.reset_timer();
	xServo.reset_DMA();

	x_servo_TIM.Instance->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	x_servo_TIM.Instance->DIER = TIM_DIER_CC3DE | TIM_DIER_CC4IE;
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}


unsigned int ref_x_blink;
static osThreadId_t blockedThread;


void x_servo_buffer_wirte(servo *Servo, unsigned int time, unsigned char type)
{

	Servo->time_ref += time;


	servoBlock_t tp = (servoBlock_t)type;
	switch(type)
	{


	case positive_EP_ch:
	case positive_CB_ch:
	case positive_I_ch:
		Servo->edge = !Servo->edge;
		Servo->buffer[Servo->index].X_SERVO_POSITIVE_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_NEGITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].X_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_EVENT_CCR = Servo->time_ref - 1023;
		Servo->bufTypeQueue[Servo->index].type = Servo->edge ? tp : fallEdge_ch;
		Servo->intQueue[Servo->intQueueIndexHead++] = type;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		break;


	case negitive_EP_ch:
	case negitive_CB_ch:
	case negitive_I_ch:
		Servo->edge = !Servo->edge;
		Servo->buffer[Servo->index].X_SERVO_POSITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].X_SERVO_NEGITIVE_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_EVENT_CCR = Servo->time_ref - 1023;
		Servo->bufTypeQueue[Servo->index].type = Servo->edge ? tp : fallEdge_ch;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		Servo->intQueue[Servo->intQueueIndexHead++] = type;
		break;

	case event_EP_ch:
	case event_CB_ch:
	case event_ch:
		Servo->buffer[Servo->index].X_SERVO_POSITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].X_SERVO_NEGITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].X_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_EVENT_CCR = Servo->time_ref - 1023;
		Servo->bufTypeQueue[Servo->index].type = tp;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		Servo->intQueue[Servo->intQueueIndexHead++] = type;
		break;

	case positive_ch:
		Servo->edge = !Servo->edge;
		Servo->buffer[Servo->index].X_SERVO_POSITIVE_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_NEGITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].X_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_EVENT_CCR = Servo->time_ref + 255;
		Servo->bufTypeQueue[Servo->index].type = (Servo->edge) ? tp : fallEdge_ch;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		break;

	case negitive_ch:
		Servo->edge = !Servo->edge;
		Servo->buffer[Servo->index].X_SERVO_POSITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].X_SERVO_NEGITIVE_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_EVENT_CCR = Servo->time_ref + 255;
		Servo->bufTypeQueue[Servo->index].type = (Servo->edge) ? tp : fallEdge_ch;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		break;

	case time_ch:
		Servo->buffer[Servo->index].X_SERVO_POSITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].X_SERVO_NEGITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].X_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].X_SERVO_EVENT_CCR = Servo->time_ref + 255;
		Servo->bufTypeQueue[Servo->index].type = tp;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		break;
	}


	if (Servo->index == 127) Servo->buffer_status |= LOW_HALF_FULL;
	else if (Servo->index == 255) Servo->buffer_status |= HIGH_HALF_FULL;

	ref_x_blink = HAL_GetTick();

	GPIOB->BSRR = GPIO_PIN_10;

	while(Servo->buffer_status == FULL)
	{
		blockedThread = osThreadGetId();
		xTaskNotifyWait(0, UINT32_MAX, 0, portMAX_DELAY);
	}

	Servo->index++;
}

static QEvt signal;


extern "C" void TIM3_IRQHandler()
{

	void (*fptr)(int) = xServo.user_event_irq_call_back;

	if(xServo.intQueueIndexHead == xServo.intQueueIndexTail)
	{
		GPIOB->BRR = GPIO_PIN_10;
		TIM3->CR1 &= ~(TIM_CR1_CEN);
		signal.sig = X_SERVO_INT_ERROR_SIG;

		__NVIC_SetPriority(TIM3_IRQn,0x0D);
		postEventISR(&signal);
		__NVIC_SetPriority(TIM3_IRQn,0x0C);

		TIM3->SR &= ~TIM_SR_CC4IF;

	}
	else
	switch(xServo.intQueue[xServo.intQueueIndexTail++])
	{
	case positive_EP_ch:
		xServo.step_count++;
		__NVIC_SetPriority(TIM3_IRQn,0x0D);
//		signal = X_SERVO_STEP;
//		osMessageQueuePut(xServo.signalQueue, &signal, 0, 0);
		__NVIC_SetPriority(TIM3_IRQn,0x0C);
		break;

	case positive_CB_ch:
		xServo.step_count++;
		__NVIC_SetPriority(TIM3_IRQn,0x0D);
		if (fptr != 0) fptr(positive_CB_ch);
		__NVIC_SetPriority(TIM3_IRQn,0x0C);
		break;

	case positive_I_ch:
		xServo.step_count++;
		break;

	case negitive_EP_ch:
		xServo.step_count--;
		__NVIC_SetPriority(TIM3_IRQn,0x0D);
//		signal = X_SERVO_STEP;
//		osMessageQueuePut(xServo.signalQueue, &signal, 0, 0);
		__NVIC_SetPriority(TIM3_IRQn,0x0C);
		break;

	case negitive_CB_ch:
		xServo.step_count--;
		__NVIC_SetPriority(TIM3_IRQn,0x0D);
		if (fptr != 0) fptr(negitive_CB_ch);
		__NVIC_SetPriority(TIM3_IRQn,0x0C);
		break;

	case negitive_I_ch:
		xServo.step_count--;
		break;

	case event_EP_ch:
		GPIOB->BRR = GPIO_PIN_10;
		TIM3->CR1 &= ~TIM_CR1_CEN;
		TIM3->CNT = TIM3->X_SERVO_DMAT_CCR - 255;

		xServo.paused = 1;

		__NVIC_SetPriority(TIM3_IRQn,0x0D);
		signal.sig = X_SERVO_PAUSE_SIG;
		postEventISR(&signal);
		__NVIC_SetPriority(TIM3_IRQn,0x0C);
		break;

	case event_CB_ch:
		GPIOB->BRR = GPIO_PIN_10;
		TIM3->CR1 &= ~TIM_CR1_CEN;
		TIM3->CNT = TIM3->X_SERVO_DMAT_CCR - 255;
		xServo.paused = 1;

		__NVIC_SetPriority(TIM3_IRQn,0x0D);
		if (fptr != 0) fptr(event_CB_ch);
		__NVIC_SetPriority(TIM3_IRQn,0x0C);
		break;

	case event_ch:
		GPIOB->BRR = GPIO_PIN_10;
		TIM3->CR1 &= ~TIM_CR1_CEN;
		TIM3->CNT = TIM3->X_SERVO_DMAT_CCR - 255;
		xServo.paused = 1;
		break;
	}

	TIM3->SR &= ~TIM_SR_CC4IF;

}




extern "C" void DMA1_Channel2_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_HTIF2)
	{
		DMA1->IFCR = DMA_IFCR_CHTIF2;
		xServo.buffer_status &= ~LOW_HALF_FULL;
	}
	else if (DMA1->ISR & DMA_ISR_TCIF2)
	{
		DMA1->IFCR = DMA_IFCR_CTCIF2;
		xServo.buffer_status &= ~HIGH_HALF_FULL;
	}

	__NVIC_SetPriority(DMA1_Channel2_IRQn, 0xD);
	BaseType_t ptr = pdFALSE;
	if (blockedThread != 0)
	vTaskNotifyGiveFromISR(blockedThread, &ptr);
	__NVIC_SetPriority(DMA1_Channel2_IRQn, 0xC);

}

