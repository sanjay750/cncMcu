/*
 * Y_servo.cpp
 *
 *  Created on: 02-Jul-2021
 *      Author: sanjay
 */

#include "main.h"
#include "cmsis_os.h"
#include "serialDebug.h"
#include "servo.h"
#include "Y_servo.h"
#include "cncStateMachine.h"
#include "qpc.h"

#define Y_SERVO_POSITIVE_CH	TIM_CHANNEL_2
#define Y_SERVO_NEGITIVE_CH	TIM_CHANNEL_4
#define Y_SERVO_DMAT_CH		TIM_CHANNEL_1
#define Y_SERVO_EVENT_CH	TIM_CHANNEL_3

#define Y_SERVO_POSITIVE_CCR	CCR2
#define Y_SERVO_NEGITIVE_CCR	CCR4
#define Y_SERVO_DMAT_CCR		CCR1
#define Y_SERVO_EVENT_CCR		CCR3



servo yServo;
DMA_HandleTypeDef y_servo_DMA;
TIM_HandleTypeDef y_servo_TIM;

void y_servo_buffer_wirte(servo *Servo, unsigned int time, unsigned char type);

void yServoInit()
{
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/**TIM4 GPIO Configuration
	PA12     ------> TIM4_CH2
	PB9     ------> TIM4_CH4
	*/


	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF10_TIM4;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	y_servo_TIM.Instance = TIM4;
	y_servo_DMA.Instance = DMA1_Channel1;
	y_servo_DMA.DmaBaseAddress = DMA1;
	y_servo_DMA.ChannelIndex = 1;

	yServo.servo_init(&y_servo_TIM, &y_servo_DMA, y_servo_buffer_wirte,
			Y_SERVO_DMAT_CH, Y_SERVO_POSITIVE_CH,
			Y_SERVO_NEGITIVE_CH, Y_SERVO_EVENT_CH);

	y_servo_TIM.Instance->CCER = TIM_CCER_CC2E | TIM_CCER_CC4E;
	y_servo_TIM.Instance->DIER = TIM_DIER_CC1DE | TIM_DIER_CC3IE;

	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);

}

void yServoReset()
{
	yServo.reset_timer();
	yServo.reset_DMA();

	y_servo_TIM.Instance->CCER = TIM_CCER_CC2E | TIM_CCER_CC4E;
	y_servo_TIM.Instance->DIER = TIM_DIER_CC1DE | TIM_DIER_CC3IE;

	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}


unsigned int ref_y_blink;
static osThreadId_t blockedThread;


void y_servo_buffer_wirte(servo *Servo, unsigned int time, unsigned char type)
{

	Servo->time_ref += time;
	servoBlock_t tp = (servoBlock_t)type;
	switch(type)
	{

	case positive_EP_ch:
	case positive_CB_ch:
	case positive_I_ch:
		Servo->edge = !Servo->edge;
		Servo->buffer[Servo->index].Y_SERVO_POSITIVE_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_NEGITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].Y_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_EVENT_CCR = Servo->time_ref - 1023;
		Servo->bufTypeQueue[Servo->index].type = Servo->edge ? tp : fallEdge_ch;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		Servo->intQueue[Servo->intQueueIndexHead++] = tp;
		break;


	case negitive_EP_ch:
	case negitive_CB_ch:
	case negitive_I_ch:
		Servo->edge = !Servo->edge;
		Servo->buffer[Servo->index].Y_SERVO_POSITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].Y_SERVO_NEGITIVE_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_EVENT_CCR = Servo->time_ref - 1023;
		Servo->bufTypeQueue[Servo->index].type = Servo->edge ? tp : fallEdge_ch;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		Servo->intQueue[Servo->intQueueIndexHead++] = tp;
		break;

	case event_EP_ch:
	case event_CB_ch:
	case event_ch:
		Servo->buffer[Servo->index].Y_SERVO_POSITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].Y_SERVO_NEGITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].Y_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_EVENT_CCR = Servo->time_ref - 1023;
		Servo->bufTypeQueue[Servo->index].type = tp;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		Servo->intQueue[Servo->intQueueIndexHead++] = type;

		break;

	case positive_ch:
		Servo->edge = !Servo->edge;
		Servo->buffer[Servo->index].Y_SERVO_POSITIVE_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_NEGITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].Y_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_EVENT_CCR = Servo->time_ref + 255;
		Servo->bufTypeQueue[Servo->index].type = (Servo->edge) ? tp : fallEdge_ch;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;

		break;

	case negitive_ch:
		Servo->edge = !Servo->edge;
		Servo->buffer[Servo->index].Y_SERVO_POSITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].Y_SERVO_NEGITIVE_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_EVENT_CCR = Servo->time_ref + 255;
		Servo->bufTypeQueue[Servo->index].type = (Servo->edge) ? tp : fallEdge_ch;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		break;


	case time_ch:
		Servo->buffer[Servo->index].Y_SERVO_POSITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].Y_SERVO_NEGITIVE_CCR = Servo->time_ref + 255;
		Servo->buffer[Servo->index].Y_SERVO_DMAT_CCR = Servo->time_ref;
		Servo->buffer[Servo->index].Y_SERVO_EVENT_CCR = Servo->time_ref + 255;
		Servo->bufTypeQueue[Servo->index].type = tp;
		Servo->bufTypeQueue[Servo->index].time = Servo->time_ref;
		break;
	}



	if (Servo->index == 127) Servo->buffer_status |= LOW_HALF_FULL;
	else if (Servo->index == 255) Servo->buffer_status |= HIGH_HALF_FULL;

	ref_y_blink = HAL_GetTick();

	GPIOC->BSRR = GPIO_PIN_9;

	while(Servo->buffer_status == FULL)
	{
		blockedThread = osThreadGetId();
		xTaskNotifyWait(0, UINT32_MAX, 0, portMAX_DELAY);
	}

	Servo->index++;
}

static QEvt signal;

extern "C" void TIM4_IRQHandler()
{


	void (*fptr)(int) = yServo.user_event_irq_call_back;

	if (yServo.intQueueIndexHead == yServo.intQueueIndexTail)
	{
		GPIOC->BRR = GPIO_PIN_9;
		TIM4->CR1 &= ~(TIM_CR1_CEN);
		signal.sig = Y_SERVO_INT_ERROR_SIG;

		__NVIC_SetPriority(TIM4_IRQn,0x0D);
		postEventISR(&signal);
		__NVIC_SetPriority(TIM4_IRQn,0x05);

		TIM4->SR &= ~TIM_SR_CC3IF;
	}
	else
	switch(yServo.intQueue[yServo.intQueueIndexTail++])
	{
	case positive_EP_ch:
		yServo.step_count++;
		__NVIC_SetPriority(TIM4_IRQn,0x0D);
//		signal = Y_SERVO_STEP;
//		osMessageQueuePut(yServo.signalQueue, &signal, 0, 0);
		__NVIC_SetPriority(TIM4_IRQn,0x05);
		break;

	case positive_CB_ch:
		yServo.step_count++;
		__NVIC_SetPriority(TIM4_IRQn,0x0D);
		if (fptr != 0) fptr(positive_CB_ch);
		__NVIC_SetPriority(TIM4_IRQn,0x05);
		break;

	case positive_I_ch:
		yServo.step_count++;
		break;

	case negitive_EP_ch:
		yServo.step_count--;
		__NVIC_SetPriority(TIM4_IRQn,0x0D);
//		signal = Y_SERVO_STEP;
//		osMessageQueuePut(yServo.signalQueue, &signal, 0, 0);
		__NVIC_SetPriority(TIM4_IRQn,0x05);
		break;

	case negitive_CB_ch:
		yServo.step_count--;
		__NVIC_SetPriority(TIM4_IRQn,0x0D);
		if (fptr != 0) fptr(negitive_CB_ch);
		__NVIC_SetPriority(TIM4_IRQn,0x05);
		break;

	case negitive_I_ch:
		yServo.step_count--;
		break;

	case event_EP_ch:
		GPIOC->BRR = GPIO_PIN_9;
		TIM4->CR1 &= ~TIM_CR1_CEN;
		TIM4->CNT = TIM4->Y_SERVO_DMAT_CCR - 255;
		__NVIC_SetPriority(TIM4_IRQn,0x0D);
		yServo.paused = 1;

		signal.sig = Y_SERVO_PAUSE_SIG;
		postEventISR(&signal);
		__NVIC_SetPriority(TIM4_IRQn,0x05);
		break;

	case event_CB_ch:
		GPIOC->BRR = GPIO_PIN_9;
		TIM4->CR1 &= ~TIM_CR1_CEN;
		TIM4->CNT = TIM4->Y_SERVO_DMAT_CCR - 255;
		yServo.paused = 1;
		__NVIC_SetPriority(TIM4_IRQn,0x0D);
		if (fptr != 0) fptr(event_CB_ch);
		__NVIC_SetPriority(TIM4_IRQn,0x05);
		break;

	case event_ch:
		GPIOC->BRR = GPIO_PIN_9;
		TIM4->CR1 &= ~TIM_CR1_CEN;
		TIM4->CNT = TIM4->Y_SERVO_DMAT_CCR - 255;
		yServo.paused = 1;
		break;
	}

	TIM4->SR &= ~TIM_SR_CC3IF;
}

extern "C" void DMA1_Channel1_IRQHandler()
{
	if (DMA1->ISR & DMA_ISR_HTIF1)
	{
		DMA1->IFCR = DMA_IFCR_CHTIF1;
		yServo.buffer_status &= ~LOW_HALF_FULL;
	}
	else if (DMA1->ISR & DMA_ISR_TCIF1)
	{
		DMA1->IFCR = DMA_IFCR_CTCIF1;
		yServo.buffer_status &= ~HIGH_HALF_FULL;
	}

	__NVIC_SetPriority(DMA1_Channel1_IRQn, 0xD);
	BaseType_t ptr = pdFALSE;
	if (blockedThread != 0)
	vTaskNotifyGiveFromISR(blockedThread, &ptr);
	__NVIC_SetPriority(DMA1_Channel1_IRQn, 0xC);
}


