/*
 * servo.cpp
 *
 *  Created on: Jun 16, 2021
 *      Author: sanjay
 */


#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "main.h"
#include "serialDebug.h"
#include "servo.h"






void servo::servo_init(TIM_HandleTypeDef* servo_TIM, DMA_HandleTypeDef* servo_DMA,
					void (*write_buff)(servo*, unsigned int, unsigned char),
					unsigned char DMA_tim_ch, unsigned char positive_tim_ch,
					unsigned char negitive_tim_ch, unsigned char event_tim_ch)
{

	this->write_buffer = write_buff;
	this->DMA_tim_ch = DMA_tim_ch;
	this->positive_tim_ch = positive_tim_ch;
	this->negitive_tim_ch = negitive_tim_ch;
	this->event_tim_ch = event_tim_ch;

	this->servo_TIM = servo_TIM;
	this->servo_DMA = servo_DMA;

	//servo TIM configuration
	servo_TIM->Init.Prescaler = 0;
	servo_TIM->Init.CounterMode = TIM_COUNTERMODE_UP;
	servo_TIM->Init.Period = 0xFFFF;
	servo_TIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	servo_TIM->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	TIM_OC_InitTypeDef output_com_config = {0};

	//ch configuration
	output_com_config.OCMode = TIM_OCMODE_TOGGLE;
	output_com_config.Pulse = 0xFF + 1;
	output_com_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	output_com_config.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, positive_tim_ch);
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, negitive_tim_ch);
	output_com_config.OCMode = TIM_OCMODE_TIMING;
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, event_tim_ch);
	output_com_config.Pulse = 1;
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, DMA_tim_ch);

	if(HAL_TIM_Base_Init(servo_TIM))
		printf("servo timer init failed\n");

	servo_DMA->Init.Mode = DMA_CIRCULAR;
	servo_DMA->Init.Direction = DMA_MEMORY_TO_PERIPH;
	servo_DMA->Init.PeriphInc = DMA_PINC_DISABLE;
	servo_DMA->Init.MemInc = DMA_MINC_ENABLE;
	servo_DMA->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	servo_DMA->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	servo_DMA->Init.Priority = DMA_PRIORITY_VERY_HIGH;


	if (HAL_DMA_Init(servo_DMA))
		printf("servo DMA init failed\n");
	else
	{
		HAL_DMA_Start(servo_DMA, (uint32_t) &buffer[0], (uint32_t) &servo_TIM->Instance->DMAR, 256*4);
		servo_DMA->Instance->CCR &= ~DMA_CCR_EN;
		servo_DMA->Instance->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE;
	}


	servo_TIM->Instance->DCR = (13 << TIM_DCR_DBA_Pos) | (3 << TIM_DCR_DBL_Pos);
}


void servo_dinit()
{

}



void servo::write_step(unsigned int time, unsigned char type)
{
	unsigned char typef = type;
	if(type == positive_EP_ch || type == positive_I_ch)	typef = positive_ch;
	else if (type == negitive_EP_ch || type == negitive_I_ch) typef = negitive_ch;

	time_acc += time;

	//printf("----------\n");

	if (time_acc > 0xFF00)
	{
		while(1)
		{
			time_acc -= 0xFF00;

			if (time_acc <= 0xFF00)
			{
				if (time_acc < 2*1023)
				{
					write_buffer(this, time_acc + 0xFF00 - 2*1023, time_ch);
					write_buffer(this, 2*1023, type);
					write_buffer(this, setp_pulse_width, typef);

					time_acc = -setp_pulse_width;
					return;
				}
				else
				{
					write_buffer(this, 0xFF00, time_ch);
					write_buffer(this, time_acc, type);
					write_buffer(this, setp_pulse_width, typef);
					time_acc = -setp_pulse_width;
				}
				return;
			}
			write_buffer(this, 0xFF00, time_ch);
		}
	}
	else
	{
		write_buffer(this, time_acc, type);
		write_buffer(this, setp_pulse_width, typef);
		time_acc = -setp_pulse_width;
	}
}


void servo::write_dead_time(unsigned int time)
{
	if (time == 0)
		return;

	time_acc += time;

	while (time_acc >= 0xFF00*3)
	{
		time_acc -= 0xFF00;
		write_buffer(this, 0xFF00, time_ch);
	}
}

void servo::write_pause_time(unsigned int time, unsigned char type)
{

	if (type != event_ch && type != event_EP_ch) type = event_ch;

	time_acc += time;

	//printf("----------\n");

	if (time_acc > 0xFF00)
	{
		while(1)
		{
			time_acc -= 0xFF00;

			if (time_acc <= 0xFF00)
			{
				if (time_acc < 2*1023)
				{
					write_buffer(this, time_acc + 0xFF00 - 2*1023, time_ch);
					write_buffer(this, 2*1023, type);
					time_acc = 0;
					return;
				}
				else
				{
					write_buffer(this, 0xFF00, time_ch);
					write_buffer(this, time_acc, type);
					time_acc = 0;
				}
				return;
			}
			write_buffer(this, 0xFF00, time_ch);
		}
	}
	else
	{
		write_buffer(this, time_acc, type);
		time_acc = 0;
	}
}

unsigned short int get_tim_ccr(TIM_CHANNEL *tim, unsigned char tch)
{
	switch(tch)
	{
	case TIM_CHANNEL_1:
		return tim->CCR1;
	case TIM_CHANNEL_2:
		return tim->CCR2;
	case TIM_CHANNEL_3:
		return tim->CCR3;
	case TIM_CHANNEL_4:
		return tim->CCR4;
	}
	return 0;
}

void servo::dumpDma(unsigned int start, unsigned int len)
{
	TIM_CHANNEL *ptr = 0;

	for (unsigned int i = start; i < start + len; i++)
	{
		ptr = &buffer[i];
//		GLOG.print("%u - PC = %u, NC= %u, DC = %u, EC = %u\n", i, get_tim_ccr(ptr, positive_tim_ch),
//				get_tim_ccr(ptr, negitive_tim_ch),
//				get_tim_ccr(ptr, DMA_tim_ch),
//				get_tim_ccr(ptr, event_tim_ch));
	}

}

unsigned char servo::isServoPaused()
{
	return !(servo_TIM->Instance->CR1 & TIM_CR1_CEN);
}


void servo::trigger_timre(unsigned short int EP)
{
	paused = 0;
	servo_DMA->Instance->CCR |= DMA_CCR_EN;
	servo_TIM->Instance->CR1 |= TIM_CR1_CEN;

	if (EP)
	{
		unsigned short int signal = EP;
		osMessageQueuePut(signalQueue, &signal, 0, 0);
	}

}


unsigned int servo::get_step_count()
{
	return step_count;
}

void servo::reset_timer()
{
	//servo TIM configuration
	servo_TIM->Instance->CR1 &= TIM_CR1_CEN;
	servo_TIM->Init.Prescaler = 0;
	servo_TIM->Init.CounterMode = TIM_COUNTERMODE_UP;
	servo_TIM->Init.Period = 0xFFFF;
	servo_TIM->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	servo_TIM->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	TIM_OC_InitTypeDef output_com_config = {0};

	//ch configuration
	output_com_config.OCMode = TIM_OCMODE_FORCED_INACTIVE;
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, positive_tim_ch);
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, negitive_tim_ch);


	output_com_config.OCMode = TIM_OCMODE_TOGGLE;
	output_com_config.Pulse = 0xFF + 1;
	output_com_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	output_com_config.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, positive_tim_ch);
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, negitive_tim_ch);

	output_com_config.OCMode = TIM_OCMODE_TIMING;
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, event_tim_ch);
	output_com_config.Pulse = 1;
	HAL_TIM_OC_ConfigChannel(servo_TIM, &output_com_config, DMA_tim_ch);

	servo_TIM->Instance->DCR = 0;
	servo_TIM->Instance->DCR = 0;
	servo_TIM->Instance->DCR = 0;

	servo_TIM->Instance->DCR = (13 << TIM_DCR_DBA_Pos) | (3 << TIM_DCR_DBL_Pos);

	if(HAL_TIM_Base_Init(servo_TIM))
		printf("servo timer init failed\n");

	time_ref = 0;
}


void servo::reset_DMA()
{
	servo_DMA->Instance->CCR &= ~DMA_CCR_EN;

	servo_DMA->Instance->CPAR = (uint32_t)&servo_TIM->Instance->DMAR;
	servo_DMA->Instance->CMAR = (uint32_t)buffer;
	servo_DMA->Instance->CNDTR = 256 * 4;
	servo_DMA->Instance->CCR &= ~DMA_CCR_EN;
	servo_DMA->Instance->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE;

	servo_DMA->DmaBaseAddress->IFCR = DMA_IFCR_CTCIF2;
	servo_DMA->DmaBaseAddress->IFCR = DMA_IFCR_CTCIF2;

	time_acc = 0;
	index = 0;
	buffer_status = 0;
	intQueueIndexHead = 0;
	intQueueIndexTail = 0;

}


void servo::reset_servo()
{
	reset_timer();
	reset_DMA();

}

