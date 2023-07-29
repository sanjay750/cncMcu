/*
 * servo.h
 *
 *  Created on: Jun 16, 2021
 *      Author: sanjay
 */



#ifndef SERVO_H_
#define SERVO_H_

#define EMPTY				0
#define LOW_HALF_FULL		1
#define HIGH_HALF_FULL		2
#define FULL				3

#define setp_pulse_width	3600


#include "cmsis_os.h"


struct servoSteps
{
	int steps;
	unsigned int stepPeriod;
};

struct TIM_CHANNEL
{
	unsigned short int CCR1;
	unsigned short int CCR2;
	unsigned short int CCR3;
	unsigned short int CCR4;
};




typedef enum
{
	positive_I_ch = 0,
	positive_EP_ch,
	positive_CB_ch,
	positive_ch,

	negitive_I_ch,
	negitive_EP_ch,
	negitive_CB_ch,
	negitive_ch,


	time_ch,
	send_ev,
	event_EP_ch,
	event_CB_ch,
	event_ch,

	fallEdge_ch,


}servoBlock_t;

struct TIM_stamp
{
	servoBlock_t type;
	unsigned int time;
};


#define servoPositiveSig		&servoSIG[0]
#define servoNegitiveSig		&servoSIG[1]
#define servoPauseSig			&servoSIG[2]
#define servoTriggerSig			&servoSIG[3]

class servo
{


public:


	void servo_init(TIM_HandleTypeDef* servo_TIM, DMA_HandleTypeDef* servo_DMA,
			void (*write_buff)(servo*, unsigned int, unsigned char),
			unsigned char DMA_tim_ch, unsigned char positive_tim_ch,
			unsigned char negitive_tim_ch, unsigned char event_tim_ch);

	void servo_dinit();

	void reset_timer();
	void reset_DMA();
	void reset_servo();


	void (*write_buffer)(servo *Servo, unsigned int time, unsigned char ch);
	void write_step(unsigned int time, unsigned char type);
	void write_dead_time(unsigned int time);
	void write_pause_time(unsigned int time, unsigned char type = event_ch);

	void dumpDma(unsigned int start, unsigned int len);

	void trigger_timre(unsigned short int ev);

	unsigned int get_step_count();
	unsigned char isServoPaused();

	void (*user_event_irq_call_back)(int ev);

	TIM_HandleTypeDef* servo_TIM;
	DMA_HandleTypeDef* servo_DMA;


	int step_count;


	unsigned char paused;
	unsigned char edge;
//protected:

	TIM_stamp bufTypeQueue[256];

	unsigned char intQueue[256];
	unsigned char intQueueIndexHead; //--
	unsigned char intQueueIndexTail; //--

	TIM_CHANNEL buffer[256];
	unsigned char index; //--
	unsigned char buffer_status; //--


	unsigned int time_ref; //--
	int time_acc; //--

	unsigned char DMA_tim_ch;
	unsigned char positive_tim_ch;
	unsigned char negitive_tim_ch;
	unsigned char event_tim_ch;

	osMessageQueueId_t signalQueue;


};

#endif
