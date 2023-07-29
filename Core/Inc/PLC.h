/*
 * PLC.h
 *
 *  Created on: 15-May-2023
 *      Author: sanjay
 */

#ifndef INC_PLC___H_
#define INC_PLC___H_


#include "main.h"
#include "stm32f3xx.h"
#include "PLC_DEF.h"

#include "cmsis_os.h"
#include "task.h"

#ifdef __cplusplus

typedef enum
{
	R_OPEN,
	R_CLOSE,
	R_TOGGLE,
	R_ERR,
}IO_state_t;

typedef enum
{
	NO_READ,
	NC_READ,
	RISE_READ,
	FALL_READ,
	CHANGE_READ,
}contType_t;

namespace PLC
{


struct X_INPUT_t
{
	input_t X;
	GPIO_TypeDef * gpio;
	uint16_t pin;
};

struct Y_OUTPUT_t
{
	output_t Y;
	GPIO_TypeDef * gpio;
	uint16_t pin;
};

extern const X_INPUT_t x_gpio[16];
extern const Y_OUTPUT_t y_gpio[16];

extern uint32_t INPUT_REG;
extern uint64_t AUX_REG;
extern uint32_t OUTPUT_REG;

extern uint32_t GresInput;
extern uint64_t GresAux;
extern uint32_t GresOutput;

extern int iWord[16];
extern uint32_t word[16];
extern double fl[16];







inline IO_state_t read(const input_t in, contType_t cont = NO_READ)
{

	if (x_gpio[in].X == X_NOPE) return R_ERR;

	const uint32_t bit = 1 << in;

	GPIO_PinState st = (x_gpio[in].gpio->IDR & x_gpio[in].pin) ? (GPIO_PIN_SET) : (GPIO_PIN_RESET);
	GPIO_PinState tSt = (GresInput & bit)?(GPIO_PIN_SET):(GPIO_PIN_RESET);
	IO_state_t tmp = R_OPEN;

	switch (cont)
	{
	case NO_READ:
		tmp = (st)?(R_CLOSE):(R_OPEN);
		break;


	case NC_READ:
		tmp = (st)?(R_OPEN):(R_CLOSE);
		break;

	case CHANGE_READ:
	{
		if (tSt != st)
		{
			tmp = R_CLOSE;
			GresInput ^= bit;
		}

		break;
	}

	case RISE_READ:
		if (tSt != st)
		{
			tmp = (st)?(R_CLOSE):(R_OPEN);
			GresInput ^= bit;
		}
		break;

	case FALL_READ:
		if (tSt != st)
		{
			tmp = (st)?(R_CLOSE):(R_OPEN);
			GresInput ^= bit;
		}
		break;
	}

	return tmp;
}

inline void readXREG(uint32_t &reg)
{
	reg = 0;
	int X = X0;
	uint32_t bit = 1;
	for (uint8_t i = 0; i < 16; i++, X++, bit <<= 1)
	{
		if (read((input_t) X) == R_CLOSE)
		{
			reg |= bit;
		}
	}
}


inline IO_state_t read(const output_t out, contType_t cont = NO_READ)
{

	if (y_gpio[out].Y == Y_NOPE) return R_ERR;

	const uint32_t bit = 1 << out;

	GPIO_PinState st = (y_gpio[out].gpio->IDR & y_gpio[out].pin) ? (GPIO_PIN_SET) : (GPIO_PIN_RESET);
	GPIO_PinState tSt = (GresOutput & bit)?(GPIO_PIN_SET):(GPIO_PIN_RESET);

	IO_state_t tmp = R_OPEN;


	switch (cont)
	{
	case NO_READ:
		tmp = (st)?(R_CLOSE):(R_OPEN);
		break;


	case NC_READ:
		tmp = (st)?(R_OPEN):(R_CLOSE);
		break;

	case CHANGE_READ:
	{
		if (tSt != st)
		{
			tmp = R_CLOSE;
			GresOutput ^= bit;
		}

		break;
	}

	case RISE_READ:
		if (tSt != st)
		{
			tmp = (st)?(R_CLOSE):(R_OPEN);
			GresOutput ^= bit;
		}
		break;

	case FALL_READ:
		if (tSt != st)
		{
			tmp = (st)?(R_CLOSE):(R_OPEN);
			GresOutput ^= bit;
		}
		break;
	}

	return tmp;
}

inline void readYREG(uint32_t &reg)
{
	reg = 0;
	int Y = X0;
	uint32_t bit = 1;
	for (uint8_t i = 0; i < 16; i++, Y++, bit <<= 1)
	{
		if (read((output_t) Y) == R_CLOSE)
		{
			reg |= bit;
		}
	}
}



inline void write(const output_t rel, IO_state_t state)
{
	if (y_gpio[rel].Y == Y_NOPE) return;

	if (state == R_OPEN) y_gpio[rel].gpio->BRR = y_gpio[rel].pin, OUTPUT_REG &= ~(1UL<<rel);
	else if (state == R_CLOSE) y_gpio[rel].gpio->BSRR = y_gpio[rel].pin, OUTPUT_REG |= (1UL<<rel);
	else if (state == R_TOGGLE) y_gpio[rel].gpio->ODR ^= y_gpio[rel].pin, OUTPUT_REG ^= (1UL<<rel);
}



inline IO_state_t read(const auxRelay_t in, contType_t cont = NO_READ)
{
	uint32_t tIn = AUX_REG, trIn = GresAux;
	IO_state_t tmp = R_OPEN;
	switch (cont)
	{
	case NO_READ:
		tmp = (tIn & in)?(R_CLOSE):(R_OPEN);
		break;


	case NC_READ:
		tmp = (tIn & in)?(R_OPEN):(R_CLOSE);
		break;

	case CHANGE_READ:
	{
		if ((tIn & in) != (trIn & in))
		{
			tmp = R_CLOSE;
			GresAux ^= in;
		}

		break;
	}

	case RISE_READ:
		if ((tIn & in) != (trIn & in))
		{
			tmp = (tIn & in)?(R_CLOSE):(R_OPEN);
			GresAux ^= in;
		}
		break;

	case FALL_READ:
		if ((tIn & in) != (trIn & in))
		{
			tmp = (tIn & in)?(R_OPEN):(R_CLOSE);
			GresAux ^= in;
		}
		break;
	}

	return tmp;
}

inline void readMREG(uint64_t &reg)
{
	reg = AUX_REG;
}

inline void write(const auxRelay_t rel, IO_state_t state)
{

	switch (state)
	{
	case R_CLOSE:
		AUX_REG |= rel;
		break;

	case R_OPEN:
		AUX_REG &= ~rel;
		break;

	case R_TOGGLE:
		AUX_REG ^= rel;
		break;

	case R_ERR:
		break;
	}
}




inline void write(const iWord_t i, int val)
{
	iWord[i] = val;
}

inline int read(const iWord_t i)
{
	return iWord[i];
}

inline void write(const word_t w, uint32_t val)
{
	word[w] = val;
}

inline uint32_t read(word_t w)
{
	return word[w];
}

union wdw_t
{
	uint32_t w[2];
	uint64_t dw;
};

inline void writeDW(const word_t w, uint64_t val)
{
	wdw_t DW;
	DW.dw = val;

	taskENTER_CRITICAL();
	write(w, DW.w[0]);
	write((word_t)(w+1), DW.w[1]);
	taskEXIT_CRITICAL();

}

inline uint64_t readDW(const word_t w)
{
	wdw_t DW;
	taskENTER_CRITICAL();
	DW.w[0] = read(w);
	DW.w[1] = read((word_t)(w+1));
	taskEXIT_CRITICAL();
	return DW.dw;
}

inline void write(const fWord_t w, double val)
{
	fl[w] = val;
}

inline double read(const fWord_t w)
{
	return fl[w];
}


void plcComInit();





}

#endif


#endif /* INC_PLC_H_ */
