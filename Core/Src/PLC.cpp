/*
 * PLC.cpp
 *
 *  Created on: 15-May-2023
 *      Author: sanjay
 */






#include "main.h"
#include "string.h"
#include "stm32f3xx_hal_flash.h"


#include "serialComHandlerV2.h"
#include "PLC.h"

namespace PLC
{


uint32_t INPUT_REG;
uint64_t AUX_REG;
uint32_t OUTPUT_REG;

uint32_t GresInput;
uint64_t GresAux;
uint32_t GresOutput;


int iWord[16];
uint32_t word[16];
double fl[16];




uint8_t respBuffer[256];
uint8_t *plcComHandler(ioHandler::reqComHandler_t* com, const uint8_t *data, uint32_t len);

ioHandler::reqComHandler_t plcRDWR
{
	'P',
	plcComHandler
};

void plcComInit()
{
	ioHandler::addReqHandler(&plcRDWR);
}

uint8_t *plcComHandler(ioHandler::reqComHandler_t* com, const uint8_t *data, uint32_t len)
{
	uint32_t l = 0, bit, state;
	double fl_;
	uint32_t word_;
	uint64_t dword_;
	int iword_;
	word_t w;
	iWord_t iw;
	fWord_t fw;

	input_t PLC_X;
	output_t PLC_Y;
	auxRelay_t PLC_M;

	if (memcmp(data, "XRb", 3) == 0)
	{
		PLC_X = (input_t)data[3];
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'X';
		respBuffer[l++] = read(PLC_X) == R_CLOSE ? ('1') : ('0');

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}

	if (memcmp(data, "XRw", 3) == 0)
	{
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'X';
		readXREG(word_);
		packData<uint32_t>(word_, respBuffer, l);

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}



	else if (memcmp(data, "MRb", 3) == 0)
	{
		PLC_M = (auxRelay_t) (1LL << data[3]);
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'M';
		respBuffer[l++] = read(PLC_M) == R_CLOSE ? ('1') : ('0');

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}

	else if (memcmp(data, "MRw", 3) == 0)
	{
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'M';
		readMREG(dword_);
		packData<uint64_t>(dword_, respBuffer, l);

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}


	else if (memcmp(data, "MWb", 3) == 0)
	{
		PLC_M = (auxRelay_t) (1LL << data[3]);
		state = data[4];


		respBuffer[l++] = 'P';
		respBuffer[l++] = 'M';
		respBuffer[l++] = state;

		if (state == '0')
			write(PLC_M, R_OPEN);
		else if (state == '1')
			write(PLC_M, R_CLOSE);
		else if (state == 'T')
			write(PLC_M, R_TOGGLE);
		else
			respBuffer[l-1] = 'E';


		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}

	else if (memcmp(data, "YRb", 3) == 0)
	{
		PLC_Y = (output_t)data[3];
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'Y';
		respBuffer[l++] = read(PLC_Y) == R_CLOSE ? ('1') : ('0');

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}

	else if (memcmp(data, "YRw", 3) == 0)
	{
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'Y';
		readYREG(word_);
		packData<uint32_t>(word_, respBuffer, l);

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}

	else if (memcmp(data, "YWb", 3) == 0)
	{
		PLC_Y = (output_t)data[3];
		state = data[4];


		respBuffer[l++] = 'P';
		respBuffer[l++] = 'Y';
		respBuffer[l++] = state;

		if (state == '0')
			write(PLC_Y, R_OPEN);
		else if (state == '1')
			write(PLC_Y, R_CLOSE);
		else if (state == 'T')
			write(PLC_Y, R_TOGGLE);
		else
			respBuffer[l-1] = 'E';


		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}

	else if (memcmp(data, "IRw", 3) == 0)
	{
		iw = (iWord_t)data[3];

		l = 0;
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'I';
		packData<int>(read(iw), respBuffer, l);


		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}
	else if (memcmp(data, "IWw", 3) == 0)
	{
		l = 3;
		iw = (iWord_t)data[l++];
		iword_ = unPackData<int>(data, l);
		write(iw, iword_);

		l = 0;
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'I';
		packData<int>(iword_, respBuffer, l);


		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}


	else if (memcmp(data, "DRw", 3) == 0)
	{
		w = (word_t)data[3];

		l = 0;
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'D';
		packData<uint32_t>(read(w), respBuffer, l);

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}
	else if (memcmp(data, "DWw", 3) == 0)
	{
		l = 3;
		w = (word_t)data[l++];
		word_ = unPackData<uint32_t>(data, l);
		write(w, word_);

		l = 0;
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'D';
		packData<uint32_t>(word_, respBuffer, l);

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}

	else if (memcmp(data, "LRw", 3) == 0)
	{
		w = (word_t)data[3];

		l = 0;
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'L';
		packData<uint64_t>(readDW(w), respBuffer, l);

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}
	else if (memcmp(data, "LWw", 3) == 0)
	{
		l = 3;
		w = (word_t)data[l++];
		dword_ = unPackData<uint64_t>(data, l);
		writeDW(w, dword_);

		l = 0;
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'L';
		packData<uint64_t>(dword_, respBuffer, l);

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}



	else if (memcmp(data, "FRw", 3) == 0)
	{
		fw = (fWord_t)data[3];

		l = 0;
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'F';
		packData<double>(read(fw), respBuffer, l);

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}
	else if (memcmp(data, "FWw", 3) == 0)
	{
		l = 3;
		fw = (fWord_t)data[l++];
		fl_ = unPackData<double>(data, l);
		write(fw, fl_);

		l = 0;
		respBuffer[l++] = 'P';
		respBuffer[l++] = 'F';
		packData<double>(fl_, respBuffer, l);

		ioHandler::sendAckResp(com->packetSno, respBuffer, l);
	}



	return NULL;
}










}
