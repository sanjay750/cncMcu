/*
 * serialComHandlerV2.h
 *
 *  Created on: 09-May-2023
 *      Author: sanjay
 */

#ifndef SERIALCOMHANDLER_SERIALCOMHANDLERV2_H_
#define SERIALCOMHANDLER_SERIALCOMHANDLERV2_H_

#include "cmsis_os.h"
#include "queue.h"
#include "task.h"

extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart4;
namespace ioHandler
{


#define MCUID	"ABCD-01234"
#define MODEL	"QUILT-2500"


enum rxState_t
{
    RX_STATE_CHR,
    RX_STATE_CR,
};

enum packetIndx_t
{
	PACKET_LEN_INDEX = 0,
	PACKET_SNO_INDEX = 2,
	PACKET_ID_INDEX = 4,
    ACK_PACKET_ID = 5,
};

enum ackStatus_t
{
	IDLE,
	ACK_WAITING,
	ACK_DONE,
};

enum txState_t
{
	TX_WAIT_PACKET,
	TX_WAIT_DMA_TCOMP,
};


struct sendPacketReq_t
{
	uint16_t psno;
	uint8_t *packet;
	int len;
};


struct reqComHandler_t
{
	char ID;
	uint8_t * (*handler)(reqComHandler_t* com, const uint8_t *data, uint32_t len);
	uint16_t packetSno;
};


enum comHandlerStatus_t
{
	free_,
	wait_for_ack,
	got_ack,
	com_error,
};


struct comHandler_t
{
	char ID;
	osThreadId_t blockedThread;
	comHandlerStatus_t status;
	int respLen;
	uint16_t packetSno;
	uint8_t *respPayload;
	uint8_t *txPayload;
	void init(char ID, uint8_t *txPl, uint8_t *respPl);
	int sendReq(const uint8_t*req, int len, uint32_t timeout = 100, int retry = 0);
};




void serialComInit();


void serialComHandler();


void addReqHandler(reqComHandler_t *reqHandler);


void sendPacket(uint16_t psno, uint8_t *pack, int len);
void sendAckResp(uint16_t psno, const uint8_t* ack, int len);

extern "C" void rxHandler();
extern "C" void txHandler();


// privet functions

void packetHandler(const uint8_t *data, int len);
int readSeral();
}

template <typename T> T unPackData(const uint8_t* buf, uint32_t &index)
{
	T temp;
	memcpy(&temp, &buf[index], sizeof(T));
	index += sizeof(T);
	return temp;
}



template <typename T> uint32_t packData(T temp, uint8_t* buf, uint32_t &index)
{

	memcpy(&buf[index], &temp, sizeof(T));
	index += sizeof(T);
	return sizeof(temp);
}

inline uint32_t packStr(const char *str, uint8_t* buf, uint32_t &index)
{
	memcpy(&buf[index], str, strlen(str));
	index += strlen(str);
	return strlen(str);
}



#endif /* SERIALCOMHANDLER_SERIALCOMHANDLERV2_H_ */
