/*
 * fifo.h
 *
 *  Created on: Feb 20, 2020
 *      Author: agarcia
 */

#ifndef FIFO_H_
#define FIFO_H_

/*
 * fifo.h
 *
 * Created: 23/07/2018 12:01:09
 *  Author: agarcia
 */

typedef struct
{
	uint16_t head;								// Head address
	uint16_t tail;								// Tail address
	uint16_t nregs;								// Maximum Log registers
	uint16_t dataLen;							// Register data length
	uint16_t npend;							    // N pending registers
	uint8_t *pbuffer;							// Pointer to data buffer
} FIFO_ctrl;

uint32_t GetFIFOElements(FIFO_ctrl *);
uint32_t FIFO_IsEmpty(FIFO_ctrl *);
uint32_t FIFO_IsFull(FIFO_ctrl* );
uint32_t GetFIFOFreeElements(FIFO_ctrl*);
void FIFO_Clear(FIFO_ctrl* p_fifo);
int32_t Pop_N_FIFO(FIFO_ctrl *, void *, uint32_t);
int32_t Push_N_FIFO(FIFO_ctrl *, void *, uint32_t);
int32_t Push_FIFO(FIFO_ctrl *, void * );
int32_t Pop_FIFO(FIFO_ctrl *, void *);
void InitFIFO(FIFO_ctrl *, void *, uint16_t , uint16_t);

#endif /* FIFO_H_ */
