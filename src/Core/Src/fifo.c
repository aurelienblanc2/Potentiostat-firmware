/*
 * fifo.c
 *
 *  Created on: Feb 20, 2020
 *      Author: agarcia
 */

#include <errno.h>
#include <string.h>
#include "user.h"
#include "fifo.h"

/* PUSH N elements to circular FIFO buffer */
int32_t Push_N_FIFO(FIFO_ctrl *p_fifo, void *elements, uint32_t count)
{
	static uint32_t inter_lock = 0;

	uint8_t *pbuf = p_fifo->pbuffer;
	uint8_t *p_elem = elements;
	int16_t next = p_fifo->head + count;
	uint32_t idx, tot_len;
	bool carry = false;

	/* Check if function re-enter when another push was running */

	if (inter_lock)
		return -1;

	inter_lock = 1;

	if (next >= p_fifo->nregs) {
		next -= p_fifo->nregs;
		carry = true;
	}

	p_fifo->npend += count;

	if (p_fifo->npend > p_fifo->nregs)
		p_fifo->npend = p_fifo->nregs;

	/* check if circular buffer is full */
	/* Push tail to next position, previous data will be overwritten */
	if ((p_fifo->head == p_fifo->tail) && (p_fifo->npend == p_fifo->nregs))
		p_fifo->tail = next;

	/* Locate head position on byte buffer */
	/* If buffer end is reached carry data to init buffer position */

	idx = (p_fifo->head * p_fifo->dataLen);
	if (carry) {
		/* Copy data from head to end of buffer */
		tot_len = ((p_fifo->nregs - p_fifo->head) * p_fifo->dataLen);
		memcpy(&pbuf[idx], p_elem, tot_len);

		/* Update pointer position */
		p_elem += tot_len;

		/* Copy data from init buffer */
		tot_len = next * p_fifo->dataLen;
		memcpy(pbuf, p_elem, tot_len);
	} else {
		tot_len = (count * p_fifo->dataLen);
		memcpy(&pbuf[idx], p_elem, tot_len);
	}

	/* Set head to next data offset. */
	p_fifo->head = next;

	/* Free lock */
	inter_lock = 0;

	return p_fifo->npend;
}

/* POP N elements from circular FIFO buffer */

int32_t Pop_N_FIFO(FIFO_ctrl *p_fifo, void *elements, uint32_t count)
{
	uint8_t *pbuf = p_fifo->pbuffer;
	uint8_t *p_elem = elements;
	int16_t next = p_fifo->tail;
	uint32_t idx, tot_len;
	bool carry = false;

	/* if the head isn't ahead of the tail, we don't have any characters
	 check if circular buffer is empty and return with an error */

	if ((p_fifo->head >= p_fifo->tail) && (!p_fifo->npend))
		return 0;

	if (count > p_fifo->nregs)
		count = p_fifo->nregs;

	next += count;

	if (next >= p_fifo->nregs) {
		next -= p_fifo->nregs;
		carry = true;
	}

	p_fifo->npend -= (p_fifo->npend > count) ? count : p_fifo->npend;

	idx = (p_fifo->tail * p_fifo->dataLen);
	if (carry) {
		tot_len = (p_fifo->nregs - p_fifo->tail) * p_fifo->dataLen;
		memcpy(p_elem, &pbuf[idx], tot_len);

		p_elem += tot_len;

		tot_len = (next * p_fifo->dataLen);
		memcpy(p_elem, pbuf, tot_len);
	} else {
		tot_len = count * p_fifo->dataLen;
		memcpy(p_elem, &pbuf[idx], tot_len);
	}

	p_fifo->tail = next;
	return count;
}

int32_t Push_FIFO(FIFO_ctrl *p_fifo, void *element)
{
	static uint32_t inter_lock = 0;
	/* next is where head will point to after this write. */

	uint8_t *pbuf = p_fifo->pbuffer;
	int16_t next = p_fifo->head + 1;
	uint32_t idx;

	/* Check if function re-enter when another push was running */

	if (inter_lock)
		return -1;

	inter_lock = 1;

	/* If head surpass number of buffer registers,
	 *  head is set to initial value */
	if (next >= p_fifo->nregs)
		next = 0;

	/* Increase number of pending registers and limit it maximum value */
	if (++p_fifo->npend > p_fifo->nregs)
		p_fifo->npend = p_fifo->nregs;

	/* check if circular buffer is full 
	 * For circular buffer set tail to next position,
	 * previous data will be over written*/

	if ((p_fifo->head == p_fifo->tail) && (p_fifo->npend == p_fifo->nregs))
		p_fifo->tail = next;

	idx = (p_fifo->head * p_fifo->dataLen);
	memcpy(&pbuf[idx], element, p_fifo->dataLen);

	/* Set head to next data offset. */
	p_fifo->head = next;

	/* Free lock */
	inter_lock = 0;

	return p_fifo->npend;
}

int32_t Pop_FIFO(FIFO_ctrl *p_fifo, void *element)
{
	uint8_t *pbuf = p_fifo->pbuffer;
	int16_t next = p_fifo->tail + 1;
	uint32_t idx;

	/* if the head isn't ahead of the tail, we don't have any element.
	 * Check if circular buffer is empty and return with an error */

	if ((p_fifo->head == p_fifo->tail) && (!p_fifo->npend))
		return -1;

	if (next >= p_fifo->nregs)
		next = 0;

	if (p_fifo->npend)
		p_fifo->npend--;

	idx = (p_fifo->tail * p_fifo->dataLen);
	memcpy(element, &pbuf[idx], p_fifo->dataLen);

	p_fifo->tail = next;
	return p_fifo->npend;
}

uint32_t GetFIFOElements(FIFO_ctrl *p_fifo)
{
	return p_fifo->npend;
}

uint32_t FIFO_IsEmpty(FIFO_ctrl *p_fifo)
{
	return (p_fifo->npend == 0);
}

uint32_t FIFO_IsFull(FIFO_ctrl *p_fifo)
{
	return (p_fifo->npend == p_fifo->nregs);
}

uint32_t GetFIFOFreeElements(FIFO_ctrl *p_fifo)
{
	return (p_fifo->nregs - p_fifo->npend);
}

void FIFO_Clear(FIFO_ctrl *p_fifo)
{
	p_fifo->npend = p_fifo->head = p_fifo->tail = 0;
}

/**********************************************************
 ** Name:     InitFIFO
 ** Function: Initialize FIFO queue struct
 ** Input:
 **		*w_fifo 	-> Pointer to FIFO_ctrl struct
 ** 	*fifo_buf	-> Pointer to FIFO data buffer
 ** 	fifo_len	-> Data buffer length in bytes
 ** 	data_len	-> Element data length in bytes
 ** 	Output:   	-> None.
 **********************************************************/

void InitFIFO(FIFO_ctrl *p_fifo, void *fifo_buf, uint16_t fifo_len,
		uint16_t data_len)
{
	p_fifo->head = p_fifo->tail = 0;
	p_fifo->pbuffer = (uint8_t*) fifo_buf;
	p_fifo->dataLen = data_len;
	p_fifo->nregs = (fifo_len / data_len);
	p_fifo->npend = 0;
}

