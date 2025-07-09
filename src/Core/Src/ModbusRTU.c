/*
 * ModbusRTU.c
 *
 * Created: 08/02/2016 8:56:06
 *  Author: agarcia
 */

#include <string.h>
#include <errno.h>
#include "user.h"

uint32_t restore_dflt_tm = 0;
stMbusReadRSP stReadRSP;
stMbusWriteRSP stWRSP;
uint8_t MB_exception[2 * AES_BCKLEN];

uint32_t Error_cnt[N_MBUS_PORTS];
uint32_t lastCom_tm[N_MBUS_PORTS];

const char *str_success = { "SUCCESS. \r\n" };
const char *str_fail = { "FAIL.\r\n" };

// Modbus functions implementation

stMbusReadRSP* GetMbReadRsp(void)
{
	return &stReadRSP;
}

stMbusWriteRSP* GetMbWriteRsp(void)
{
	return &stWRSP;
}

inline uint16_t GetModBusReg(uint8_t *bytes)
{
	return (uint16_t) ((*bytes << 8) | *(bytes + 1));
}

uint16_t fReadHldRegRSP(uint8_t *pSt, uint16_t start, uint16_t max_legth,
		stMbusReadRSP *pRSP, uint8_t port)
{
	int16_t x = start, y = 0;

	if ((pRSP->nBytes > MB_MAX_DATALENGTH * 2)
			|| ((start + pRSP->nBytes) > max_legth))
		return ILLEGAL_DATA_ADDRESS;

	pRSP->Data[y++] = pRSP->SlaveID;
	pRSP->Data[y++] = pRSP->Function;
	pRSP->Data[y++] = pRSP->nBytes;

	for (uint32_t i = 0; i < pRSP->nBytes; i += 2, x += 2, y += 2) {
		pRSP->Data[y] = *(pSt + x + 1);
		pRSP->Data[y + 1] = *(pSt + x);
	};

	x = CRC16_compute(pRSP->Data, y);

	pRSP->Data[y++] = (uint8_t) x;
	pRSP->Data[y++] = (uint8_t) (x >> 8);

	pRSP->nBytes = y;

	return NO_ERROR;
}

uint16_t fWriteHldRegRSP(uint8_t *pSt, uint8_t *pData, uint16_t max_legth,
		stMbusWriteRSP *pstWRSP, uint8_t port)
{
	uint16_t sRegister, nBytes;
	uint32_t y = 0;

	nBytes = 2 * pstWRSP->nRegisters;
	sRegister = 2 * (pstWRSP->Start & 0x00ff);

	pstWRSP->Start = swap_u16(pstWRSP->Start);
	pstWRSP->nRegisters = swap_u16(pstWRSP->nRegisters);
	pstWRSP->Data[y++] = pstWRSP->SlaveID;

	memcpy(&pstWRSP->Data[y], &pstWRSP->Function, 5);
	y += 5;
	pstWRSP->m_crc = CRC16_compute(pstWRSP->Data, y);

	pstWRSP->Data[y++] = (uint8_t) pstWRSP->m_crc;
	pstWRSP->Data[y++] = (uint8_t) (pstWRSP->m_crc >> 8);

	pstWRSP->nBytes = y;

	if ((sRegister > MB_MAX_DATALENGTH * 2) || (sRegister + nBytes > max_legth))
		return ILLEGAL_DATA_ADDRESS;

	if (pSt != NULL) {
		pSt += sRegister;

		for (uint16_t i = 0; i < nBytes; i += 2) {
			*(pSt + i) = *(pData + i + 1);
			*(pSt + i + 1) = *(pData + i);
		}
	}

	return NO_ERROR;
}

uint32_t Compute_MBUSRequest(uint8_t *data, uint8_t port, uint8_t slave,
		modbus_response_fn p_resp_fn)
{
	uint8_t *p_buf = data;
	uint8_t unit, function;
	uint16_t st_addr, reg_cnt;
	uint32_t Mb_Error = NO_ERROR;

	unit = *p_buf;

	if ((unit == slave) || (unit == 0xff))
		p_buf++;
	else
		return SLAVE_NOT_ME;

	function = *(p_buf++);
	st_addr = GetModBusReg(p_buf);
	p_buf += 2;

	reg_cnt = GetModBusReg(p_buf);
	p_buf += 2;

	if (function == FUNC_READ_HOLDING_REGISTERS
			|| function == FUNC_READ_INPUT_REGISTERS) {
		uint16_t icrc;

		stReadRSP.SlaveID = unit;
		stReadRSP.Function = function;
		stReadRSP.nBytes = (uint8_t) (reg_cnt << 1);

		icrc = swap_u16(GetModBusReg(p_buf));

		if (icrc != CRC16_compute((uint8_t*) data, p_buf - data))
			Mb_Error = ILLEGAL_DATA_VALUE;
		else {
			Mb_Error = ProcessMbusRequest((uint8_t*) &stReadRSP, NULL, st_addr,
					eREG_RD, port);
			if ((Mb_Error == NO_ERROR) && (p_resp_fn != NULL))
				p_resp_fn((uint8_t*) &stReadRSP.Data, stReadRSP.nBytes, port);
		}
	} else if (function == FUNC_WRITE_MULTIPLE_REGISTERS) {
		uint32_t end = (uint32_t) *(p_buf++);
		uint16_t icrc, _crc;

		stWRSP.SlaveID = unit;
		stWRSP.Function = function;
		stWRSP.Start = st_addr;
		stWRSP.nRegisters = reg_cnt;
		stWRSP.m_crc = 0;

		icrc = swap_u16(GetModBusReg(p_buf + end));
		_crc = CRC16_compute(data, (p_buf - data) + end);
		if (icrc != _crc) {
			uint16_t *p_ex = (void*) &MB_exception;
			*(p_ex++) = icrc;
			*(p_ex++) = _crc;
			*(p_ex++) = swap_u16(GetModBusReg(p_buf + end));

			Mb_Error = ILLEGAL_DATA_VALUE;
		} else {
			Mb_Error = ProcessMbusRequest((uint8_t*) &stWRSP, p_buf, st_addr,
					eREG_WR, port);
			if ((Mb_Error == NO_ERROR) && (p_resp_fn != NULL))
				p_resp_fn((uint8_t*) &stWRSP.Data, stWRSP.nBytes, port);
		}
	} else if (function == FUNC_READ_WRITE_MULTIPLE_REGISTERS) {
		uint16_t icrc;

		stReadRSP.SlaveID = unit;
		stReadRSP.Function = function;
		stReadRSP.nBytes = (uint8_t) (reg_cnt << 1);

		stWRSP.SlaveID = unit;
		stWRSP.Function = function;
		stWRSP.Start = GetModBusReg(p_buf);
		p_buf += 2;
		stWRSP.nRegisters = GetModBusReg(p_buf);
		p_buf += 2;
		stWRSP.nBytes = *(p_buf++);

		stWRSP.m_crc = 0;

		icrc = swap_u16(GetModBusReg(p_buf + stWRSP.nBytes));

		if (icrc != CRC16_compute(data, (p_buf - data) + stWRSP.nBytes))
			Mb_Error = ILLEGAL_DATA_VALUE;
		else {
			Mb_Error = ProcessMbusRequest((uint8_t*) &stWRSP, p_buf,
					stWRSP.Start, eREG_WR, port);
			if (Mb_Error == NO_ERROR) {
				Mb_Error = ProcessMbusRequest((uint8_t*) &stReadRSP, NULL,
						st_addr, eREG_RD, port);
				if ((Mb_Error == NO_ERROR) && (p_resp_fn != NULL))
					p_resp_fn((uint8_t*) &stReadRSP.Data, stReadRSP.nBytes,
							port);
			}
		}
	} else {
		Mb_Error = ILLEGAL_FUNCTION;
	}

	if (Mb_Error != NO_ERROR) {
		uint32_t c, z = 0;

		MB_exception[z++] = unit;
		/* Function code Adding 0x80 results in exception code */
		MB_exception[z++] = function + 0x80;
		MB_exception[z++] = Mb_Error;

		c = CRC16_compute(MB_exception, z);

		MB_exception[z++] = (uint8_t) c;
		MB_exception[z++] = (uint8_t) (c >> 8);
		p_resp_fn(MB_exception, z, port);
	}

	return (uint32_t) Mb_Error;
}

uint32_t ProcessMbusRequest(uint8_t *p_rsp, uint8_t *pdata, uint16_t start,
		uint16_t acces_type, uint8_t port)
{
	stMbusWriteRSP *mb_write_rsp = NULL;
	stMbusReadRSP *mb_read_rsp = NULL;
	mbQuery *pList = Get_CfgList();
	uint16_t acceslevel = GetSystemConfig()->password.Accesslevel.Reg;
	volatile uint16_t r_group, s_register;
	uint32_t error = ILLEGAL_DATA_ADDRESS;

	s_register = ((uint8_t) (start << 1));
	r_group = ((uint8_t) (start >> 8));

	if ((acces_type == eREG_WR) && (pdata != NULL))
		mb_write_rsp = (stMbusWriteRSP*) p_rsp;
	else if (acces_type == eREG_RD)
		mb_read_rsp = (stMbusReadRSP*) p_rsp;
	else
		return ILLEGAL_FUNCTION;

	for (uint32_t i = 0; i < GetListSize(); pList++, i++) {
		if (pList->group == r_group && ((pList->regdir & acces_type) != 0)
				&& ((acceslevel & pList->passlevel) != 0)) {
			if ((acces_type == eREG_WR) && (mb_write_rsp != NULL)) {
				uint32_t len = mb_write_rsp->nRegisters << 1;
				error = fWriteHldRegRSP(pList->pData, pdata, pList->length,
						mb_write_rsp, port);
				/* On write operation first write data
				 * and then call process function */
				if ((error == NO_ERROR) && (pList->pfunc != NULL))
					error = pList->pfunc(pList->pData, len, acces_type);
			} else if (mb_read_rsp != NULL) {
				/* On read operation first call process function
				 * and then write to buffer for send */
				error = (pList->pfunc != NULL) ?
						pList->pfunc(pList->pData,
								mb_read_rsp->nBytes, acces_type)
								: NO_ERROR;
				if (error == NO_ERROR)
					error = fReadHldRegRSP(pList->pData, s_register,
							pList->length, mb_read_rsp, port);
			}
			break;
		}
	}
	return error;
}

uint32_t fGetLastCOM_tm(uint32_t port)
{
	if (port < N_MBUS_PORTS)
		return lastCom_tm[port];
	else
		return 0;
}

uint32_t fGetErrCOM_Cnt(uint32_t port)
{
	if (port < N_MBUS_PORTS)
		return Error_cnt[port];
	else
		return 0;
}

void fClr_LastCOM(uint32_t port)
{
	if (port < N_MBUS_PORTS)
		lastCom_tm[port] = GetTick_us();
}

void ModbusInit(void)
{
	for (uint32_t x = 0; x < N_MBUS_PORTS; x++) {
		Error_cnt[x] = 0;
		lastCom_tm[x] = GetTickCount();
	}
}
