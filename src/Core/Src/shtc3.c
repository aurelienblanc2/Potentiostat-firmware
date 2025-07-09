//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHTC3 Sample Code (V1.0)
// File      :  shtc3.c (V1.0)
// Author    :  RFU
// Date      :  24-Nov-2017
// Controller:  STM32F100RB
// IDE       :  �Vision V5.17.0.0
// Compiler  :  Armcc
// Brief     :  Sensor Layer: Implementation of functions for sensor access.
//==============================================================================

#include "user.h"

static etError SHTC3_Read2BytesAndCrc(uint16_t *data);
static etError SHTC3_WriteCommand(etCommands cmd);
static etError SHTC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,
		uint8_t checksum);

static shrc3_ctx *p_ctx;

void SHTC3_Init(shrc3_ctx *dev_ctx)
{
	p_ctx = dev_ctx;
}

etError SHTC3_GetTempAndHumiPolling(float *temp, float *humi)
{
	etError res;           // error code

	shtc3_raw_ht ht_data;

	// measure, read temperature first, clock stretching disabled (polling)
	res = SHTC3_WriteCommand(MEAS_T_RH_POLLING);

	if (res != SHTC3_NO_ERROR)
		return res;

	res = SHTC3_ACK_ERROR;

	// poll every 1ms for measurement ready
	for (int x = 0; (x < SHTC3_MAX_RETRIES) && (res != SHTC3_NO_ERROR);
			--x, delay_us(1000))
		res = p_ctx->read_reg(p_ctx->handler, p_ctx->addr, &ht_data,
				sizeof(ht_data));

	// if no error, calculate temperature in ºC and humidity in %RH
	if (res == SHTC3_NO_ERROR) {
		*temp = SHTC3_CALC_TEMPERATURE((float)swap16(ht_data.i_temp));
		*humi = SHTC3_CALC_HUMIDITY((float)swap16(ht_data.i_hum));
	}
	return res;
}

etError SHTC3_GetId(uint16_t *id)
{
	etError res = SHTC3_WriteCommand(READ_ID);

	if (res == SHTC3_NO_ERROR)
		SHTC3_Read2BytesAndCrc(id);

	return res;
}

etError SHTC3_Sleep(void)
{
	return SHTC3_WriteCommand(SLEEP);
}

etError SHTC3_Wakeup(void)
{
	etError error = SHTC3_WriteCommand(WAKEUP);
	return error;
}

etError SHTC3_SoftReset(void)
{
	// write reset command

	return SHTC3_WriteCommand(SOFT_RESET);;
}

static etError SHTC3_WriteCommand(etCommands cmd)
{
	uint16_t cmd_buf = swap16(cmd);

	// Timeout = Ti2cclk * 2048 * Timeout (12bit) = Timeout * 12us for 170MHz
	return p_ctx->write_reg(p_ctx->handler, p_ctx->addr, &cmd_buf,
			sizeof(cmd_buf));

}

static etError SHTC3_Read2BytesAndCrc(uint16_t *data)
{
	uint8_t rd_buf[3]; // read data array

	int32_t res = p_ctx->read_reg(p_ctx->handler, p_ctx->addr, rd_buf,
			sizeof(rd_buf));

	if (res == SHTC3_NO_ERROR) {
		// verify checksum that's 3 byte
		res = SHTC3_CheckCrc(rd_buf, 2, rd_buf[2]);

		// combine the two data bytes to a 16-bit value
		if (res == SHTC3_NO_ERROR)
			*data = (rd_buf[0] << 8) | rd_buf[1];
	}

	return res;
}

static etError SHTC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,
		uint8_t checksum)
{
	uint8_t bit;        // bit mask
	uint8_t crc = 0xFF; // calculated checksum
	uint8_t byteCtr;    // byte counter

	// calculates 8-Bit checksum with given polynomial
	for (byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
		crc ^= (data[byteCtr]);
		for (bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ CRC_POLYNOMIAL;
			} else {
				crc = (crc << 1);
			}
		}
	}

	// verify checksum
	if (crc != checksum) {
		return SHTC3_CHKSUM_ERROR;
	} else {
		return SHTC3_NO_ERROR;
	}
}

static etError SHTC3_GetFormattedResult(void *data, eDataIndex ix, float *result)
{
	const int l = sizeof(uint16_t);
	uint8_t *buf;
	int32_t res;

	if((data == NULL) || (result == NULL))
		return -1;

	buf = data;
	buf += (ix == TEMPERATURE_IX) ? TEMPERATURE_IX : HUMIDITY_IX;
	res = SHTC3_CheckCrc(buf, l, buf[l]);
	if ( res ==  SHTC3_NO_ERROR) {
		float x = (float)SHTC3_GET_DATA_FROM_BUFFER(buf);
		*result = (ix == TEMPERATURE_IX) ?
				SHTC3_CALC_TEMPERATURE(x) : SHTC3_CALC_HUMIDITY(x);
	} else {
		*result = NAN;
	}

	return res;
}

EHT_Status Humidity_Sensor_Runtime(void)
{
	static EHT_Status ht_state= EHT_IDLE;
	static uint32_t rty = 0, st_tm = 0;
	static uint8_t buf[8] = {0};
	static uint32_t max_cycle_tm = 0, cycle_tm = 0;
	uint32_t dif_tm = 0;
	etError res = SHTC3_NO_ERROR;

	cycle_tm = GetTick_us();

	switch (ht_state) {
	case EHT_IDLE:
		if (p_ctx->poll_tm
				&& (EllapsedTime(st_tm) > p_ctx->poll_tm)
				&& (SHTC3_Wakeup() == SHTC3_NO_ERROR)){
			rty = 0;
			st_tm = GetTickCount();
			ht_state = EHT_START_MEAS;
		}
		break;
	case EHT_START_MEAS:
		if (EllapsedTime(st_tm) > SHRC3_WAKEUP_TIME) {
			res = SHTC3_WriteCommand(MEAS_T_RH_POLLING);
			st_tm = GetTickCount();
			ht_state = (res == SHTC3_NO_ERROR) ? EHT_WAIT_DATA : EHT_ERROR;
		}
		break;
	case EHT_WAIT_DATA:
		if (EllapsedTime(st_tm) > SHRC3_CAPTURE_TIME) {
			res = p_ctx->read_reg(p_ctx->handler,
					p_ctx->addr, buf, SHTC3_READ_RESULT_LEN);
			if (res == SHTC3_NO_ERROR) {
				ht_state = EHT_FORMAT_DATA;
			} else  if ((++rty) > SHTC3_MAX_RETRIES) {
				ht_state = EHT_ERROR;
			}
			st_tm = GetTickCount();
		}
		break;
	case EHT_FORMAT_DATA:
		SHTC3_GetFormattedResult(buf, TEMPERATURE_IX, p_ctx->temperature);
		SHTC3_GetFormattedResult(buf, HUMIDITY_IX, p_ctx->humidity);
		ht_state = EHT_END;
		break;
	case EHT_ERROR:
	case EHT_END:
		SHTC3_Sleep();
		st_tm = GetTickCount();
		ht_state = EHT_IDLE;
		break;
	}

	dif_tm = TIMEDIFF(cycle_tm, GetTick_us());

	if(dif_tm  > max_cycle_tm)
		max_cycle_tm = dif_tm;

	return ht_state;
}




