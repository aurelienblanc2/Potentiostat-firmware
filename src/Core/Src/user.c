/*
 * user.c
 *
 *  Created on: 7 nov. 2017
 *      Author: agarcia
 */

#include "user.h"
#include "usbd_cdc_if.h"

extern TIM_HandleTypeDef htim5;
extern RTC_HandleTypeDef hrtc;

__attribute__ ((section(".firm_version")))stVersion FirmVersion = { .major = 1,
		.minor = 0, .patch = 0, .pcb_rev = 'A', .pcb_var = '1', .TimeStamp =
				__TIMESTAMP__, };

const uint8_t dflt_passwort[] = "Poten1!";

const PID_Config pid_cfg_dflt = {
	.kp = 100,
	.kd = 0.01,
	.ki = 0.000025,
	.err_min = -0.00000000001,
	.err_max = 0.00000000001,
	.itgl_max = 10.0,
	.itgl_min = -10.0,
	.rate_max = 0.05,
	.rate_min = -0.05,
	.output_max = 5.0,
	.output_min = -5.0,
};

PID_Param pid_par = {0};

u_chipserial chip_serial;
u_chip_ID64 chip_ID64;
struct tm sys_dtime;

uint32_t bSaveNVM = 0;

/* USB cdc to modbus variable definition */

uint32_t usb_rx_tmr = 0;	// Last communication time in ms
uint32_t rcv_ix = 0;		// Received bytes counter
uint8_t usb_frame_buf[MODBUS_BUFFER_LEN] = { 0 };	// Received bytes buffer

shrc3_ctx ht_ctx = { 0 };

system_config sys_cfg = { 0 };

uint8_t* GetChipSerial(void)
{
	chip_serial.Serial32[0] = *((uint32_t*) UID_BASE);
	chip_serial.Serial32[1] = *((uint32_t*) (UID_BASE + 4));
	chip_serial.Serial32[2] = *((uint32_t*) (UID_BASE + 8));
	chip_serial.Serial32[3] = 0;
	return chip_serial.Serial8;
}

uint8_t* GetChipID64(void)
{
	chip_ID64.dword[0] = *((uint32_t*) UID_BASE);
	chip_ID64.byte[4] = *((uint32_t*) (UID_BASE + 4));
	chip_ID64.byte[5] = *((uint32_t*) (UID_BASE + 8));
	chip_ID64.byte[6] = *((uint32_t*) (UID_BASE + 9));
	chip_ID64.byte[7] = *((uint32_t*) (UID_BASE + 10));
	return chip_ID64.byte;
}

struct tm* GetDateTime_st(void)
{
	return &sys_dtime;
}

system_config* GetSystemConfig(void)
{
	return &sys_cfg;
}
uint8_t* GetDfltPass(void)
{
	return (uint8_t*) dflt_passwort;
}

struct tm* GetSysDateTime(void)
{
	return &sys_dtime;
}

void delay_us(uint32_t delay)
{
	uint32_t t_start = TIM5->CNT;
	while ((TIM5->CR1 & TIM_CR1_CEN) && TimeDiff(t_start, TIM5->CNT) < delay);
}

uint32_t* GetTick_us_hldr(void)
{
	return (uint32_t*) &TIM5->CNT;
}

inline uint32_t GetTick_us(void)
{
	return TIM5->CNT;
}

uint32_t GetUint32FromBuffer(uint8_t *p_d)
{
	return (uint32_t) ((*p_d << 24) | (*(p_d + 1) << 16) | (*(p_d + 2) << 8)
			| *(p_d + 3));
}

uint64_t GetUint64FromBuffer(uint8_t *p_d)
{
	uint32_t l_w, h_w;

	h_w = (*p_d << 24) | (*(p_d + 1) << 16) | (*(p_d + 2) << 8) | *(p_d + 3);
	l_w = (*(p_d + 4) << 24) | (*(p_d + 5) << 16) | (*(p_d + 6) << 8)
			| *(p_d + 7);

	return (uint64_t) ((((uint64_t) h_w) << 32) | l_w);
}

/* Compute the CRC of data buffer pointed by *data with length in bytes */
uint16_t CRC16_compute(uint8_t *data, uint16_t length)
{
	CRC_HandleTypeDef h_crc;
	uint16_t crc16 = 0;

	h_crc.Instance = CRC;
	h_crc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
	h_crc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
	h_crc.Init.GeneratingPolynomial = 0x8005;
	h_crc.Init.CRCLength = CRC_POLYLENGTH_16B;
	h_crc.Init.InitValue = 0xffff;
	h_crc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
	h_crc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
	h_crc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;

	if (HAL_CRC_Init(&h_crc) == HAL_OK)
		crc16 = HAL_CRC_Calculate(&h_crc, (uint32_t*) data, length);

	return crc16;
}

uint32_t CRC32_compute(uint8_t *buffer, uint32_t size)
{
	CRC_HandleTypeDef h_crc;
	uint32_t crc32 = 0;

	h_crc.Instance = CRC;
	h_crc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
	h_crc.Init.DefaultInitValueUse = DEFAULT_POLYNOMIAL_DISABLE;
	h_crc.Init.GeneratingPolynomial = 0x04C11DB7;
	h_crc.Init.CRCLength = CRC_POLYLENGTH_32B;
	h_crc.Init.InitValue = 0xFFFFFFFF;
	h_crc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
	h_crc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
	h_crc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;

	/* Compute CRC32 and make One Complement crc32 */
	if (HAL_CRC_Init(&h_crc) == HAL_OK)
		crc32 = ~HAL_CRC_Calculate(&h_crc, (uint32_t*) buffer, size);

	return crc32;
}

/*******************************************************************
 Method      :  TimeDiff

 Description :
 This function gets the difference between two time stamp, time
 overflow is considered.
 **********************************************************************/

inline uint32_t TimeDiff(uint32_t time_start, uint32_t time_end)
{
	return TIMEDIFF(time_start, time_end);
}

/*******************************************************************
 Method      :  TimeExpired

 Description :
 This function get ellapsed time from time_start
 **********************************************************************/

inline uint32_t EllapsedTime(uint32_t time_start)
{
	return TIMEDIFF(time_start, HAL_GetTick());
}

/*******************************************************************
 Method      :  TimeExpired

 Description :
 This function check if an interval time was expired
 **********************************************************************/

inline uint32_t TimeExpired(uint32_t start, uint32_t interval)
{
	return (TIMEDIFF(start, HAL_GetTick()) > interval);
}

/* 3 digits BIN to BCD converter */

uint32_t fBin2BCD(uint8_t u)
{
	uint8_t c, d;

	for (c = 0; u >= 100; u -= 100, c++);
	for (d = 0; u >= 10; u -= 10, d++);
	return (uint32_t) ((c << 8) | (d << 4) | u);
}

/* 3 digits BCD to BIN converter */

inline uint32_t fBCD2Bin(uint8_t n)
{
	return (uint32_t) ((100 * (n & 0xf00) >> 8)
			+ (10 * (n & 0xf0) >> 4)
			+ (n & 0xf));
}

void StructTm_To_PackDt(PacketTime *pPacktDT, struct tm *pDT)
{
	pPacktDT->sec = (uint8_t) pDT->tm_sec;
	pPacktDT->min = (uint8_t) pDT->tm_min;
	pPacktDT->hour = (uint8_t) pDT->tm_hour;
	pPacktDT->day = (uint8_t) pDT->tm_mday;
	pPacktDT->mon = (uint8_t) (pDT->tm_mon + 1);
	pPacktDT->year = (uint8_t) (pDT->tm_year + (CUT_YEAR - RTC_CENTURY));
}

void PackDT_To_StructTm(PacketTime *pPacktDT, struct tm *pDT)
{
	pDT->tm_sec = (int32_t) pPacktDT->sec;
	pDT->tm_min = (int32_t) pPacktDT->min;
	pDT->tm_hour = (int32_t) pPacktDT->hour;
	pDT->tm_mday = (int32_t) pPacktDT->day;
	pDT->tm_mon = (int32_t) (pPacktDT->mon - 1);
	pDT->tm_year = (int32_t) (pPacktDT->year + (RTC_CENTURY - CUT_YEAR));
}

int32_t GetDatetimeFromIRTC(void *arg)
{
	struct tm *p_dt = arg;
	RTC_DateTypeDef date_def;
	RTC_TimeTypeDef time_def;

	if (hrtc.Instance->ICSR & RTC_ICSR_RSF) {
		/* Call first get time to avoid long update time lapsus */
		HAL_RTC_GetTime(&hrtc, &time_def, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &date_def, RTC_FORMAT_BIN);

		p_dt->tm_hour = time_def.Hours + p_dt->tm_isdst;
		p_dt->tm_min = time_def.Minutes;
		p_dt->tm_sec = time_def.Seconds;

		p_dt->tm_mday = date_def.Date;
		p_dt->tm_mon = date_def.Month - RTC_MONTH_JANUARY;
		p_dt->tm_year = (uint8_t) (date_def.Year + (RTC_CENTURY - CUT_YEAR));
		p_dt->tm_wday = date_def.WeekDay - RTC_WEEKDAY_MONDAY;

		BlinkLed_Idx_Start(LED_GREEN, 1);
		return SUCCESS;
	}
	return ERROR;
}

int32_t SetDatetimeToInternalRTC(struct tm *p_dt)
{
	int32_t result = HAL_OK;
	int32_t hour = p_dt->tm_hour - p_dt->tm_isdst;
	RTC_DateTypeDef date_def;
	RTC_TimeTypeDef time_def;

	if (hour < 0)
		hour = 23;

	time_def.Hours = hour;
	time_def.Minutes = p_dt->tm_min;
	time_def.Seconds = p_dt->tm_sec;
	time_def.SubSeconds = 0;
	time_def.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	time_def.StoreOperation = RTC_STOREOPERATION_RESET;
	result = HAL_RTC_SetTime(&hrtc, &time_def, RTC_FORMAT_BIN);

	date_def.Date = p_dt->tm_mday;
	date_def.Month = p_dt->tm_mon + RTC_MONTH_JANUARY;
	date_def.Year = (uint8_t) (p_dt->tm_year + (CUT_YEAR - RTC_CENTURY));
	date_def.WeekDay = p_dt->tm_wday + RTC_WEEKDAY_MONDAY;
	result |= HAL_RTC_SetDate(&hrtc, &date_def, RTC_FORMAT_BIN);

	return result;
}

int32_t Read_I2C_Buffer(void *handler, uint8_t addr, void *buffer,
		uint16_t length)
{
	return HAL_I2C_Master_Receive((I2C_HandleTypeDef*) handler, addr,
			(uint8_t*) buffer, length, 1500);
}

int32_t Write_I2C_Buffer(void *handler, uint8_t addr, void *buffer,
		uint16_t length)
{
	return HAL_I2C_Master_Transmit((I2C_HandleTypeDef*) handler, addr,
			(uint8_t*) buffer, length, 1500);
}

/* Send modbus response */

int32_t CDC_Response_Frame(void *data, uint16_t length, uint8_t port)
{
	return (int32_t) CDC_Transmit_FS(data, length);
}

/* Process received usb modbus data frame */

uint32_t frame_det_tm = MODBUS_DET_TIME;

void usb_cdc_runtime(void)
{
	if (rcv_ix && (TIMEDIFF(usb_rx_tmr, htim5.Instance->CNT) > frame_det_tm)) {

		if (Compute_MBUSRequest(usb_frame_buf, USB_PORT, MODBUS_USB_ID,
				CDC_Response_Frame) == NO_ERROR)
			BlinkLed_Idx_Start(LED_BLUE, 1);

		rcv_ix = 0;
	}
}

void LoadSystemConfigDefaults(void)
{
	memset(&sys_cfg, 0, sizeof(system_config));
	memcpy(sys_cfg.p_dac_cfg, GetDAC_Cfg_Dflt(), eDAC_MAX * sizeof(stDAC_Cfg));
	memcpy(sys_cfg.p_ancfg, GetAnalog_cfg_dflt(), eANCH_MAX * sizeof(stAnCfg));
	memcpy(&sys_cfg.pid_cfg, &pid_cfg_dflt, sizeof(PID_Config));

	sys_cfg.tm.frame_det = MODBUS_DET_TIME;
	sys_cfg.tm.adc_smp = ADC_CH_POLLING_TIME;
	sys_cfg.tm.fifo_smp = POTCTRL_POLLING_TIME;
	sys_cfg.tm.ht_smp = HT_SAMPLING_TIME;

	memcpy(sys_cfg.chip_serial.Serial8, GetChipSerial(),
			sizeof(sys_cfg.chip_serial.Serial8));

	sys_cfg.password.Accesslevel.Reg = eALLACCESS;
}

/****************************************************************************
 * PID process functions
 ****************************************************************************/

/*! \fn int Compute_PID(void *par)

	\brief	Advanced Power Control PID.

	\param[in]	par	pointer to PID_Param struct

	\return		PID_OUTPUT_CHANGED
	\return 	PID_OUTPUT_UNCHANGED
 */

int32_t Compute_PID(void *par)
{
	PID_Param *pid = par;
	PID_Config *p_cfg;
	float dt, slp, in;
	uint32_t clk;

	if((pid == NULL) || (pid->cfg == NULL) || (pid->invar == NULL))
		return EPID_ERROR;

	p_cfg = pid->cfg;
	in = *pid->invar;

	/* Update PID time register */
	clk = GetTickCount();
	dt = TIMEDIFF(pid->st_tm, clk);
	pid->st_tm = clk;

	/* Check if itn's on dead-band zone */
	pid->error = (pid->target - in);
	if (OUTWARD(pid->error, pid->cfg->err_min, pid->cfg->err_max)) {
		float res;
		int32_t ret;
		pid->derv = (pid->error - pid->l_error) / dt;
		pid->itgl += (pid->error * dt);
		pid->l_error = pid->error;

		/* Integral limiter */

		pid->itgl = CLAMP(pid->itgl, p_cfg->itgl_min, p_cfg->itgl_max);
		res =  (p_cfg->kp * pid->error);
		res += (p_cfg->kd * pid->derv);
		res += (p_cfg->ki * pid->itgl);

		/* Changing rate limiter */
		slp = (res / dt);
		if (OUTWARD(slp, p_cfg->rate_min, p_cfg->rate_max)) {
			slp = CLAMP(slp, p_cfg->rate_min, p_cfg->rate_max);
			res = slp * dt;
		}

		/* Accumulate result increment to output value */
		pid->output += (isfinite(res)) ? res : 0;

		ret = OUTWARD(pid->output, p_cfg->output_min, p_cfg->output_max) ?
				EPID_OUTPUT_LIMITED : EPID_OUTPUT_CHANGED;

		/* Output limiter  */
		pid->output = CLAMP(pid->output, p_cfg->output_min, p_cfg->output_max);
		pid->l_output = pid->output;

		return ret;
	}

	pid->itgl = 0;
	return EPID_OUTPUT_UNCHANGED;
}

int32_t Humidity_Sensor_Init(void *i2c_hdlr)
{
	int32_t res = HAL_ERROR;
	potentiostat_param* pot;
	if (i2c_hdlr == NULL)
		return res;

	pot = GetPotentiostatParam();
	ht_ctx.addr = (SHRC3_ADDR << 1);
	ht_ctx.handler = i2c_hdlr;
	ht_ctx.read_reg = Read_I2C_Buffer;
	ht_ctx.write_reg = Write_I2C_Buffer;

	ht_ctx.poll_tm = sys_cfg.tm.ht_smp;
	ht_ctx.temperature = &pot->ht.extemp;
	ht_ctx.humidity = &pot->ht.humidity;

	SHTC3_Init(&ht_ctx);

	res =  HAL_I2C_Init((I2C_HandleTypeDef*) i2c_hdlr);
	if (res == HAL_OK) {
		uint16_t id;
		res = SHTC3_Wakeup();
		res |= SHTC3_GetId(&id);
		res = ((res == HAL_OK) && ((id & ID_MASK) == ID_VALUE)) ?
				HAL_OK : HAL_ERROR;
	}
	return res;
}

void PID_Init(PID_Config *cfg, float *input_var)
{
	pid_par.cfg = cfg;
	pid_par.invar = input_var;
}

