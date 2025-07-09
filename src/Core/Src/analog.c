/*
 * Analog.c
 *
 *  Created on: Jan 11, 2021
 *      Author: angel
 */

#include "user.h"

extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef hadc1, hadc3, hadc5;
extern DAC_HandleTypeDef hdac1, hdac2, hdac3;
extern TIM_HandleTypeDef htim5;
extern system_config sys_cfg;
extern PID_Param pid_par;

/* Function prototypes */
static void ProcessAnalog_VREF(void*);
static void ProcessAnalog(void*);
static void PocessWEOUT(void*);
static void PocessREOUT(void*);
static void ProcessDacValue(void*);

/* Declare 2 buffer one for DMA ping-pong operation and another
 * for process data purposes 3 adc's 2 buffers and 3 channels for each adc */

uint16_t adc_buf[3][2][3] __attribute__ ((aligned (8)));

/* Calibrated VREFINT value at 30ºC */

float verfint_cal;
float k_adc = KADC16;

const ADC_HandleTypeDef *p_adc_ins[3] = { &hadc1, &hadc3, &hadc5 };
const uint32_t adc_ch_qty[3] = { ADC1_N_CH, ADC3_N_CH, ADC5_N_CH };

/* Set ADC's and analog input settings */
/* Set *val pointer to &adc_buf[x][1][x] instead &adc_buf[x][0][x]
 * because it's working buffer */

const stADCChn adc_ch_cfg[] = {
		{ &hadc1, eADC1_WEOUT, &adc_buf[0][1][0] },
		{ &hadc3, eADC3_REOUT, &adc_buf[1][1][0] },
		{ &hadc5, eADC5_VREFINT, &adc_buf[2][1][0] },
		{ &hadc5, eADC5_MCU_TEMP, &adc_buf[2][1][1] },
		{ &hadc5, eADC5_MCU_VBAT, &adc_buf[2][1][2] }
};

const stAnCfg an_cfg_dflt[] = {
		[eANCH_WEOUT] = { .AN = { 1.0, 0 }, 100 },
		[eANCH_REOUT] = { .AN = { VREOUT_GAIN, VREOUT_OFFSET }, 100 },
		[eANCH_VREFINT] = { .AN = {	1.0, 0 }, 10 },
		[eANCH_MCU_TEMP] = { .AN = { 1.0, 0 }, 10 },
		[eANCH_MCU_VBAT] = { .AN = { 3.0, 0 }, 10 }
};

stAnCfg *p_ancfg = &sys_cfg.p_ancfg[0];

stAnalogData an_values[] = {
		{ eANCH_WEOUT,
				PocessWEOUT,
				(stADCChn*) &adc_ch_cfg[eANCH_WEOUT],
				&sys_cfg.p_ancfg[eANCH_WEOUT] },
		{ eANCH_REOUT,
				PocessREOUT,
				(stADCChn*) &adc_ch_cfg[eANCH_REOUT],
				&sys_cfg.p_ancfg[eANCH_REOUT] },
		{ eANCH_VREFINT,
				ProcessAnalog_VREF,
				(stADCChn*) &adc_ch_cfg[eANCH_VREFINT],
				&sys_cfg.p_ancfg[eANCH_VREFINT] },
		{ eANCH_MCU_TEMP,
				ProcessAnalog,
				(stADCChn*) &adc_ch_cfg[eANCH_MCU_TEMP],
				&sys_cfg.p_ancfg[eANCH_MCU_TEMP] },
		{ eANCH_MCU_VBAT,
				ProcessAnalog,
				(stADCChn*) &adc_ch_cfg[eANCH_MCU_VBAT],
				&sys_cfg.p_ancfg[eANCH_MCU_VBAT] }
};

/* Set comparator and DAC's settings */

const st_DAC_Chn dac_ch_cfg[] = {
		[eDAC_VCEIN] = { &hdac1, DAC_CHANNEL_1 },
		[eDAC_TOAREF] = { &hdac2, DAC_CHANNEL_1 },
		[eDAC_OPVREF] = { &hdac3, DAC_CHANNEL_2 }
};


const stDAC_Cfg dac_cfg_dflt[] = {
		[eDAC_VCEIN] = { VCEIN_GAIN, VCEIN_OFFSET, 0},
		[eDAC_TOAREF] = { 1.0, 0, 0 },
		[eDAC_OPVREF] = { 1.0, 0, HALF_V_SUPPLY }
};

stDAC_Cfg *p_dac_cfg = &sys_cfg.p_dac_cfg[0];

stDAC_Data dac_values[] = {
		{ eDAC_VCEIN,
				ProcessDacValue,
				(st_DAC_Chn*) &dac_ch_cfg[eDAC_VCEIN],
				&sys_cfg.p_dac_cfg[eDAC_VCEIN] },
		{ eDAC_TOAREF,
				ProcessDacValue,
				(st_DAC_Chn*) &dac_ch_cfg[eDAC_TOAREF],
				&sys_cfg.p_dac_cfg[eDAC_TOAREF] },
		{ eDAC_OPVREF,
				ProcessDacValue,
				(st_DAC_Chn*) &dac_ch_cfg[eDAC_OPVREF],
				&sys_cfg.p_dac_cfg[eDAC_OPVREF] }
};

float adc_fifo_buf[ADC_FIFO_CNT];
float dac_fifo_buf[DAC_FIFO_CNT];
FIFO_ctrl fifo_adc, fifo_dac;
potentiostat_param poten_par = { 0 };

potentiostat_param* GetPotentiostatParam(void) {
	return &poten_par;
}

stADCChn* GetADC_Channels(void) {
	return (stADCChn*) adc_ch_cfg;
}

stAnCfg* GetAnalog_cfg_dflt(void) {
	return (stAnCfg*) an_cfg_dflt;
}

stAnCfg* GetAnalog_cfg(void) {
	return p_ancfg;
}

stAnalogData* GetAnalogData(uint32_t channel) {
	return (channel < eANCH_MAX) ? &an_values[channel] : NULL;
}

stDAC_Data* GetDACData(void) {
	return dac_values;
}

stDAC_Cfg* GetDAC_Cfg_Dflt(void) {
	return (stDAC_Cfg*) dac_cfg_dflt;
}

void ProcessAnalog_VREF(void *p_val) {
	stAnalogData *p_an = p_val;
	stAnCfg *p_cfg = p_an->p_cfg;
	float x = ((float) *p_an->p_chCfg->val);

	p_an->current = ((verfint_cal * ADC16_CNTS) / x);
	p_an->mean = CUMULATIVE_AVERAGE(p_an->current, p_an->mean, p_cfg->samples);
}

static void Init_MCU_Temp_Constants(stAnCfg *p_cfg) {
	/* MCU temperature sensor calibration parameters stored in ROM */

	float k_ts = ((float) ((*TEMPSENSOR_CAL2_ADDR) - (*TEMPSENSOR_CAL1_ADDR)))
			/ ((float) (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP));
	float ts_vcal1 = ((float) (((*TEMPSENSOR_CAL1_ADDR)
			* TEMPSENSOR_CAL_VREFANALOG)) / ((float) (ADC12_CNTS * 1000)));

	p_cfg->AN.Slope = k_ts;
	p_cfg->AN.Offset = ((float) TEMPSENSOR_CAL1_TEMP) - (k_ts * ts_vcal1);
}

static void ProcessAnalog(void *p_val) {
	stAnalogData *p_an = (stAnalogData*) p_val;
	stAnCfg *p_cfg = p_an->p_cfg;
	float x = ((float) *(p_an->p_chCfg->val)) * k_adc;
	p_an->current = LINE_ADJUST(x, p_cfg->AN.Slope, p_cfg->AN.Offset);
	p_an->mean = CUMULATIVE_AVERAGE(p_an->current, p_an->mean, p_cfg->samples);
}

/* TIA current measurement formulas
 * VWE = TOAref - (GX * I), TOAref is set by DAC2
 * WEout = OPref - (GI * VWE),
 * OPref is set by DAC3 = (VDD / 2) = 1.65V, GI = (1 / 3)
 * WEout = OPref - (GI * (TOAref - (GX * I)), replacing VWE
 * I = (GI * TOAref - OPref + WEout)/(GI * GX), cleaning I
 */

static void PocessWEOUT(void *p_val) {
	stAnalogData *p_an = (stAnalogData*) p_val;
	float i, vwe;

	ProcessAnalog(p_val);

	/* Get present VWE value in volts */
	vwe = INV_WEO_GAIN * (poten_par.dac.opvref - p_an->mean);
	/* Get present TIA current in amps */
	i = (poten_par.dac.toaref - vwe) / poten_par.ctrl.gx;
	/* Test resulting current is a number */
	i = isnormal(i) ? i : 0;

	/* Update poten_par value and state */
	poten_par.adc.weout = i;
	poten_par.ctrl.st |= EPOT_ST_WE;
	;
}

static void PocessREOUT(void *p_val) {
	stAnalogData *p_an = (stAnalogData*) p_val;

	ProcessAnalog(p_val);
	poten_par.adc.reout = p_an->mean;
	poten_par.ctrl.st |= EPOT_ST_RE;
}

/* Set DAC to value in volts */

static void ProcessDacValue(void *p_st) {
	float *potdac = &poten_par.dac.vcein;
	stDAC_Data *p_dac = p_st;
	float x = LINE_ADJUST(p_dac->set_val,
			p_dac->p_cfg->Slope, p_dac->p_cfg->Offset);

	/* Update POTENTIOSTAT DAC values */
	if ((p_dac->id < eDAC_MAX) && isfinite(x)) {
		potdac[p_dac->id] = x;

		/* If OPVref value is modified update VCEIN offset value */
		if(p_dac->id == eDAC_OPVREF){
			p_dac_cfg[eDAC_VCEIN].Offset = x;
			p_ancfg[eANCH_REOUT].AN.Offset = -p_ancfg[eANCH_REOUT].AN.Slope * x;
		}

		HAL_DAC_SetValue(p_dac->p_chCfg->nDAC, p_dac->p_chCfg->nChannel,
				DAC_ALIGN_12B_R, CLAMP(lrintf(x * KDAC12), 0, DAC12_CNTS));
		//HAL_DAC_Start(p_dac->p_chCfg->nDAC, p_dac->p_chCfg->nChannel);
	}
}

void Set_DAC_Value(eDAC_CHANNELS channel, float val) {
	if (channel < eDAC_MAX) {
		dac_values[channel].set_val = val;
		dac_values[channel].dac_handle(&dac_values[channel]);
	}
}

void DAC_Init(stDAC_Cfg *p_cfg) {
	memcpy(p_dac_cfg, p_cfg, eDAC_MAX * sizeof(stDAC_Cfg));

	for (eDAC_CHANNELS x = 0; x < eDAC_MAX; x++)
		Set_DAC_Value(x, p_dac_cfg[x].value);
}

void Analog_Init(stAnCfg *p_cfg) {
	/* Selected Trans-impedance amplifier initial Gain */
	Set_TIA_Gain(eTIA_SW_GAIN_100K);

	/* Default values initialization */
	for (uint32_t i = 0; i < eANCH_MAX; i++) {
		p_ancfg[i] = p_cfg[i];
		an_values[i].current = 0;
		an_values[i].mean = 0;
	}

	/* Internal band gap reference calibrated value at 30ºC */
	verfint_cal = (float) ((VREFINT_CAL_VREF * (*VREFINT_CAL_ADDR))
			/ (ADC12_CNTS * 1000));
	an_values[eANCH_MCU_TEMP].mean = k_adc;

	/* MCU temperature sensor constants initialization */
	Init_MCU_Temp_Constants(an_values[eANCH_MCU_TEMP].p_cfg);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc_buf[0][0][0], ADC1_N_CH);
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &adc_buf[1][0][0], ADC3_N_CH);
	HAL_ADC_Start_DMA(&hadc5, (uint32_t*) &adc_buf[2][0][0], ADC5_N_CH);

	/* Initialize ADC FIFO queue */
	InitFIFO(&fifo_adc, adc_fifo_buf, sizeof(adc_fifo_buf),
			sizeof(poten_group_adc));

	/* Initialize DAC FIFO queue */
	InitFIFO(&fifo_dac, dac_fifo_buf, sizeof(dac_fifo_buf), sizeof(float));
}

void SetCESwitch(uint32_t val)
{
	poten_par.ctrl.ce_sw = (val != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
	HAL_GPIO_WritePin(CE_EN_GPIO_Port,
			CE_EN_Pin,
			poten_par.ctrl.ce_sw);
}

/*! \fn float Set_TIA_Gain(eTIA_GAIN gain)


 \brief Set trans-impedande amplifier gain.

 \param[in] 	gain, allowed values are in type enum eTIA_GAIN
 \return		real gain value in float
 */

float Set_TIA_Gain(eTIA_GAIN gain) {
	const tia_switch tia_switch_matrix[] = { { SW_GAIN_1K, TIA_GAIN_1K }, {
			SW_GAIN_10K, TIA_GAIN_10K }, { SW_GAIN_100K, TIA_GAIN_100K }, {
			SW_GAIN_1M, TIA_GAIN_1M }, { SW_GAIN_10M, TIA_GAIN_10M } };

	uint32_t val = LL_GPIO_ReadOutputPort(GPIOB);
	uint32_t ix = CLAMP(gain, eTIA_SW_GAIN_1K, eTIA_SW_GAIN_1M);

	/* Clear TIA gain switch bits.
	 * Set TIA switch gain bits.*/
	val &= ~TIA_SW_MASK;
	val |= (tia_switch_matrix[ix].value & TIA_SW_MASK);
	LL_GPIO_WriteOutputPort(GPIOB, val);

	/* Update POTENTIOSTAT structure gain parameter */
	poten_par.ctrl.gx = tia_switch_matrix[ix].f_gain;

	return poten_par.ctrl.gx;
}

static void Process_Potentiostat_FIFO(void) {
	static uint32_t fifo_tm = 0;
	uint32_t stp = 0;
	uint32_t *p_tm_us = GetTick_us_hldr();
	uint32_t dt = TIMEDIFF(fifo_tm, *p_tm_us);

	/* Wait for reaching FIFO sample time */
	if (dt < sys_cfg.tm.fifo_smp)
		return;

	poten_par.ctrl.samp_tm *= (SMP_MAVG - 1);
	poten_par.ctrl.samp_tm += dt;
	poten_par.ctrl.samp_tm >>= SMP_MAVG_SH;
	if (poten_par.ctrl.st & EPOT_ST_PID) {
			int32_t res = Compute_PID(&pid_par);

			if( res != EPID_OUTPUT_UNCHANGED) {
				Set_DAC_Value(eDAC_VCEIN, pid_par.output);
			}
			if(res == EPID_OUTPUT_LIMITED)
				BlinkLed_Idx_Start(LED_RED, 1);

			/* Code for pid using fifo, not working yet
			float val;
			poten_par.ctrl.adc_fifo_elem = Push_FIFO(&fifo_adc, &poten_par);
			if (!FIFO_IsEmpty(&fifo_dac)) {
				poten_par.ctrl.dac_fifo_elem = Pop_FIFO(&fifo_dac, &val);
				pid_par.target = val;
			}
			*/

			/* If STOP was signaled and stop PID control  */
			stp = (poten_par.ctrl.st & EPOT_ST_STOP);
		}
	else if (poten_par.ctrl.st & EPOT_ST_FIFO){
		float val;

		/* Push ADC read values to POTENTIOSTAT ADC FIFO */
		poten_par.ctrl.adc_fifo_elem = Push_FIFO(&fifo_adc, &poten_par);

		/* Pop DAC FIFO value and write to VCEIN DAC channel */
		poten_par.ctrl.dac_fifo_elem = Pop_FIFO(&fifo_dac, &val);
		if (poten_par.ctrl.dac_fifo_elem >= 0) {
			Set_DAC_Value(eDAC_VCEIN, val);
		} else {
			BlinkLed_Idx_Start(LED_RED, 1);

			/* If STOP was signaled and DAC FIFO is empty stop capture */
			stp = (poten_par.ctrl.st & EPOT_ST_STOP);
		}
	}

	/* If STOP was signaled and DAC FIFO is empty stop capture */
	if (stp) {
		poten_par.ctrl.st &= ~(EPOT_ST_RUN | EPOT_ST_STOP
				| EPOT_ST_FIFO | EPOT_ST_PID);
		SetCESwitch(0);
	}

	fifo_tm = *p_tm_us;
}

void Set_WE_RE_Zero(void)
{
	float x = dac_values[eDAC_OPVREF].set_val - an_values[eANCH_WEOUT].mean;
	an_values[eANCH_WEOUT].p_cfg->AN.Offset = x;

	an_values[eANCH_REOUT].p_cfg->AN.Offset -= an_values[eANCH_REOUT].mean;
}

void Analog_RunTime(void) {
	const uint32_t *p_tm_us = GetTick_us_hldr();
	static uint32_t delay_tm = 0;
	uint32_t f_adc, i, x;
	stAnalogData *p_an;

	/* If delay time is not zero and not reached return */
	if (TIMEDIFF(delay_tm, *p_tm_us) < sys_cfg.tm.adc_smp)
		return;

	/* Start ADC DMA sequential channel conversion */
	for (i = 0, f_adc = 0; i < ELEMEN_CNT(p_adc_ins); i++) {
		if (p_adc_ins[i]->DMA_Handle->State != HAL_DMA_STATE_BUSY) {
			uint16_t *p_adc = &adc_buf[i][0][0], *p_wrk = &adc_buf[i][1][0];

			f_adc |= (1 << i);
			/* Move ADC conversion buffer to working buffer
			 * and start ADC capture */
			memcpy(p_wrk, p_adc, sizeof(uint16_t) * adc_ch_qty[i]);

			HAL_ADC_Start_DMA((ADC_HandleTypeDef*) p_adc_ins[i],
					(uint32_t*) p_adc, adc_ch_qty[i]);
		}
	}

	/* Process ADC data on working buffer */
	for (i = 0; i < ELEMEN_CNT(p_adc_ins); i++) {
		if (!(f_adc & (1 << i)))
			continue;

		for (x = 0, p_an = an_values; x < eANCH_MAX; x++, p_an++) {
			if (p_an->p_chCfg->nADC != p_adc_ins[i])
				continue;

			p_an->analog_handle(p_an);

			/* Check if ADC WE, RE values are done. */
			if (CHECK_BITS((EPOT_ST_WE | EPOT_ST_RE), poten_par.ctrl.st)) {
				if (poten_par.ctrl.st & EPOT_ST_RUN)
					Process_Potentiostat_FIFO();
				poten_par.ctrl.st &= ~(EPOT_ST_WE | EPOT_ST_RE);
			}
		}
	}
	delay_tm = *p_tm_us;
}




