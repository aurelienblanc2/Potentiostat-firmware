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
extern EIS_Param eis_par;
extern EIS_Exp eis_exp;
extern EIS_Buffer eis_buf;
extern EIS_Processed eis_pro;

/* Function prototypes */
static void ProcessAnalog_VREF(void*);
static void ProcessAnalog(void*);
static void ProcessWEOUT(void*);
static void ProcessREOUT(void*);
static void ProcessDacValue(void*);

/* Declare 2 buffer one for DMA ping-pong operation and another
 * for process data purposes 3 adc's 2 buffers and 3 channels for each adc */

uint16_t adc_buf[3][2][3] __attribute__ ((aligned (8)));
uint32_t adc_tm_buf[3][2];

/* Calibrated VREFINT value at 30ºC */

float verfint_cal;
float k_adc = KADC16;

const ADC_HandleTypeDef *p_adc_ins[3] = { &hadc1, &hadc3, &hadc5 };
const uint32_t adc_ch_qty[3] = { ADC1_N_CH, ADC3_N_CH, ADC5_N_CH };
const uint32_t adc_tm_qty[3] = { ADC1_TM, ADC3_TM, ADC5_TM };

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
				ProcessWEOUT,
				(stADCChn*) &adc_ch_cfg[eANCH_WEOUT],
				&sys_cfg.p_ancfg[eANCH_WEOUT] },
		{ eANCH_REOUT,
				ProcessREOUT,
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
float eis_time_measured_potential = 0;
float eis_time_measured_intensity = 0;
float eis_first_time_measured = 0;
float sine_table[WAVE_TABLE_SIZE];

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

	if (!(poten_par.ctrl.st & EPOT_ST_EIS))
	{
	    p_an->mean = CUMULATIVE_AVERAGE(p_an->current, p_an->mean, p_cfg->samples);
	}
	else
	{
	    p_an->mean = p_an->current;
	}
}

/* TIA current measurement formulas
 * VWE = TOAref - (GX * I), TOAref is set by DAC2
 * WEout = OPref - (GI * VWE),
 * OPref is set by DAC3 = (VDD / 2) = 1.65V, GI = (1 / 3)
 * WEout = OPref - (GI * (TOAref - (GX * I)), replacing VWE
 * I = (GI * TOAref - OPref + WEout)/(GI * GX), cleaning I
 */

static void ProcessWEOUT(void *p_val) {
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
	
	eis_time_measured_intensity = adc_tm_buf[0][1];
}

static void ProcessREOUT(void *p_val) {
	stAnalogData *p_an = (stAnalogData*) p_val;

	ProcessAnalog(p_val);
	poten_par.adc.reout = p_an->mean;
	poten_par.ctrl.st |= EPOT_ST_RE;
	
	eis_time_measured_potential = adc_tm_buf[1][1];
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

    /* Initialize Sin table for EIS Wave Generation */
	Init_WaveTable();
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

	fifo_tm = *p_tm_us;

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
		
		if (!(poten_par.ctrl.st & EPOT_ST_EIS))
		{
			/* Push ADC read values to POTENTIOSTAT ADC FIFO */
			poten_par.ctrl.adc_fifo_elem = Push_FIFO(&fifo_adc, &poten_par);

			/* Pop DAC FIFO value and write to VCEIN DAC channel */
			float val;
			poten_par.ctrl.dac_fifo_elem = Pop_FIFO(&fifo_dac, &val);
			if (poten_par.ctrl.dac_fifo_elem >= 0) {
				Set_DAC_Value(eDAC_VCEIN, val);
			} else {
				BlinkLed_Idx_Start(LED_RED, 1);

				/* If STOP was signaled and DAC FIFO is empty stop capture */
				stp = (poten_par.ctrl.st & EPOT_ST_STOP);
			}
		}
		else
		{
			if (eis_exp.flag == false)
			{
				eis_exp.flag = true;
				eis_exp.current_time = 0;
				eis_buf.idx = 0;
				return;
			}

			// TODO: End frequency can be not executed depending of the start freq and the point per decade => need to correct that
            if (eis_exp.current_freq >= eis_par.end_freq)
            {
                if(eis_exp.current_time > (eis_exp.number_cycle+EIS_DISCARD_CYCLE)/eis_exp.current_freq)
                {
                    // Processing and sending result
                    Process_Impedance();

                    // Init for next frequency
                    eis_exp.current_freq /= eis_par.step_factor;

                    // Optimal sampling calculation
                    uint32_t sampling = (EIS_NUMBER_CYCLE/eis_exp.current_freq*1000.*1000.) / (EIS_BUFFER_SIZE/2);

                    // Cycle number calculation
			        if (POTCTRL_POLLING_TIME_EIS > sampling)
			      	{
			      		sampling = POTCTRL_POLLING_TIME_EIS;
			      		eis_exp.number_cycle = ((uint8_t) (EIS_BUFFER_SIZE*sampling/1000./1000.*eis_exp.current_freq))-1;
			      	}
			      	else
			      	{
			      		eis_exp.number_cycle = EIS_NUMBER_CYCLE;
			      	}

                    // Sampling calculation
			        uint32_t base = POTCTRL_POLLING_TIME_EIS;
					uint32_t multiple = sampling / base;
			        sys_cfg.tm.fifo_smp = multiple * base;

                	eis_exp.flag = false;
                }
                else
                {
                    float val = Generate_Wavefront();
                    Set_DAC_Value(eDAC_VCEIN, val);

                    // Remove some cycle here to avoid transition effect
                    if(eis_exp.current_time >= EIS_DISCARD_CYCLE/eis_exp.current_freq)
                    {
                        // TODO : Protect reset of the clock
                        float tpot = eis_time_measured_potential / 1000000.0f; // Convert to second
                        float tint = eis_time_measured_intensity / 1000000.0f; // Convert to second

                        if (eis_buf.idx == 0)
                        {
                        	eis_first_time_measured = MIN(tpot, tint);
                        }

                        if (eis_buf.idx < EIS_BUFFER_SIZE)
                        {
	                        eis_buf.time_measured_potential[eis_buf.idx] = tpot - eis_first_time_measured;
	                        eis_buf.time_measured_intensity[eis_buf.idx] = tint - eis_first_time_measured;
	                        eis_buf.potential_measured[eis_buf.idx] = poten_par.adc.reout;
	                        eis_buf.intensity_measured[eis_buf.idx] = poten_par.adc.weout;
	                        eis_buf.idx += 1;
                    	}
                    }

                    // Updating time
                    eis_exp.current_time += dt/ 1000000.0f;
                }
            }
            else
            {
                eis_pro.frequency = eis_exp.current_freq;
                eis_pro.impedance_re = 0;
                eis_pro.impedance_im = 0;
                poten_par.ctrl.adc_fifo_elem = Push_FIFO(&fifo_adc, &(eis_pro));

                /* If STOP was signaled and DAC FIFO is empty stop capture */
                stp = (poten_par.ctrl.st & EPOT_ST_STOP);
            }
		}
	}

	/* If STOP was signaled and DAC FIFO is empty stop capture */
	if (stp) {
		poten_par.ctrl.st &= ~(EPOT_ST_RUN | EPOT_ST_STOP
				| EPOT_ST_FIFO | EPOT_ST_PID);
		SetCESwitch(0);
	}
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
	uint32_t f_adc, i, x, nb_adc, nb_chan;
	stAnalogData *p_an;

	/* If delay time is not zero and not reached return */
	if (TIMEDIFF(delay_tm, *p_tm_us) < sys_cfg.tm.adc_smp)
		return;

	delay_tm = *p_tm_us;

	if (!(poten_par.ctrl.st & EPOT_ST_EIS))
	{
	    nb_adc = ELEMEN_CNT(p_adc_ins);
	    nb_chan = eANCH_MAX;
	}
	else
	{
	    // TODO : Cleaner way
	    nb_adc = 2; // Only hadc1 and hadc2
	    nb_chan = 2;
	}

	/* Start ADC DMA sequential channel conversion */
	for (i = 0, f_adc = 0; i < nb_adc; i++) {
		if (p_adc_ins[i]->DMA_Handle->State != HAL_DMA_STATE_BUSY) {
			uint16_t *p_adc = &adc_buf[i][0][0], *p_wrk = &adc_buf[i][1][0];
			uint32_t *p_adc_tm = &adc_tm_buf[i][0], *p_wrk_tm = &adc_tm_buf[i][1];

			f_adc |= (1 << i);
			/* Move ADC conversion buffer to working buffer
			 * and start ADC capture */
			memcpy(p_wrk, p_adc, sizeof(uint16_t) * adc_ch_qty[i]);
			memcpy(p_wrk_tm, p_adc_tm, sizeof(uint32_t) * adc_tm_qty[i]);

			HAL_ADC_Start_DMA((ADC_HandleTypeDef*) p_adc_ins[i],
					(uint32_t*) p_adc, adc_ch_qty[i]);
		}
	}

	/* Process ADC data on working buffer */
	for (i = 0; i < nb_adc; i++) {
		if (!(f_adc & (1 << i)))
			continue;

		for (x = 0, p_an = an_values; x < nb_chan; x++, p_an++) {
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
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    uint32_t ts = *GetTick_us_hldr(); // microsecond timestamp

    if (hadc == &hadc1) {
        adc_tm_buf[0][0] = ts;
    } else if (hadc == &hadc3) {
        adc_tm_buf[1][0] = ts;
    }
}

float Generate_Wavefront(void) {
    float phase = eis_exp.current_freq * eis_exp.current_time;
    phase -= (int)phase; 

    // lookup sine
    uint32_t idx = (uint32_t)(phase * WAVE_TABLE_SIZE) & (WAVE_TABLE_SIZE - 1);
    float s = sine_table[idx];

    // generate command signal
    float pot = eis_par.dc_potential + eis_par.perturbation_potential * s;
    return pot;
}

void Process_Impedance(void){

    cplx Vph = compute_phasor_timestamped(eis_buf.potential_measured, eis_buf.time_measured_potential, eis_buf.idx, eis_exp.current_freq);
    cplx Iph = compute_phasor_timestamped(eis_buf.intensity_measured, eis_buf.time_measured_intensity, eis_buf.idx, eis_exp.current_freq);

    cplx Z = cplx_div(Vph, Iph);

    eis_pro.frequency = eis_exp.current_freq;
    eis_pro.impedance_re = (float) Z.re;
    eis_pro.impedance_im = (float) Z.im;

    poten_par.ctrl.adc_fifo_elem = Push_FIFO(&fifo_adc, &(eis_pro));
}

cplx compute_phasor_timestamped(const float *x, const float *t, uint16_t N, float freq) {
    double omega = 2.0 * M_PI * freq;
    double acc_re = 0.0;
    double acc_im = 0.0;
    double mean = 0.0;

    for (uint16_t n = 0; n < N; ++n) mean += x[n];
    mean /= (double)N;

	for (uint16_t n = 1; n < N; ++n) {
        double dt = t[n] - t[n - 1];
        // Safety check for non-monotonic timestamps
        if (dt <= 0.0) continue;

        // Hann window weights
        double w1 = 0.5 * (1.0 - cos(2.0 * M_PI * (n - 1) / (N - 1)));
        double w2 = 0.5 * (1.0 - cos(2.0 * M_PI * n / (N - 1)));

        // Average window and signal values (trapezoid)
        double xn = 0.5 * ((x[n - 1] - mean) * w1 + (x[n] - mean) * w2);

        // Midpoint time for phase
        double tm = 0.5 * (t[n] + t[n - 1]);
        double ang = omega * tm;

        acc_re += xn * cos(ang) * dt;
        acc_im -= xn * sin(ang) * dt;
    }

    // Normalize by total time span and Hann coherent gain
    double Tspan = t[N - 1] - t[0];
    // Avoid division by zero
    if (Tspan <= 0.0) Tspan = 1.0;

    // Hann coherent gain ≈ 0.5 (average amplitude reduction)
    double hann_gain = 0.5;

    // Scale with time-based normalization
    double scale = (2.0 / (Tspan * hann_gain));
    acc_re *= scale;
    acc_im *= scale;

    cplx out = { acc_re, acc_im };
    return out;
}

cplx cplx_div(cplx a, cplx b) {
    cplx res;
    double denom = b.re*b.re + b.im*b.im;
    if (denom == 0.0) {
        res.re = res.im = NAN; // handle singular case
        return res;
    }
    res.re = (a.re*b.re + a.im*b.im) / denom;
    res.im = (a.im*b.re - a.re*b.im) / denom;
    return res;
}

void Init_WaveTable(void) {
    for (uint32_t i = 0; i < WAVE_TABLE_SIZE; i++) {
        sine_table[i] = sinf(2.0f * M_PI * (float)i / (float)WAVE_TABLE_SIZE);
    }
}