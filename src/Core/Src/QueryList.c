/*
 * QueryList.c
 *
 *  Created on: 28 sept. 2018
 *      Author: agarcia
 */

#include <stdint.h>
#include <time.h>
#include <stddef.h>
#include <string.h>
#include "user.h"


/* Variables added to query list mus be declared as external due to
	constant value is required*/

extern struct tm sys_dtime;
extern system_config sys_cfg;
extern stVersion FirmVersion;
extern system_config sys_cfg;
extern FIFO_ctrl fifo_adc, fifo_dac;
extern potentiostat_param poten_par;
extern PID_Param pid_par;

const uint8_t MasterPass[8] = "1357BKDR";

uint8_t fifo_tx_buf[256];
uint8_t fifo_rx_buf[256];

int gain_val = eTIA_SW_GAIN_100K;
float analog_reg[eANCH_MAX];
float dac_output[eDAC_MAX];

stModBusCmd mbCMDInput;
timer_task command_tmr;

/* Process functions definition */
int32_t ProcessDateTimeCfg(void *, uint16_t, uint16_t );
int32_t ProcessCommand(void *, uint16_t, uint16_t);
int32_t ProcessCfgPassLevel(void *, uint16_t, uint16_t);
int32_t ProcessCfgPassword(void *, uint16_t, uint16_t);
int32_t ProcessAnalogData(void *, uint16_t, uint16_t);
int32_t ProcessDacOut(void *, uint16_t, uint16_t);
int32_t ProcessTiaGain(void *, uint16_t, uint16_t);
int32_t ProcessADCFIFO(void *, uint16_t, uint16_t);
int32_t ProcessDACFIFO(void *, uint16_t, uint16_t);

const mbQuery rdCfg_List[]={
	{REG_ADC_FIFO,
			eREG_RD,
			eRD_HLD,
			sizeof(fifo_tx_buf),
			fifo_tx_buf,
			ProcessADCFIFO},
	{REG_DAC_FIFO,
			eREG_RD | eREG_WR,
			eRD_HLD,
			sizeof(fifo_rx_buf),
			fifo_rx_buf,
			ProcessDACFIFO},
	{REG_POT_VALUES,
			eREG_RD,
			eRD_HLD,
			sizeof(potentiostat_param),
			&poten_par,
			NULL},
	{REG_ANALOG,
			eREG_RD,
			eRD_HLD,
			sizeof(analog_reg),
			analog_reg,
			ProcessAnalogData},
	{REG_DAC_OUT,
			eREG_RD | eREG_WR,
			eRD_CFG | eWR_CFG,
			sizeof(dac_output),
			&dac_output,
			ProcessDacOut},
	{REG_PID,
			eREG_RD,
			eRD_CFG | eWR_CFG,
			offsetof(PID_Param, cfg),
			&pid_par,
			NULL},
	{REG_CFG_ANALOG_IN,
			eREG_RD | eREG_WR,
			eRD_CFG | eWR_CFG,
			sizeof(sys_cfg.p_ancfg),
			sys_cfg.p_ancfg, NULL},
	{REG_CFG_ANALOG_OUT,
			eREG_RD | eREG_WR,
			eRD_CFG | eWR_CFG,
			sizeof(sys_cfg.p_dac_cfg),
			sys_cfg.p_dac_cfg,
			NULL},
	{REG_CFG_PID,
			eREG_RD | eREG_WR,
			eRD_CFG | eWR_CFG,
			sizeof(PID_Config),
			&sys_cfg.pid_cfg,
			NULL},
	{REG_CFG_SYSTEM,
			eREG_RD | eREG_WR,
			eRD_CFG | eWR_CFG,
			(offsetof(system_config, password) - offsetof(system_config, tm)),
			&sys_cfg.tm,
			NULL},
	{REG_CFG_CHIPID,
			eREG_RD ,
			eRD_CFG,
			sizeof(system_config),
			sys_cfg.chip_serial.Serial8,
			NULL},
	{REG_CFG_FLASH_ST,
			eREG_RD ,
			eRD_CFG,
			(offsetof(system_config, checksum) - offsetof(system_config, dt)),
			&sys_cfg.dt,
			NULL},
	{REG_CFG_DATETIME,
			eREG_RD | eREG_WR,
			eRD_CFG | eWR_CFG,
			sizeof(struct tm),
			&sys_dtime,
			&ProcessDateTimeCfg},
	{REG_COMMAND,
			eREG_WR,
			eWR_CFG,
			sizeof(stModBusCmd),
			&mbCMDInput,
			&ProcessCommand},
	{REG_CFG_PASSWORD,
			eREG_RD  | eREG_WR,
			eRD_PASS | eWR_PASS,
			sizeof(stPassword),
			&sys_cfg.password,
			&ProcessCfgPassword},
	{REG_CFG_PASSLEVEL,
			eREG_RD  | eREG_WR,
			eRD_PASS | eWR_PASS,
			8,
			&sys_cfg.password.cfg_password,
			&ProcessCfgPassLevel},
};

uint32_t GetListSize(void)
{
	return (sizeof(rdCfg_List) / sizeof(mbQuery));
}

mbQuery *Get_CfgList(void)
{
	return (mbQuery *)rdCfg_List;
}

int32_t ProcessDateTimeCfg(void *pdata, uint16_t len, uint16_t acces_type)
{
	if(acces_type == eREG_WR)
	{
		SetDatetimeToInternalRTC(pdata);
	}

	return NO_ERROR;
}

static int32_t System_Reset(void *p_data)
{
	NVIC_SystemReset();

	return NO_ERROR;
}

float GetFloatFromBuffer(uint8_t *p_d)
{
	float val = 0;
	uint32_t *p_val = (uint32_t *)&val;

	for (uint32_t s = 0; s < 32; p_d++, s += 8)
		*p_val |= ((*p_d) <<s);

	return val;
}

int32_t ProcessCommand(void *pdata, uint16_t len, uint16_t acces_type)
{
	stModBusCmd *mb_cmd = pdata;
	uint32_t Mb_Error = NO_ERROR;

	if (acces_type == eREG_RD)
		return Mb_Error;

	switch (mb_cmd->cmd) {
	case CMD_SENSOR_ZERO:
		/* Put zero sensor adjust function call here */
		Set_WE_RE_Zero();
		break;
	case CMD_RESET:
		/* Delayed execution timer start */
		if (mb_cmd->param[0] != 0) {
			task_timer_set(&command_tmr, System_Reset, NULL, TASK_TIMER_MS(25));
			task_timer_start(&command_tmr, 1);
		}
		break;
	case CMD_LOADDFLT:
		LoadDataConfig(&sys_cfg, (eLOADMODE)mb_cmd->param[0]);
		break;
	case CMD_SET_TIA_GAIN:
		if (mb_cmd->param[0] > eTIA_SW_MAX)
			Mb_Error = ILLEGAL_DATA_VALUE;
		else
			Set_TIA_Gain((eTIA_GAIN)mb_cmd->param[0]);
		break;
	case CMD_SET_SWITCH:
		SetCESwitch(mb_cmd->param[0]);
		break;
	case CMD_FIFO_START:
		FIFO_Clear(&fifo_adc);
		poten_par.ctrl.st |= (EPOT_ST_RUN  | EPOT_ST_FIFO);
		SetCESwitch(mb_cmd->param[0]);
		break;
	case CMD_PID_START:
		{
			float t = GetFloatFromBuffer(mb_cmd->param);
			if (isfinite(t)) {
				pid_par.target = t;
				poten_par.ctrl.st |= (EPOT_ST_RUN  | EPOT_ST_FIFO | EPOT_ST_PID);
			};
		}
		break;
	case CMD_TEST_STOP:
		poten_par.ctrl.st |= EPOT_ST_STOP;
		break;
	case CMD_CLEAR_FIFO:
		if(mb_cmd->param[0] != 0)
			FIFO_Clear(&fifo_adc);
		break;
	case CMD_CFG_SAVE:
		task_timer_set(&command_tmr, SaveDataConfig, &sys_cfg,
				TASK_TIMER_MS(25));
		task_timer_start(&command_tmr, 1);
		break;
	default:
		Mb_Error = ILLEGAL_DATA_VALUE;
		break;
	}
	return Mb_Error;
}


int32_t ProcessCfgPassLevel(void *pdata, uint16_t len, uint16_t acces_type)
{
	int32_t Mb_Error = NO_ERROR;
	stPassword *pPwd = pdata;

	if (acces_type == eREG_RD)
		return Mb_Error;

	if (!memcmp(&pPwd->cfg_password, &sys_cfg.password.cfg_password, 8)) {
		/* Check user password on match set user access
		 * User maximum access is 	PASS_READ_HLD  |  PASS_WRITE_HLD
		 * | PASS_READ_CFG | PASS_WRITE_CFG |PASS_READ_SLAVES
		 * | PASS_WRITE_SLAVES */

		sys_cfg.password.Accesslevel.Reg = pPwd->Accesslevel.Reg & 0x0000003f;
	}
	else if (!memcmp(&pPwd->cfg_password, &MasterPass, 8)) {
		/* Check Master password on match allow set
		 * all levels access and configuration parameters */

		sys_cfg.password.Accesslevel.Reg = pPwd->Accesslevel.Reg;
		sys_cfg.password.MasterPass_tm = pPwd->MasterPass_tm;
		sys_cfg.password.Accesslevel.bits.Master_EN = 1;
	} else {
		Mb_Error = ILLEGAL_DATA_VALUE;
	}
	return Mb_Error;
}

int32_t ProcessCfgPassword(void *pdata, uint16_t len, uint16_t acces_type)
{
	int32_t Mb_Error = NO_ERROR;
	stPassword *pPwd = pdata;

	if (acces_type == eREG_RD)
		return Mb_Error;

	/* Set default password and access level */

	if (!memcmp(&pPwd->cfg_password, &MasterPass, 8)) {
		sys_cfg.password.l_access_reg = sys_cfg.password.Accesslevel.Reg;
		sys_cfg.password.Accesslevel.Reg =
				(PASS_READ_HLD | PASS_READ_CFG | PASS_WRITE_CFG);
		memcpy(sys_cfg.password.cfg_password, pPwd->cfg_password, 8);
		sys_cfg.password.master_st_tm = HAL_GetTick();
	} else {
		Mb_Error = ILLEGAL_DATA_VALUE;
	}
	return Mb_Error;
}

int32_t ProcessAnalogData(void *pdata, uint16_t len, uint16_t acces_type)
{
	uint32_t Mb_Error = NO_ERROR;

	if (acces_type == eREG_RD) {
		stAnalogData *p_an = GetAnalogData(0);
		for (uint32_t x = 0; x < eANCH_MAX; analog_reg[x] = p_an[x].mean, x++);
	}
	return Mb_Error;
}

int32_t ProcessDacOut(void *pdata, uint16_t len, uint16_t acces_type)
{
	uint32_t Mb_Error = NO_ERROR;

	if (acces_type == eREG_RD) {
		stDAC_Data *p_an = GetDACData();
		for (uint32_t x = 0; x < eDAC_MAX;
				dac_output[x] = p_an[x].set_val, x++);
	} else if (acces_type == eREG_WR) {
		for (uint32_t x = 0; x < eDAC_MAX; Set_DAC_Value(x,
				dac_output[x]), x++);
	}
	return Mb_Error;

}

int32_t ProcessTiaGain(void *pdata, uint16_t len, uint16_t acces_type)
{
	uint32_t Mb_Error = NO_ERROR;

	if (acces_type == eREG_WR)
	{
		if (gain_val > eTIA_SW_MAX)
		{
			Mb_Error = ILLEGAL_DATA_VALUE;
		} else {
			Set_TIA_Gain((eTIA_GAIN)gain_val);
		}
	}
	return Mb_Error;

}

int32_t ProcessADCFIFO(void *pdata, uint16_t len, uint16_t acces_type)
{
	uint32_t Mb_Error = NO_ERROR;

	if (acces_type == eREG_RD) {
		uint32_t n_fifo_regs = (len / fifo_adc.dataLen);
		if (fifo_adc.npend < n_fifo_regs)
			Mb_Error = SLAVE_DEVICE_BUSY;
		else
			Pop_N_FIFO(&fifo_adc, pdata, (len / fifo_adc.dataLen));
	}

	return Mb_Error;
}

int32_t ProcessDACFIFO(void *pdata, uint16_t len, uint16_t acces_type)
{
	uint32_t Mb_Error = NO_ERROR;

	if (acces_type == eREG_WR) {
		uint32_t n_fifo_regs = (len / fifo_dac.dataLen);
		if (GetFIFOFreeElements(&fifo_dac)< n_fifo_regs)
			Mb_Error = SLAVE_DEVICE_BUSY;
		else
			Push_N_FIFO(&fifo_dac, pdata, (len / fifo_dac.dataLen));
	}

	return Mb_Error;
}

void QueryListInit(void)
{
	task_timer_create(&command_tmr, NULL, NULL, TASK_TIMER_MS(25));
}



