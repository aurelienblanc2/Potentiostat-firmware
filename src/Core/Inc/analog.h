/*
 * Analog.h
 *
 *  Created on: 22 nov. 2017
 *      Author: agarcia
 */

#ifndef ANALOG_H_
#define ANALOG_H_

#define TIA_SW_PORT 	GPIOB
#define TIA_SW_SHIFT	9
#define TIA_SW_MASK	 	(0xf << TIA_SW_SHIFT)

#define SW_GAIN_1K		(0x01 << TIA_SW_SHIFT)
#define SW_GAIN_10K		(0x02 << TIA_SW_SHIFT)
#define SW_GAIN_100K	(0x04 << TIA_SW_SHIFT)
#define SW_GAIN_1M		(0x08 << TIA_SW_SHIFT)
#define SW_GAIN_10M		(0x00 << TIA_SW_SHIFT)

/* Gain for trans-impedance amplifier is 10M fixed resistor
 * in parallel with switched resistors : 1K, 10K, 100K, 1M */


#define TIA_GAIN_1K		999.900009
#define TIA_GAIN_10K	9990.00999
#define TIA_GAIN_100K	99009.90099
#define TIA_GAIN_1M		909090.9091
#define TIA_GAIN_10M	1e7


#define CONVERT_TIMEOUT	100

#define ADCVREF		3.3

#define ADC12_CNTS	4095.0
#define KADC12		(ADCVREF / ADC12_CNTS)

#define ADC16_CNTS	65535.0
#define KADC16		(ADCVREF / ADC16_CNTS)

#define DAC12_CNTS	ADC12_CNTS
#define KDAC12		(1 / KADC12)

#define V_SUPPLY	3.3
#define HALF_V_SUPPLY	(V_SUPPLY / 2.0)

#define VCEIN_GAIN 		(-1.0/ 3.0)
#define VCEIN_OFFSET	HALF_V_SUPPLY

#define VREOUT_GAIN 	(-3.0)
#define VREOUT_OFFSET	(-VREOUT_GAIN * HALF_V_SUPPLY)

/* Number of channels to sample for each ADC */

#define ADC1_N_CH	1
#define ADC3_N_CH	1
#define ADC5_N_CH	3

#define INV_WEO_GAIN 	3.0	// (1 / WEO_GAIN)

#define ADC_CH_POLLING_TIME	50		// ADC all channels sampling time us.
#define POTCTRL_POLLING_TIME 200	// POTENTIOSTAT controller sampling time us.
#define ADC_FIFO_CNT	4096
#define DAC_FIFO_CNT	4096

#define SMP_MAVG_SH	6				// ADC moving average rate exponent 2^4 = 16
#define SMP_MAVG (1 <<SMP_MAVG_SH)	// Number of moving average samples

/* ADC channel enumerations */

enum eADC1_CHANNELS
{
	eADC1_WEOUT = ADC_CHANNEL_2,
};

enum eADC3_CHANNELS
{
	eADC3_REOUT = ADC_CHANNEL_12,
};

enum eADC5_CHANNELS
{
	eADC5_VREFINT = ADC_CHANNEL_VREFINT,
	eADC5_MCU_TEMP = ADC_CHANNEL_TEMPSENSOR_ADC5,
	eADC5_MCU_VBAT = ADC_CHANNEL_VBAT,
};

/* Analog channel enumeration such as it will
 * be arranged in the configuration array.*/

typedef enum _eAN_CHANNELS
{
	eANCH_WEOUT = 0,
	eANCH_REOUT,
	eANCH_VREFINT,
	eANCH_MCU_TEMP,
	eANCH_MCU_VBAT,
	eANCH_MAX
}eAN_CHANNELS;

typedef enum _eDAC_CHANNELS
{
	eDAC_VCEIN = 0,
	eDAC_TOAREF,
	eDAC_OPVREF,
	eDAC_MAX
}eDAC_CHANNELS;


/* Analog input structure definitions */

typedef struct tag_st_ADCChn
{
	void *nADC;				// Basse address of ADC module
	uint32_t nChannel;		// Number of channel
	uint16_t *val;			// pointer to raw adc value
}stADCChn;

/* Analog input scale, offset and mean configuration structure */
typedef struct tag_stAnCfg
{
	struct
	{
		float Slope;
		float Offset;
	}AN;
	float samples;
}stAnCfg;

/* Analog input working data structure */
typedef struct tag_stAnalogData
{
	eAN_CHANNELS id;
	const void (*analog_handle)(void *);	// function to process data
	const stADCChn *p_chCfg;				// pointer to ADC channel configuration
	stAnCfg *p_cfg;							// Pointer to analog configuration struct
	float current;							// Instant value
	float mean;								// Processed averaged value
}stAnalogData;

/* DAC channel structure definition */

typedef struct tag_st_DAC_Chn
{
	void *nDAC;				// Basse address of ADC module
	uint32_t nChannel;		// Number of channel
}st_DAC_Chn;

/* DAC scale, offset and mean configuration structure */
typedef struct tag_stDAC_Cfg
{
	float Slope;
	float Offset;
	float value;	// Initial DAC value
}stDAC_Cfg;

/* DAC working data structure */
typedef struct tag_stDAC_Data
{
	eDAC_CHANNELS id;
	const void (*dac_handle)(void *);	// function to process data
	const st_DAC_Chn *p_chCfg;			// pointer to DAC configuration struct
	stDAC_Cfg *p_cfg;					// Pointer to analog configuration struct
	float set_val;						// Present set value
}stDAC_Data;

typedef enum _eTIA_SW_GAIN
{
	eTIA_SW_GAIN_1K = 0,
	eTIA_SW_GAIN_10K,
	eTIA_SW_GAIN_100K,
	eTIA_SW_GAIN_1M,
	eTIA_SW_GAIN_10M,
	eTIA_SW_MAX
}eTIA_GAIN;

typedef struct _tia_switch
{
	uint32_t value;
	float f_gain;
}tia_switch;

enum _EPOT_ST
{
	EPOT_ST_NONE = 0,
	EPOT_ST_WE ,
	EPOT_ST_RE = (1 <<1),
	EPOT_ST_FIFO = (1 <<2),
	EPOT_ST_PID = (1 <<3),
	EPOT_ST_RUN = (1 <<4),
	EPOT_ST_STOP = (1 <<5),
	EPOT_ST_CE = (1 <<8)
};

typedef struct _poten_group_adc
{
	float weout;		// Present WEOUT value
	float reout;		// Present REOUT value
}poten_group_adc;

typedef struct _poten_group_dac
{
	float vcein;		// DAC output VCEin
	float toaref;		// DAC output TOAref
	float opvref;		// DAC output OPvref2
}poten_group_dac;

typedef struct _poten_group_ht
{
	float extemp;		// Temperature from SHTC3 sensor
	float humidity;		// Humidity from SHTC3 sensor
}poten_group_ht;

typedef struct _poten_group_ctrl
{
	float gx;					// TIA present setting gain
	int32_t adc_fifo_elem;		// ADC pending data
	int32_t dac_fifo_elem;		// DAC pending data
	uint32_t samp_tm;			// POTENTIOSTAT sample period
	union
	{
		struct
		{
			uint32_t we_rdy :1;		// we value ready.
			uint32_t re_rdy :1;		// re value ready.
			uint32_t fifo_mod :1;	// Run in FIFO mode.
			uint32_t pid_mod :1;	// Run in PID current control mode.
			uint32_t run : 1;		// RUn mode status.
			uint32_t :3;
			uint32_t ce_sw:1;		// CE switch status.
			uint32_t :24;
		};
		uint32_t st;				// full state register
	};
}poten_group_ctrl;

typedef struct _potentiostat_param
{
	poten_group_adc adc;
	poten_group_dac dac;
	poten_group_ht ht;
	poten_group_ctrl ctrl;
}potentiostat_param;

/*typedef struct _potentiostat_param
{
	float weout;		// Present WEOUT value
	float reout;		// Present REOUT value
	float gx;			// TIA present setting gain
	float vcein;		// DAC output VCEin
	float toaref;		// DAC output TOAref
	float opvref;		// DAC output OPvref2
	float extemp;		// Temperature from SHTC3 sensor
	float humidity;		// Humidity from SHTC3 sensor
	uint32_t samp_tm;	// POTENTIOSTAT sample period
	eADC_FIFO_ST st;	// Updating values state
}potentiostat_param;*/

potentiostat_param *GetPotentiostatParam(void);
stADCChn *GetADC_Channels(void);
stAnCfg* GetAnalog_cfg_dflt(void);
stAnCfg* GetAnalog_cfg(void);
stAnalogData *GetAnalogData(uint32_t);
stDAC_Cfg *GetDAC_Cfg_Dflt(void);
stDAC_Data *GetDACData(void);

void Analog_Init(stAnCfg *);
void Analog_DeInit(void);
void DAC_Init(stDAC_Cfg *);
void Set_DAC_Value(eDAC_CHANNELS, float);
void SetCESwitch(uint32_t val);
float Set_TIA_Gain(eTIA_GAIN);
void Set_WE_RE_Zero(void);
void Analog_RunTime(void);
#endif /* ANALOG_H_ */
