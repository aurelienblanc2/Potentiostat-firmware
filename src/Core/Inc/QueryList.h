/*
 * QueryList.h
 *
 *  Created on: 28 sept. 2018
 *      Author: agarcia
 */

#ifndef QUERYLIST_H_
#define QUERYLIST_H_

typedef enum _eREG_ACCESS
{
	eNOACCESS = 0,
	eRD_HLD = 1,
	eWR_HLD = (1<<1),
	eRD_CFG = (1<<2),
	eWR_CFG = (1<<3),
	eRD_SYS = (1<<4),
	eWR_SYS = (1<<5),
	eRD_PASS = (1<<8),
	eWR_PASS = (1<<9),
	eALLACCESS = 0xffff
}eREG_ACCESS;

typedef enum _ePASS_LEVEL
{
	PASS_READ_HLD = eRD_HLD,
	PASS_WRITE_HLD = eWR_HLD,
	PASS_READ_CFG = eRD_CFG,
	PASS_WRITE_CFG = eWR_CFG,
	PASS_READ_SYS = eRD_SYS,
	PASS_WRITE_SYS = eWR_SYS,
	PASS_WRITE_PASSWORD = eRD_PASS,
	PASS_READ_PASSWORD = eWR_PASS,
	PASS_NONE = eALLACCESS,
}ePASS_LEVEL;

enum
{
	CMD_SENSOR_ZERO = 0xE0,
	CMD_RESET,
	CMD_LOADDFLT,
	CMD_SET_TIA_GAIN,
	CMD_SET_SWITCH,
	CMD_FIFO_START,
	CMD_PID_START,
	CMD_TEST_STOP,
	CMD_CLEAR_FIFO,
	CMD_CFG_SAVE = 0xF0
};

enum
{
	REG_ADC_FIFO = 0x01,
	REG_DAC_FIFO,
	REG_POT_VALUES,
	REG_ANALOG,
	REG_DAC_OUT,
	REG_PID,
	REG_CFG_ANALOG_IN = 0x30,
	REG_CFG_ANALOG_OUT,
	REG_CFG_PID,
	REG_CFG_SYSTEM = 0x40,
	REG_CFG_CHIPID = 0x43,
	REG_CFG_FLASH_ST,
	REG_CFG_DATETIME = 0x45,
	REG_COMMAND = 0x4F,
	REG_MT_PID_PAR = 0x60,
	REG_MAX =0xFF
};

/* Boot loader operation registers address */
enum
{
	REG_FLASH_SIZE = 0xF0,
	REG_APP_START,
	REG_WR_ADDRESS,
	REG_RD_ADDRESS,
	REG_FIRM_VER,
	REG_FIRM_TIMESTAMP,
	REG_FIRM_CHECKSUM,
	REG_READ_FLASH,
	REG_PROGRAM_PAGE,
	REG_ERASE_ALL,
	REG_RESET_MCU,
	REG_CHECK_MEM,
	REG_FPAGE_SIZE,
	REG_FLASH_ALIGN
};

enum
{
	REG_CFG_PASSWORD = 0x50,
	REG_CFG_PASSLEVEL,
	REG_BACKDOOR = 0x5F
};


enum
{
	eREG_RD = 1,
	eREG_WR = 2
};

typedef struct _mbQuery
{
	uint16_t group;		// Address group
	uint16_t regdir;	// Direction read / write
	uint16_t passlevel;	// Allowed pass level
	uint16_t length;	// Maximum Number of bytes to read
	void *pData;		// Pointer to data buffer
	int32_t (*pfunc)(void *, uint16_t, uint16_t);	// function to process additional operations
}mbQuery;

mbQuery *Get_CfgList(void);
uint32_t GetListSize(void);
void QueryListInit(void);

#endif /* QUERYLIST_H_ */
