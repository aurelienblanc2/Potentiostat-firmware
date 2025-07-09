/*
 * NVM.c
 *
 *  Created on: 10 feb. 2020
 *      Author: agarcia
 */

#include "user.h"

stFlashPGM FirmFile_Ctrl = { 0 };

stFlashPGM* fGetFlashInst(void)
{
	return &FirmFile_Ctrl;
}

uint32_t NVM_GetAddress(void)
{
	return (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE);
}

/* Load data from flash memory or restore it to default values 	*/
/*	dflt = 0 Load from flash, dflt = 1 load default values 		*/

uint32_t LoadDataConfig(void *p_cfg, eLOADMODE mode)
{
	uint32_t icrc, result = FR_OK;
	system_config *pConfig = p_cfg;
	uint32_t NVM_Address = NVM_GetAddress();
	int32_t bsave = 0;

	/* if mode bit LOAD_FROM_FILE try to load file from FAT file system
	 * and clear LOAD_FROM_NVM bit */
	if (mode & LOAD_FROM_FILE) {
		result = config_file_rw(pConfig, sizeof(system_config), FA_READ);
		mode &= ~LOAD_FROM_NVM;
	}

	/*If mode bit LOAD_FROM_FILE is enabled and operation fails
	 * or mode bit LOAD_FROM_NVM is enabled load from FLASH */
	if ((result != FR_OK) || (mode & LOAD_FROM_NVM))
		memcpy((void*) pConfig, (void*) NVM_Address, sizeof(system_config));

	icrc = CRC32_compute((uint8_t*) pConfig, offsetof(system_config, checksum));

	/*if (!(mode & LOAD_DEFAULT) && (pConfig->checksum == icrc)) {
		memcpy(pConfig->chip_serial.Serial8, GetChipSerial(), 16);
		result = FR_OK;
	} else {
		// Load default configuration

		LoadSystemConfigDefaults();
		bsave = (mode & LOAD_IFDEF_SAVE);
	}
	Taking out to avoid conf from NVM to be loaded every time*/
	LoadSystemConfigDefaults();
	bsave = (mode & LOAD_IFDEF_SAVE);
	if (bsave) {
		result = SaveDataConfig(pConfig);
	}

	return result;
}

HAL_StatusTypeDef WriteNVM_Data(void *data, uint32_t length)
{
	uint32_t NVM_Address = NVM_GetAddress();
	HAL_StatusTypeDef status = HAL_ERROR;

	HAL_FLASH_Unlock();

	FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_FSTPG | FLASH_CR_MER1);
	FLASH_PageErase(FLASH_PAGE_NB - 1, FLASH_BANK_BOTH);

	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

	if (status == HAL_OK) {
		uint32_t addr = NVM_Address, i = 0;
		uint64_t f_data;

		FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_MER1);

		while (length) {
			if (length < 8) {
				f_data = 0;
				memcpy(&f_data, (data + i), length);
				length = 0;
			} else {
				memcpy(&f_data, (data + i), 8);
				length -= 8;
			}

			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i,
					f_data);

			if (status != HAL_OK)
				break;

			i += 8;
		}
	}
	HAL_FLASH_Lock();
	return status;
}

int32_t SaveDataConfig(void *p_cfg)
{
	system_config *config = p_cfg;
	struct tm *date_time = GetSysDateTime();

	config->n_flash_writes++;
	StructTm_To_PackDt(&config->dt, date_time);
	config->checksum = CRC32_compute((uint8_t*) config,
			offsetof(system_config, checksum));

	/* Save data to file and NVM to have a copy */

	config_file_rw(config, sizeof(system_config), FA_WRITE);
	return WriteNVM_Data(config, sizeof(system_config));
}

void NVM_Init(void)
{
	FirmFile_Ctrl.file_rPointer = 0;
	FirmFile_Ctrl.file_wPointer = 0;
	FirmFile_Ctrl.flash_size = (FLASH_SIZE - FLASH_PAGE_SIZE);
	FirmFile_Ctrl.app_start = 0;	// Application start on disk file is 0
	FirmFile_Ctrl.page_size = FLASH_PAGE_SIZE;
	FirmFile_Ctrl.alignment = 8;
}

