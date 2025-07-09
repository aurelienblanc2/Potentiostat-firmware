/*
 * NVM.h
 *
 *  Created on: 10 feb. 2020
 *      Author: agarcia
 */

#ifndef SRC_NVM_H_
#define SRC_NVM_H_

typedef enum _eLOADMODE
{
	LOAD_NONE = 0,
	LOAD_DEFAULT  = (1 <<0),
	LOAD_FROM_NVM = (1 <<1),	// Load configuration data from
	LOAD_FROM_FILE = (1 <<2),
	LOAD_IFDEF_SAVE = (1 <<3),
}eLOADMODE;

typedef struct
{
	uint32_t file_rPointer;		// current  read address
	uint32_t file_wPointer;		// current write address
	uint32_t flash_size;		// current flash size
	uint32_t app_start;			// Application start address
	uint16_t page_size;			// Flash page size
	uint16_t alignment;			// Data alignment in bytes
	uint32_t crc32;				// Computed CRC32 memory checksum
	uint8_t buffer[2 * MODBUS_RTU_MAX_REGISTERS];	// Page data buffer
}stFlashPGM;

stFlashPGM *fGetFlashInst(void);

int32_t SaveDataConfig(void *);
uint32_t LoadDataConfig(void *, eLOADMODE);
void NVM_Init(void);


#endif /* SRC_NVM_H_ */
