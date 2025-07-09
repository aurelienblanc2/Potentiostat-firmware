/*
 * FileManager.h
 *
 *  Created on: Jan 22, 2021
 *      Author: angel
 */

#ifndef INC_FILEMANAGER_H_
#define INC_FILEMANAGER_H_

#define MAX_CFG_FILES	20

#define FIND_IDX_FIRST	0
#define FIND_IDX_LAST 	-1
#define CONFIG_FILENAME	"tracker_00"

typedef struct _find_element
{
	uint32_t idx;
	uint32_t time;
}find_element;

int32_t config_file_rw(void *p_buf, uint32_t len, uint32_t mode);

#endif /* INC_FILEMANAGER_H_ */
