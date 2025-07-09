/*
 * FileManager.c
 *
 *  Created on: Jan 22, 2021
 *      Author: angel
 */

#include "user.h"

const char *devcfg_path = "devcfg";
const char *devcgh_name = "dev_00.cfg";

int compare_cfg_index(const void *a, const void *b)
{
	find_element *p_a = (find_element *)a, *p_b = (find_element *)b;
	return (p_a->time - p_b->time);
}


/*! \fn int32_t config_filename_set_index(char *filename, uint32_t index)

	\brief Search two digit  index on filename and increase it

	\param[in] *filename void pointer filename
	\param[in] index filename present index

	\return		result FR_OK success otherwise FR_INVALID_NAME.
 */

int32_t config_filename_set_index(char *filename, uint32_t index)
{
	int32_t res = FR_INVALID_NAME;
	char *p_ix = strpbrk (filename, "0123456789");

	if(p_ix != NULL)
	{
		snprintf(p_ix, strlen(p_ix) + 1, "%02lu.cfg", index);
		res = FR_OK;
	}
	return res;
}

/* Find last saved configuration file based on saved time stamp
 * Input pointer to char for store result file name
 * Return -1 for error, number of configuration files if success
 */
int32_t config_file_get_idx(char *p_fname, int32_t index)
{
	int32_t fr;     /* Return value */
    DIR dj;         /* Directory object */
    FILINFO fno;    /* File information */
	find_element elem[MAX_CFG_FILES];

	/* Start to search for device config files */
    fr = f_findfirst(&dj, &fno, "devcfg", "dev_*.cfg");
    if(fr == FR_OK)
    {
    	char *p_ix;
    	uint32_t fx = 0;

    	while((fr == FR_OK) && fno.fname[0] && (fx < MAX_CFG_FILES))
    	{
    		p_ix = strpbrk (fno.fname, "0123456789");
    		if(p_ix != NULL)
    		{
    			elem[fx].idx = strtoul(p_ix, NULL, 10);
    			elem[fx].time = (uint32_t)((fno.fdate << 16) | fno.ftime);
    		}

    		fr = f_findnext(&dj, &fno);
    		++fx;
    	}


    	if((index == FIND_IDX_LAST) || (index >= MAX_CFG_FILES)){
    		if(fx != 0){
				qsort (elem, fx, sizeof (find_element), compare_cfg_index);
				index = elem[0].idx;
    		} else {
    			index = 0;
    		}

    	}

    	strcpy(p_fname, devcgh_name);
    	fr = (config_filename_set_index(p_fname, index) == FR_OK) ? fx : -1;
    }
    else
    {
    	fr = -1;
    }

    f_closedir(&dj);

    return fr;
}


int32_t config_file_rw(void *p_buf, uint32_t len, uint32_t mode)
{
	FIL devcfg;
    int32_t fr;     /* Return value */
   	uint32_t option = (mode == FA_WRITE) ? FA_CREATE_ALWAYS : FA_OPEN_EXISTING;
	char c_name[16] = {0};
   	char f_name[64] = {0};

   	fr = config_file_get_idx(c_name, FIND_IDX_LAST);
    if(fr < 0)
    {
    	/* If directory or file doesn't exist and mode is for writing create it */
    	if(mode == FA_WRITE)
    		fr = f_mkdir (devcfg_path);
    }
    else
    {

    	if(mode == FA_WRITE)
    	{
    		char *p_ix = strpbrk (c_name, "0123456789");
    		uint32_t cfg_idx = strtoul(p_ix, NULL, 10) + 1;

    		if(cfg_idx >= MAX_CFG_FILES)
    			cfg_idx = 0;

    		config_filename_set_index(c_name, cfg_idx);
    	}
    }

    snprintf(f_name, sizeof(f_name),"%s%s%s", devcfg_path, "/", c_name);

	fr = f_open(&devcfg , f_name, (option | mode));
	if(fr == FR_OK)
	{
		uint32_t rw_bytes;
		if(mode == FA_WRITE)
			fr = f_write(&devcfg, p_buf, len, (UINT *)&rw_bytes);
		else
			fr = f_read(&devcfg, p_buf, len, (UINT *)&rw_bytes);

		f_close(&devcfg);
	}
    return fr;
}
