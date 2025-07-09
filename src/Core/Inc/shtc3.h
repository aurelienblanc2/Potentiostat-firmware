//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHTC3 Sample Code (V1.0)
// File      :  shtc3.h (V1.0)
// Author    :  RFU
// Date      :  24-Nov-2017
// Controller:  STM32F100RB
// IDE       :  �Vision V5.17.0.0
// Compiler  :  Armcc
// Brief     :  Sensor Layer: Definitions of commands and functions for sensor
//                            access.
//==============================================================================

#include <stdint.h>
#ifndef SHTC3_H
#define SHTC3_H

#define CRC_POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
#define SHRC3_ADDR	0x70	// I2C slave device address
#define SHTC3_MAX_RETRIES	3	 // max. retries to read the measurement (polling)
#define ID_MASK	0x083F
#define ID_VALUE 0x0807

#define SHRC3_WAKEUP_TIME	1	/* Wait 1ms for reading wakeup from sleep. */
#define SHRC3_CAPTURE_TIME	12	/* Wait 10ms for reading Humidity value */
#define SHTC3_READ_RESULT_LEN	6	/* Number of result bytes to read from I2C */
#define SHTC3_T_SLOPE (175.0 / 65536.0)
#define SHTC3_T_OFFSET -45.0

#define SHTC3_H_SLOPE (100.0 / 65536.0)

#define SHTC3_GET_DATA_FROM_BUFFER(p)	((uint16_t)((*p << 8) | *(p + 1)))
#define SHTC3_CALC_TEMPERATURE(x)	((SHTC3_T_SLOPE * x) + SHTC3_T_OFFSET)
#define SHTC3_CALC_HUMIDITY(x)		(SHTC3_H_SLOPE * x)


typedef enum _eReadDataIndex
{
	TEMPERATURE_IX = 0,
	HUMIDITY_IX = 3,
}eDataIndex;

typedef enum
{
  SHTC3_NO_ERROR       = 0x00, // no error
  SHTC3_ACK_ERROR      = 0x01, // no acknowledgment error
  SHTC3_CHKSUM_ERROR = 0x02 // checksum mismatch error
}etError;

typedef enum
{
  READ_ID            = 0xEFC8, // command: read ID register
  SOFT_RESET         = 0x805D, // soft reset
  SLEEP              = 0xB098, // sleep
  WAKEUP             = 0x3517, // wakeup
  MEAS_T_RH_POLLING  = 0x7866, // meas. read T first, clock stretching disabled
  MEAS_T_RH_CLOCKSTR = 0x7CA2, // meas. read T first, clock stretching enabled
  MEAS_RH_T_POLLING  = 0x58E0, // meas. read RH first, clock stretching disabled
  MEAS_RH_T_CLOCKSTR = 0x5C24  // meas. read RH first, clock stretching enabled
}etCommands;

typedef enum _EHT_STATUS
{
	EHT_IDLE = 0,
	EHT_START_MEAS,
	EHT_WAIT_DATA,
	EHT_FORMAT_DATA,
	EHT_ERROR,
	EHT_END
}EHT_Status;

/* Read and write function pointer prototypes.
 * parameters are :
 * i2c bus handler pointer, slave address, pointer to data buffer and length
 */
typedef int32_t (*shrc3_write_ptr)(void *, uint8_t, void *, uint16_t);
typedef int32_t (*shrc3_read_ptr) (void *, uint8_t, void *, uint16_t);

typedef struct _shrc3_ctx
{
	uint8_t addr;	// Slave device address
	void *handler;	// I2C bus handler
	shrc3_write_ptr  write_reg;	// Write register function pointer
	shrc3_read_ptr   read_reg;	// Read register function pointer
	uint32_t poll_tm;
	float *temperature;
	float *humidity;
} shrc3_ctx;

typedef struct _shtc3_raw_ht
{
	  uint16_t i_temp;     // temperature raw value from sensor
	  uint16_t i_hum;    	// humidity raw value from sensor
	  uint8_t buf[8];			// I2C Read data buffer
}shtc3_raw_ht;

//==============================================================================
void SHTC3_Init(shrc3_ctx *);
//==============================================================================
// Initializes the I2C bus for communication with the sensor.
//------------------------------------------------------------------------------

//==============================================================================
etError SHTC3_GetId(uint16_t *id);
//==============================================================================
// Gets the ID from the sensor.
//------------------------------------------------------------------------------
// input:  *id          pointer to a integer, where the id will be stored
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      NO_ERROR       = no error

//==============================================================================
etError SHTC3_GetTempAndHumi(float *temp, float *humi);
//==============================================================================
// Gets the temperature [�C] and the humidity [%RH].
//------------------------------------------------------------------------------
// input:  *temp        pointer to a floating point value, where the calculated
//                      temperature will be stored
//         *humi        pointer to a floating point value, where the calculated
//                      humidity will be stored
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      NO_ERROR       = no error
//
// remark: If you use this function, then the sensor blocks the I2C-bus with
//         clock stretching during the measurement.

//==============================================================================
etError SHTC3_GetTempAndHumiPolling(float *temp, float *humi);
//==============================================================================
// Gets the temperature [�C] and the humidity [%RH]. This function polls every
// 1ms until measurement is ready.
//------------------------------------------------------------------------------
// input:  *temp        pointer to a floating point value, where the calculated
//                      temperature will be stored
//         *humi        pointer to a floating point value, where the calculated
//                      humidity will be stored
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      NO_ERROR       = no error

etError SHTC3_Sleep(void);
etError SHTC3_Wakeup(void);

//==============================================================================
etError SHTC3_SoftReset(void);
//==============================================================================
// Calls the soft reset mechanism that forces the sensor into a well-defined
// state without removing the power supply.
//------------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      NO_ERROR       = no error

EHT_Status Humidity_Sensor_Runtime(void);

#endif
