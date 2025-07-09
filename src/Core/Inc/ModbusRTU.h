/*
 * ModbusRTU.h
 *
 * Created: 08/02/2016 8:58:54
 *  Author: agarcia
 */ 


#ifndef MODBUSRTU_H_
#define MODBUSRTU_H_

#define MASTER_PORT			1		// Number of master port
#define MBUS_RSPTOUT		6		// 600uS timeout for 19200baud
// #define Slave_ID			1		// Mod bus Salve address
#define MBUS_TIMEOUT		35		// Timeout in bit time

#define IDPROG_VAL	251
#define MODBUS_RTU_MAX_REGISTERS	125

/* Frame timeout as defined on MODBUS over serial line specification and implementation guide V1.02
 *	Defined as 3.5 * MODBUS TIMER CLOCK FREQUENCY (10MHz) */

#define MODBUS_FTM35	35000000UL

/********************************************************************
These exceptions are defined in the MODBUS protocol.  These can be
used by the slave to communicate problems with the transmission back
to the master who can also use these to easily check the exceptions.
The first exception is the only one that is not part of the protocol
specification.  The TIMEOUT exception is returned when no slave
responds to the master's request within the timeout period.
********************************************************************/
typedef enum _exception{NO_ERROR=0,ILLEGAL_FUNCTION=1,ILLEGAL_DATA_ADDRESS=2,
	ILLEGAL_DATA_VALUE=3,SLAVE_DEVICE_FAILURE=4,ACKNOWLEDGE=5,SLAVE_DEVICE_BUSY=6,
	MEMORY_PARITY_ERROR=8,GATEWAY_PATH_UNAVAILABLE=10,GATEWAY_TARGET_NO_RESPONSE=11,
TIMEOUT=12, SLAVE_NOT_ME=15, NOSEND_RSP = 16 } exception;

/********************************************************************
These functions are defined in the MODBUS protocol.  These can be
used by the slave to check the incomming function.  See
ex_modbus_slave.c for example usage.
********************************************************************/
typedef enum _function{FUNC_READ_COILS=0x01,FUNC_READ_DISCRETE_INPUT=0x02,
	FUNC_READ_HOLDING_REGISTERS=0x03,FUNC_READ_INPUT_REGISTERS=0x04,
	FUNC_WRITE_SINGLE_COIL=0x05,FUNC_WRITE_SINGLE_REGISTER=0x06,
	FUNC_READ_EXCEPTION_STATUS=0x07,FUNC_DIAGNOSTICS=0x08,
	FUNC_GET_COMM_EVENT_COUNTER=0x0B,FUNC_GET_COMM_EVENT_LOG=0x0C,
	FUNC_WRITE_MULTIPLE_COILS=0x0F,FUNC_WRITE_MULTIPLE_REGISTERS=0x10,
	FUNC_REPORT_SLAVE_ID=0x11,FUNC_READ_FILE_RECORD=0x14,
	FUNC_WRITE_FILE_RECORD=0x15,FUNC_MASK_WRITE_REGISTER=0x16,
FUNC_READ_WRITE_MULTIPLE_REGISTERS=0x17,FUNC_READ_FIFO_QUEUE=0x18} function;


// MODBUS define

#define MB_MAX_DATALENGTH		126	// Maximum data length for modbus datagram in uint16_t size

// Defines of MODBUS memory regions

#define MEAS_GROUP      0x00    // Measured registers and digital output register
#define DEVCFG_GROUP    0xF0    // Device configuration registers


// Define value to execute commands

#define CMD_PASS 0xff00

#define BACKDOOR_LEVEL 0xffffffff

//  Default configuration for MODBUS response buffer

#define MB_COM_BUFFER	256		// Comunication buffer size

enum eMBRTM_STATE
{
	eMBRTM_RESET = 0,
	eMBRTM_WAIT_RX,
	eMBRTM_PROC_RQST,
	eMBRTM_WAIT_TX,
	eMBRTM_LOCK
};

typedef struct
{
	uint8_t SlaveID;
	uint8_t Function;
	uint8_t nBytes;	// Byte count
	uint8_t Data[MB_COM_BUFFER + 1];
}stMbusReadRSP;

#define AES_BCKLEN 16 //Block length in bytes AES is 128b block only

typedef struct
{
	uint8_t SlaveID;
	uint8_t Function;
	uint16_t Start;
	uint16_t nRegisters;
	uint16_t m_crc;
	uint32_t nBytes;
	uint8_t Data[2 * AES_BCKLEN];
}stMbusWriteRSP;

typedef struct
{
	uint8_t SlaveID;			// Modbus Slave address
	uint8_t Function;			// Modbus Function
	uint16_t Start;             // Register start
	uint16_t nRegisters;        // N� of register to read or write
	uint16_t nBytes;            // N� of bytes 2*Length
	uint16_t m_crc;               // Modbus Frame CRC
	uint8_t Data[MB_COM_BUFFER];   // Formation frame data buffer
	uint8_t *pData;             // Pointer to data (if read points to input buffer, if write points data to send)
}stMbusRequest;

typedef struct
{
	uint16_t cmd;
	uint8_t param[18];
}stModBusCmd;

typedef int32_t (*modbus_response_fn)(void *data, uint16_t length, uint8_t port);

// Function prototypes

stMbusReadRSP *GetMbReadRsp(void);
stMbusWriteRSP *GetMbWriteRsp(void);
uint16_t GetModBusReg(uint8_t *);
uint16_t fReadHldRegRSP(uint8_t *,uint16_t ,uint16_t, stMbusReadRSP *, uint8_t );   // Function Read Holding register response computing
uint16_t fWriteHldRegRSP(uint8_t* ,uint8_t *, uint16_t, stMbusWriteRSP *, uint8_t);
uint32_t Compute_MBUSRequest(uint8_t *pData, uint8_t, uint8_t, modbus_response_fn);                        // Mod bus master request computing
uint32_t ProcessMbusRequest(uint8_t *, uint8_t *, uint16_t , uint16_t , uint8_t);
uint32_t fGetLastCOM_tm(uint32_t );		// Get last communication time in tick count
uint32_t fGetErrCOM_Cnt(uint32_t );		// Get number of comunication errrors
void fClr_LastCOM(uint32_t );
void ModbusInit(void);

#else
	#warning "ModbusRTU.h have been defined!"
#endif /* MODBUSRTU_H_ */

