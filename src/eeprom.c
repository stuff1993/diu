#include "eeprom.h"

#include <stdint.h>

#include "i2c.h"
#include "timer.h"

extern volatile uint8_t I2CMasterBuffer[I2C_PORT_NUM][BUFSIZE];
extern volatile uint32_t I2CWriteLength[I2C_PORT_NUM];
extern volatile uint32_t I2CReadLength[I2C_PORT_NUM];
extern volatile uint8_t I2CSlaveBuffer[I2C_PORT_NUM][BUFSIZE];

/******************************************************************************
 ** Function:    ee_init
 **
 ** Description: Reads a word from EEPROM (Uses I2CRead)
 **
 ** Parameters:  Address to read from
 ** Return:      Data at address
 **
 ******************************************************************************/
void ee_init(void)
{
	switch (PORT_USED)
	{
	case 0:
		I2C0Init();
		break;
	case 1:
		I2C1Init();
		break;
	case 2:
		I2C2Init();
		break;
	}
}

/******************************************************************************
 ** Function:    ee_read
 **
 ** Description: Reads a word from EEPROM (Uses I2CRead)
 **
 ** Parameters:  Address to read from
 ** Return:      Data at address
 **
 ******************************************************************************/
uint32_t ee_read(uint16_t _ee_addr)
{
	uint32_t retDATA = 0;

	retDATA = i2c_read(_ee_addr + 3);
	retDATA = (retDATA << 8) + i2c_read(_ee_addr + 2);
	retDATA = (retDATA << 8) + i2c_read(_ee_addr + 1);
	retDATA = (retDATA << 8) + i2c_read(_ee_addr + 0);

	return retDATA;
}

/******************************************************************************
 ** Function:    ee_seq_read
 **
 ** Description: Reads a word from EEPROM (Uses I2CRead)
 **
 ** Parameters:  1. Address to read from
 **              2. Byte length of read
 ** Return:      Data at address
 **
 ******************************************************************************/
uint32_t ee_seq_read(uint16_t _ee_addr, int _len)
{
	uint32_t _ret = 0;

	i2c_seq_read(_ee_addr, _len);
	while (_len--)
	{
		_ret += I2CSlaveBuffer[PORT_USED][_len] << (_len * 8);
	}

	return _ret;
}

/******************************************************************************
 ** Function:    ee_write
 **
 ** Description: Saves a word to EEPROM (Uses I2CWrite)
 **
 ** Parameters:  1. Address to save to
 **              2. Data to save (convert to uint with converter first)
 ** Return:      None
 **
 ******************************************************************************/
void ee_write(uint16_t _ee_addr, uint32_t _ee_data)
{
	uint8_t temp0 = (_ee_data & 0x000000FF);
	uint8_t temp1 = (_ee_data & 0x0000FF00) >> 8;
	uint8_t temp2 = (_ee_data & 0x00FF0000) >> 16;
	uint8_t temp3 = (_ee_data & 0xFF000000) >> 24;

	i2c_write(_ee_addr, temp0, temp1, temp2, temp3);
}

/******************************************************************************
 ** Function:    i2c_read
 **
 ** Description: Reads a byte from EEPROM
 **
 ** Parameters:  Address to read from
 ** Return:      Data at address
 **
 ******************************************************************************/
uint32_t i2c_read(uint16_t _ee_addr)
{
	int i;

	for (i = 0; i < BUFSIZE; i++) // clear buffer
	{
		I2CMasterBuffer[PORT_USED][i] = 0;
	}

	I2CWriteLength[PORT_USED] = 3;
	I2CReadLength[PORT_USED] = 1;
	I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
	I2CMasterBuffer[PORT_USED][1] = (_ee_addr & 0x0f00) >> 8; // address
	I2CMasterBuffer[PORT_USED][2] = _ee_addr & 0x00ff;        // address
	I2CMasterBuffer[PORT_USED][3] = _24LC256_ADDR | RD_BIT;
	I2CEngine( PORT_USED);
	I2CStop(PORT_USED);

	return (uint32_t) I2CSlaveBuffer[PORT_USED][0];
}

/******************************************************************************
 ** Function:    i2c_seq_read
 **
 ** Description: Reads a byte from EEPROM
 **
 ** Parameters:  1. Address to read from
 **              2. Byte length of read
 ** Return:      None
 **
 ******************************************************************************/
void i2c_seq_read(uint16_t _ee_addr, int read_len)
{
	int i;
	for (i = 0; i < BUFSIZE; i++) // clear buffer
	{
		I2CSlaveBuffer[PORT_USED][i] = 0;
	}

	I2CWriteLength[PORT_USED] = 3;
	I2CReadLength[PORT_USED] = read_len;
	I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
	I2CMasterBuffer[PORT_USED][1] = (_ee_addr & 0x0f00) >> 8; // address
	I2CMasterBuffer[PORT_USED][2] = _ee_addr & 0x00ff;        // address
	I2CMasterBuffer[PORT_USED][3] = _24LC256_ADDR | RD_BIT;
	I2CEngine( PORT_USED);
	I2CStop(PORT_USED);
}

/******************************************************************************
 ** Function:    i2c_write
 **
 ** Description: Saves a word to EEPROM
 **
 ** Parameters:  1. Address to save to
 **              2. Data to save
 ** Return:      None
 **
 ******************************************************************************/
void i2c_write(uint16_t _ee_addr, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3)
{
	I2CWriteLength[PORT_USED] = 7;
	I2CReadLength[PORT_USED] = 0;
	I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
	I2CMasterBuffer[PORT_USED][1] = (_ee_addr & 0x0f00) >> 8; // address
	I2CMasterBuffer[PORT_USED][2] = _ee_addr & 0x00ff;        // address
	I2CMasterBuffer[PORT_USED][3] = data0;
	I2CMasterBuffer[PORT_USED][4] = data1;
	I2CMasterBuffer[PORT_USED][5] = data2;
	I2CMasterBuffer[PORT_USED][6] = data3;
	I2CEngine( PORT_USED);

	delayMs(1, 2);
}
