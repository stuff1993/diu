#ifndef EEPROM_H_
#define EEPROM_H_

#include <stdint.h>

#define PORT_USED 1 // I2C port

uint32_t  	ee_read             (uint16_t _EEadd);
uint32_t  	ee_seq_read         (uint16_t _EEadd, int _len);
void      	ee_write            (uint16_t _EEadd, uint32_t data);
uint32_t  	i2c_read            (uint16_t _EEadd);
void      	i2c_seq_read        (uint16_t _EEadd, int read_len);
void      	i2c_write           (uint16_t _EEadd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3);

#endif /* EEPROM_H_ */
