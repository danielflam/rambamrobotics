/*
 * SoftI2cUtils.h
 *
 *  Created on: Apr 12, 2014
 *      Author: Danny
 */

#ifndef SOFTI2CUTILS_H_
#define SOFTI2CUTILS_H_

#include <SoftI2C.h>

uint8_t read(uint8_t devAddress, uint8_t regAddress, uint8_t *buf, uint8_t count);
uint8_t read(uint8_t devAddress, uint8_t regAddress, uint8_t & val);
int16_t readLH(uint8_t devAddress, uint8_t regAddress);
int8_t readRegister(uint8_t devAddress, uint8_t regAddress);

//------------------------------------------------------------------------------
/*
 * write 'count' bytes to device starting at 'address'.
 */
uint8_t write(uint8_t devAddresss, uint8_t address, uint8_t *buf,
		uint8_t count);
uint8_t write(uint8_t devAddress, uint8_t *buf, uint8_t count);
uint8_t write(uint8_t devAddress, uint8_t regAddress, uint8_t val);


#endif /* SOFTI2CUTILS_H_ */
