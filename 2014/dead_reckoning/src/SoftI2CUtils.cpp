/*
 * SoftI2CUtils.cpp
 *
 *  Created on: Apr 12, 2014
 *      Author: Danny
 */

#ifndef SOFTI2CUTILS_CPP_
#define SOFTI2CUTILS_CPP_

#include <SoftI2CUtils.h>

	//------------------------------------------------------------------------------
	/*
	 * Read 'count' bytes from the device starting at 'address'.
	 */
	uint8_t read(uint8_t devAddress, uint8_t address, uint8_t *buf,
			uint8_t count) {

//		i2c_start_wait(devAddress | I2C_WRITE);
		if (!i2c_start(devAddress | I2C_WRITE))
			return false;
		if (!i2c_write(address)) {
			i2c_stop();
			return false;
		}
		i2c_stop();

		if (!i2c_start(devAddress | I2C_READ)) {
			i2c_stop();

			return false;
		}

		while (count > 0) {
			count--;
			bool lastone = count < 1;
			*buf = i2c_read(lastone);
			buf++;
		}
		i2c_stop();

		return true;
	}

	uint8_t read(uint8_t devAddress, uint8_t address, uint8_t & val) {
		uint8_t res = read(devAddress, address, (uint8_t *) &val, 1);
		return res;
	}

	int8_t readRegister(uint8_t devAddress, uint8_t regAddress)
	{
		uint8_t tmp;
		read(devAddress, regAddress, tmp);
		return tmp;
	}

	int16_t readLH(uint8_t devAddress, uint8_t address)
	{
		byte h, l;
		  read(devAddress, address, l);
		  read(devAddress, address+1, h);
		  return l + (h<<8);
	}
	//------------------------------------------------------------------------------
	/*
	 * write 'count' bytes to device starting at 'address'.
	 */
	uint8_t write(uint8_t devAddresss, uint8_t address, uint8_t *buf,
			uint8_t count) {

		//i2c_start_wait(devAddresss | I2C_WRITE);

		if (!i2c_start(devAddresss | I2C_WRITE))
			return false;

		if (!i2c_write(address))
			return false;

		while (count > 0) {
			count--;
			if (!i2c_write(*buf))
				return false;

			buf++;
		}

		i2c_stop();

		return true;
	}

	uint8_t write(uint8_t devAddress, uint8_t *buf, uint8_t count) {

		//i2c_start_wait(devAddress | I2C_WRITE);

		if (!i2c_start(devAddress | I2C_WRITE))
			return false;

		while (count-- > 0) {
			if (!i2c_write(*buf++))
				return false;
		}

		i2c_stop();

		return true;
	}

	uint8_t write(uint8_t devAddress, uint8_t regAddress, uint8_t val) {

		return write(devAddress, regAddress, (uint8_t *) &val, 1);
	}

	/*	uint8_t writeHL(uint8_t devAddress, uint8_t regAddress, uint16_t val) {

	 return write(devAddress, regAddress, (uint8_t *) &val, 1);
	 } */

	/*	uint8_t writeLH(uint8_t devAddress, uint8_t regAddress, uint16_t val) {

	 return write(devAddress, regAddress, (uint8_t *) &val, 1);
	 } */





#endif /* SOFTI2CUTILS_CPP_ */
