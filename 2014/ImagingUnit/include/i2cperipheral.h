#ifndef _I2C_PERIPHERAL_H
#define _I2C_PERIPHERAL_H

#include <SoftI2C.h>
#include <stopwatch.h>
#include <stdint.h>

#define I2CPERIPHERAL_UNLOCKED 1
#define I2CPERIPHERAL_LOCKED 2
#define I2CPERIPHERAL_GET 3
#define I2CPERIPHERAL_SET 4
#define I2CPERIPHERAL_ERROR_LOCKED 5
#define I2CPERIPHERAL_ERROR_UNKNOWN_REQUEST 6
#define I2CPERIPHERAL_ERROR_TIMER_BUSY 7
#define I2CPERIPHERAL_ERROR_NO_LOCK_AQUIRED 8

template<uint8_t I2CADDRESS>
class I2CPeripheral {
public:
	I2CPeripheral(unsigned long delayTime = 5) {
		peripheralDelayTime = delayTime;
		hasDataAvaiable = false;
		lockAquired = false;
	}

	virtual ~I2CPeripheral() {
	}
	void setInterval(unsigned long interval) {
		stopwatch.setInterval(interval);
	}

	bool busy() {
		return getMutex();
	}

	bool dataAvailable() {
		return hasDataAvaiable;
	}

	void afterNewDataProcessed() {
		hasDataAvaiable = false;
	}

	bool update() {
		if (lockBus() == I2CPERIPHERAL_LOCKED) {
			hasDataAvaiable = hasDataAvaiable || onUpdate();
			unlockBus();
		}

		return hasDataAvaiable;
	}

	//------------------------------------------------------------------------------
	/*
	 * Read 'count' bytes from the device starting at 'address'.
	 */
	uint8_t read(uint8_t address, uint8_t *buf, uint8_t count) {

		i2c_start_wait(I2CADDRESS | I2C_WRITE);
//		if (!i2c_start(I2CADDRESS | I2C_WRITE))
//			return false;
		if (!i2c_write(address))
		{
			i2c_stop();
			return false;
		}

		i2c_stop();

		if (!i2c_start(I2CADDRESS | I2C_READ))
		{
			i2c_stop();

			return false;
		}

		while (count > 0) {
			count --;
			bool lastone = count < 1;
			*buf = i2c_read(lastone);
			buf++;
		}
		i2c_stop();

		return true;
	}

	uint8_t read(uint8_t address, uint8_t & val) {
		uint8_t tmp;
		uint8_t res = read(address, (uint8_t *) &tmp, 1);
		val = tmp;
		return res;
	}

	//------------------------------------------------------------------------------
	/*
	 * write 'count' bytes to device starting at 'address'.
	 */
	uint8_t write(uint8_t address, uint8_t *buf, uint8_t count) {

		i2c_start_wait(I2CADDRESS | I2C_WRITE);

		//if (!i2c_start(I2CADDRESS | I2C_WRITE))
		//	return false;

		if (!i2c_write(address))
			return false;

		while(count > 0)
		{
			count--;
			if (!i2c_write(*buf))
				return false;

			buf++;
		}

		i2c_stop();

		return true;
	}

	uint8_t write(uint8_t *buf, uint8_t count) {

		i2c_start_wait(I2CADDRESS | I2C_WRITE);

		//if (!i2c_start(I2CADDRESS | I2C_WRITE))
		//	return false;

		while(count-- > 0)
		{
			if (!i2c_write(*buf++))
				return false;
		}

		i2c_stop();

		return true;
	}

	uint8_t write(uint8_t address, uint8_t val) {

		return write(address, (uint8_t *) &val, 1);
	}

	bool isPeripheralError() const {
		return peripheralError;
	}

	void setPeripheralError(bool peripheralError) {
		this->peripheralError = peripheralError;
	}

protected:

// User must implement this function
// returns true if new data is available as a result of operation
	virtual bool onUpdate() = 0;

	int lockBus() {
		if (!stopwatch.timeout(STOPWATCH_NORESTART)) {
			return I2CPERIPHERAL_ERROR_TIMER_BUSY;
		}

		bool mutex = getMutex();

		if (mutex) {
			return I2CPERIPHERAL_ERROR_LOCKED;
		}

		lockAquired = true;
		mutex = true;
		stopwatch.reset();

		return I2CPERIPHERAL_LOCKED;
	}

	int unlockBus() {
//        if (!lockAquired)
//        {
//          return I2CPERIPHERAL_ERROR_NO_LOCK_AQUIRED;
//        }

		bool mutex = getMutex();

		if (!mutex) {
			return I2CPERIPHERAL_ERROR_NO_LOCK_AQUIRED;
		}

		mutex = false;
		lockAquired = false;
		stopwatch.reset();
		return I2CPERIPHERAL_UNLOCKED;
	}

	void setDataAvailable() {
		hasDataAvaiable = true;
	}

// How much time between each peripheral access?
	unsigned long peripheralDelayTime;
	bool lockAquired;
	bool hasDataAvaiable;
	bool peripheralError;

private:

	Stopwatch stopwatch;

// for single device
	static bool & getMutex() {
		static bool mutex = false;

		return mutex;
	}
};

#endif

