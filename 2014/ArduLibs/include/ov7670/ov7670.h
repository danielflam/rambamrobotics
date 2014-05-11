/*
 * OV7670.h
 *
 *  Created on: 2013/10/31
 *      Author: sin
 */

#ifndef OV7670_H_
#define OV7670_H_

//#ifdef ARDUINO
#include <Arduino.h>
#include <Wire/Wire.h>
//#include "gpio.h"
//#include <progmem.h>
#include "DigitalIO/DigitalPin.h"
#define FLASH_DEF PROGMEM
//#else
//#define FLASH_DEF
//#endif

#include "OV7670_regdef.h"

const uint8_t _VSYNC = 2;
const uint8_t _HREF = 3;
const uint8_t _WEN = 4;
const uint8_t _RCLK = 5;
const uint8_t _RRST =  6;
const uint8_t _OE = 7;

class OV7670 {


	static const byte I2C_ADDR = 0x21;
	static const prog_uint8_t REGCONF_QQVGA_RGB565[] FLASH_DEF;
	static const prog_uint8_t REGCONF_QVGA_RGB565[] FLASH_DEF;
	static const prog_uint8_t REGCONF_COLOR[] FLASH_DEF;
	static const prog_uint8_t REGCONF_COLOR_SETTING[] FLASH_DEF;

public:


	OV7670(){}


	uint8_t readRegister(byte reg);
	uint8_t writeRegister(byte reg, byte val);

	boolean begin(void);

	boolean resetRegisters(void) {
		return writeRegister(REG_COM7, COM7_RESET) == 0; // reset all registers
	}

	boolean writeRegisterValues(const prog_uint8_t regvals[]);

	void outputEnable() { fastDigitalWriteLow(_OE); } // _OE
	void outputDisable() { fastDigitalWriteHigh(_OE); } // _OE

	void writeEnable() { fastDigitalWriteLow(_WEN); }
	void writeDisable() { fastDigitalWriteHigh(_WEN); }

	void RRST(uint8_t val) { fastDigitalWrite(_RRST,val); }

	/*
	void startRCLK(uint8_t speed = 0) {
		 // RCLK 1MHz  by TC2 fast-PWM with TOP=OCRA
		switch (speed) {
		case 1: // LOW
		  TCCR2A =  _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
		  TCCR2B =  _BV(WGM22) | _BV(CS20); // | _BV(CS21) | _BV(CS22);
		  OCR2A = 63;
		  OCR2B = 31;
		  break;
		case 3: // High
		default: // 1MHz
		  TCCR2A =  _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
		  TCCR2B =  _BV(WGM22) | _BV(CS20); //| _BV(CS21) | _BV(CS22);
		  OCR2A = 7;
		  OCR2B = 3;
		  break;
		}
	}

	void stopRCLK(void) {
		TCCR2A &=  ~(_BV(COM2B1) );
	}
	*/

	void RCLK(uint8_t val) {
		fastDigitalWrite(_RCLK, val); //(_RCLK, val);
	}

	void waitVSYNCRising() {
		while (fastDigitalRead(_VSYNC)) {}
		while (!fastDigitalRead(_VSYNC)) {}
	}
	void waitVSYNCFalling() {
		while (!fastDigitalRead(_VSYNC)) {}
		while (fastDigitalRead(_VSYNC)) {}
	}

	void waitPulseHREF(uint8_t pulse = 1) {
		if (pulse) {
			while (!fastDigitalRead(_HREF));
			while (fastDigitalRead(_HREF));
		} else {
			while (fastDigitalRead(_HREF));
			while (!fastDigitalRead(_HREF));
		}
	}

	void mirrorflip(uint8_t mode) {
		uint8_t val = readRegister(REG_MVFP);
		val &= ~0x30;
		// b1 -- hmirror, b0 -- vflip
		writeRegister(REG_MVFP, ( (mode&1? (1<<5) : 0) | (mode>>1&1 ? (1<<4) : 0 ) ) );
	}

	void maxgain(uint8_t mx) {
		uint8_t val = readRegister(REG_COM9);
		val &= ~0x70;
		val |= (mx&7)<<4;
		writeRegister(REG_COM9, val);
	}

	void enhanceEdge(uint8_t fac) {
		uint8_t val = readRegister(REG_EDGE);
		val &= ~0x0f;
		val |= fac&0x0f;
		writeRegister(REG_EDGE, val);
	}

	void contrast(uint8_t c) {
		writeRegister(REG_EDGE, c);
	}
};

#endif /* OV7670_H_ */
