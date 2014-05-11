/*
 * OV7670.cpp
 *
 *  Created on: 2013/11/05
 *      Author: sin
 */

#include <ov7670.h>

uint8_t OV7670::readRegister(byte reg) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR, (byte)1);
  return Wire.read();
}

uint8_t OV7670::writeRegister(byte reg, byte val) {
	Wire.beginTransmission(I2C_ADDR);
	Wire.write(reg);
	Wire.write(val);
	return Wire.endTransmission();
}

boolean OV7670::writeRegisterValues(const prog_uint8_t regvals[]) {
	uint8_t reg, val;
	boolean result;
	Serial.println("write registers...");
	// OV7670_REGCONF_QVGA
	for (int i = 0; (reg = pgm_read_byte_near(regvals+2*i)) != 0xff
			&& (val = pgm_read_byte_near(regvals+2*i+1)) != 0xff; i++) {
		Serial.print(reg, HEX);
		Serial.print(": ");
		Serial.println(val, HEX);
		for(int i = 0; i < 3 ; i++) {
			result = writeRegister(reg, val);
			if ( result == 0 )
				break;
		}
		if ( result != 0 )
			return false;
	}
	return true;
}

boolean OV7670::begin(void) {

	// setup parrallel interface.
//	fastPinMode(_WEN, OUTPUT); // WEN
	fastDigitalWriteLow(_WEN);
	fastPinMode(_VSYNC, INPUT);  // VSYNC

	fastPinMode(_HREF, INPUT);  // HREF
	fastPinMode(_RRST, OUTPUT);
	fastDigitalWriteHigh(_RRST);

	// RCLK 1MHz  by TC2 fast-PWM with TOP=OCRA
	fastPinMode(_RCLK, OUTPUT);
	fastDigitalWriteHigh(_RCLK);

	fastPinMode(_OE, OUTPUT);
	fastDigitalWriteHigh(_OE);

#ifdef DEBUG
	Serial.println(_HREF, HEX);
	Serial.println(_RRST, HEX);
	Serial.println("Ok?");
	while(1);
#endif

	Serial.println("pin modes have been set.");

	resetRegisters(); // reset all registers
	Serial.println("registers reset.");

	return writeRegisterValues(REGCONF_QVGA_RGB565)
			&& writeRegisterValues(REGCONF_COLOR_SETTING);

}


const prog_uint8_t OV7670::REGCONF_QQVGA_RGB565[] FLASH_DEF = {
		REG_CLKRC, 0x80, // clock prescaler: Reserved, Use external directly, 1(BIT[5:0]+1)
		REG_COM11, 0x12, // com11: night mode disable, Min. frame rate of night mode,
						 // 50/60 auto detection, Banding filter value select, reserved, exposure timing, reserved

		REG_TSLB, 0x04, // normal mode
		REG_COM7, 0x04, // output format: rgb
		REG_RGB444, 0x00, // disable RGB444, select xRGB
		REG_COM15, 0xD0, // set RGB565, range 00-ff
		//
		// not even sure what all these do, gonna check the oscilloscope and go
		// from there...
		REG_HSTART, 0x16, REG_HSTOP, 0x04, REG_HREF, 0x24, // hreff: lower bits of hstart, hstop
		REG_VSTART, 0x02, REG_VSTOP, 0x7a, REG_VREF, 0x0a, // vref: lower bits of vstart, vstop
		REG_COM10, 0x02, // com10: BIT[1] vsync negative
		REG_COM3, 0x04, 	// COM3: BIT[2] DCW enable
		REG_COM14, 0x1a, // BIT[4] DCW and scaling PCLK enable,
						 // BIT[3] manual scaling enable BIT[2:0] PCLK divider, divide by 4
		//ov7670_set(REG_COM14, 0x1b); // divide by 8
		REG_MVFP, 0x27, // (default x01) BIT[5] mirror, BIT[4] VFlip, BIT[2] black sun enable, Bit[7, 6, 3, 1, 0] reserved.
		REG_SCALING_DCWCTR, 0x22, // Bit[5:4] vertical down sampling rate,
		// Bit[1:0] hrizontal down sampling rate downsample by 4
		//ov7670_set(0x72, 0x33); // downsample by 8
		REG_SCALING_PCLK_DIV, 0x02, // Bit[3] bypass clock divider for DSP scale control, [2:0], divide by 4
		//ov7670_set(0x73, 0xf3); // divide by 8
		0xff, 0xff,
};

const prog_uint8_t OV7670::REGCONF_QVGA_RGB565[] FLASH_DEF = {
// QVGA RGB565
        REG_CLKRC,0x80,
        REG_COM11,0x0A,
        REG_TSLB,0x04,
        REG_COM7,0x04,
        REG_RGB444, 0x00,
        REG_COM15, 0xd0,
        REG_HSTART,0x16,
        REG_HSTOP,0x04,
        REG_HREF,0x80,
        REG_VSTART,0x02,
        REG_VSTOP,0x7a,
        REG_VREF,0x0a,
        REG_COM10,0x02,
        REG_COM3, 0x04,
        REG_COM14, 0x19,
        REG_MVFP, (0x07 | 0x10 | 0x20),
        REG_SCALING_DCWCTR, 0x11,
        REG_SCALING_PCLK_DIV, 0xf1,
        0xff, 0xff
};

const prog_uint8_t OV7670::REGCONF_COLOR_SETTING[] FLASH_DEF = {
	      // COLOR SETTING
	        0x4f,0x80,
	        0x50,0x80,
	        0x51,0x00,
	        0x52,0x22,
	        0x53,0x5e,
	        0x54,0x80,
	        0x56,0x40,
	        0x58,0x9e,
	        0x59,0x88,
	        0x5a,0x88,
	        0x5b,0x44,
	        0x5c,0x67,
	        0x5d,0x49,
	        0x5e,0x0e,
	        0x69,0x00,
	        0x6a,0x40,
	        0x6b,0x0a,
	        0x6c,0x0a,
	        0x6d,0x55,
	        0x6e,0x11,
	        0x6f,0x9f,

	        0xb0,0x84,

	        0xff, 0xff
};

const prog_uint8_t OV7670::REGCONF_COLOR[] FLASH_DEF = {
	    // COLOR SETTING
	    0x4f, 0x80,
	    0x50, 0x80,
	    0x51, 0x00,
	    0x52, 0x22,
	    0x53, 0x5e,
	    0x54, 0x80,
	    0x56, 0x40,
	    0x58, 0x9e,
	    0x59, 0x88,
	    0x5a, 0x88,
	    0x5b, 0x44,
	    0x5c, 0x67,
	    0x5d, 0x49,
	    0x5e, 0x0e,
	    0x69, 0x00,
	    0x6a, 0x40,
	    0x6b, 0x0a,
	    0x6c, 0x0a,
	    0x6d, 0x55,
	    0x6e, 0x11,
	    0x6f, 0x9f,

	    0xb0, 0x84,

		/*
		0x138f, // com8: default 8f // [2:0] agc, awb, aec
		//0x01b0,
		//0x02b0,
		0x145a, // com9: default 4a
		//0x0c40, // tristate, bit
		//0x1202, // test color bar

		// AWB
		0x13e7, // AWB on, Light mode Auto
		0x6f9f, // Simple AWB
		//
		0x4f99,
		0x5099,
		0x5100,
		0x5228,
		0x5371,
		0x5499,
		0x589e,
		// brightness
		0x5500,
		// contrast
		0x5650,
		//
	*/
		0xff, 0xff,
};

