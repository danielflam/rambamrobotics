/*
 * adjd_s311.h
 *
 *  Created on: Apr 6, 2014
 *      Author: Danny
 */

#ifndef ADJD_S311_H_
#define ADJD_S311_H_

// ADJD-S311's I2C address, don't change
#define ADJD_S311_ADDRESS 0x74

#define RED 0
#define GREEN 1
#define BLUE 2
#define CLEAR 3

// ADJD-S311's register list
#define CTRL 0x00
#define CONFIG 0x01
#define CAP_RED 0x06
#define CAP_GREEN 0x07
#define CAP_BLUE 0x08
#define CAP_CLEAR 0x09
#define INT_RED_LO 0xA
#define INT_RED_HI 0xB
#define INT_GREEN_LO 0xC
#define INT_GREEN_HI 0xD
#define INT_BLUE_LO 0xE
#define INT_BLUE_HI 0xF
#define INT_CLEAR_LO 0x10
#define INT_CLEAR_HI 0x11
#define DATA_RED_LO 0x40
#define DATA_RED_HI 0x41
#define DATA_GREEN_LO 0x42
#define DATA_GREEN_HI 0x43
#define DATA_BLUE_LO 0x44
#define DATA_BLUE_HI 0x45
#define DATA_CLEAR_LO 0x46
#define DATA_CLEAR_HI 0x47
#define OFFSET_RED 0x48
#define OFFSET_GREEN 0x49
#define OFFSET_BLUE 0x4A
#define OFFSET_CLEAR 0x4B

class ADJD_S311 {

public:

	// initial values for integration time registers
	unsigned char colorCap[4]; // = {9, 9, 2, 5};  // values must be between 0 and 15
	unsigned int colorInt[4]; // = {2048, 2048, 2048, 2048};  // max value for these is 4095
	unsigned int colorData[4]; // This is where we store the RGB and C data values
	signed char colorOffset[4];  // Stores RGB and C offset values
	float colorYUV[4];
	byte ledPin;

	ADJD_S311() {
		ledPin = 2;
		colorCap[0] = 9;
		colorCap[1] = 9;
		colorCap[2] = 2;
		colorCap[3] = 5;

		colorInt[0] = 2048;
		colorInt[1] = 2048;
		colorInt[2] = 2048;
		colorInt[3] = 2048;
	}

	void writeRegister(unsigned char data, unsigned char address) {
		Wire.beginTransmission(ADJD_S311_ADDRESS);
		Wire.write(address);
		Wire.write(data);
		Wire.endTransmission();
	}

	// read a byte of data from ADJD-S311 address
	unsigned char readRegister(unsigned char address) {
//		unsigned char data;

		Wire.beginTransmission(ADJD_S311_ADDRESS);
		Wire.write(address);
		Wire.endTransmission();

		Wire.requestFrom(ADJD_S311_ADDRESS, 1);
		while (!Wire.available())
			;  // wait till we can get data

		return Wire.read();
	}

	// Write two bytes of data to ADJD-S311 address and addres+1
	int readRegisterInt(unsigned char address) {
		return readRegister(address) + (readRegister(address + 1) << 8);
	}

	/* writeInt() - This function writes a 12-bit value
	 to the LO and HI integration registers */
	void writeInt(int address, int gain) {
		if (gain < 4096) {
			byte msb = gain >> 8;
			byte lsb = gain;

			writeRegister(lsb, address);
			writeRegister(msb, address + 1);
		}
	}

	/* performMeasurement() - This must be called before
	 reading any of the data registers. This commands the
	 ADJD-S311 to perform a measurement, and store the data
	 into the data registers.*/
	void performMeasurement() {
		writeRegister(0x01, 0x00); // start sensing
		while (readRegister(0x00) != 0)
			; // waiting for a result
	}

	/* getRGBC() - This function reads all of the ADJD-S311's
	 data registers and stores them into colorData[]. */
	void getRGBC() {
		performMeasurement();

		colorData[RED] = readRegisterInt(DATA_RED_LO);
		colorData[GREEN] = readRegisterInt(DATA_GREEN_LO);
		colorData[BLUE] = readRegisterInt(DATA_BLUE_LO);
		colorData[CLEAR] = readRegisterInt(DATA_CLEAR_LO);

	}

	/* getYUV() - This function reads all of the ADJD-S311's
	 data registers and stores them into colorData[] and then convert to YUV
	 format. */
	void getYUV()
	{
		getRGBC();
		colorYUV[0] = float(colorData[RED] / 4.0) * 0.299
				+ float(colorData[GREEN] / 4.0) * 0.587
				+ float(colorData[BLUE] / 4.0) * 0.114;
		colorYUV[1] = float(colorData[RED] / 4.0) * -0.169
				+ float(colorData[GREEN] / 4.0) * -0.332
				+ float(colorData[BLUE] / 4.0) * 0.500 + 128;
		colorYUV[2] = float(colorData[RED] / 4.0) * 0.500
				+ float(colorData[GREEN] / 4.0) * -0.419
				+ float(colorData[BLUE] / 4.0) * -0.0813 + 128;
		colorYUV[3] = float(colorData[RED] / 4.0) * 0.6
				+ float(colorData[BLUE] / 4.0) * 0.4;

	}

	/* getOffset() - This function performs the offset reading
	 and stores the offset data into the colorOffset[] array.
	 You can turn on data trimming by uncommenting out the
	 writing 0x01 to 0x01 code.
	 */
	void getOffset() {
		digitalWrite(ledPin, LOW);  // turn LED off
		delay(10);  // wait a tic
		writeRegister(0x02, 0x00); // start sensing
		while (readRegister(0x00) != 0)
			; // waiting for a result
		//writeRegister(0x01, 0x01);  // set trim
		//delay(100);
		for (int i = 0; i < 4; i++)
			colorOffset[i] = (signed char) readRegister(OFFSET_RED + i);
		digitalWrite(ledPin, HIGH);
	}

	/* printADJD_S311Values() reads, formats, and prints all important registers
	 of the ADJD-S311.
	 It doesn't perform any measurements, so you'll need to call getRGBC() to print
	 new values.
	 */
	void print() {
		Serial.println("\t\t Red \t Green \t Blue \t Clear");
		Serial.print("Data: \t\t ");
		for (int i = 0; i < 4; i++) {
			Serial.print(colorData[i]);
			Serial.print("\t ");
		}
		Serial.println();
		Serial.print("Caps: \t\t ");
		for (int i = 0; i < 4; i++) {
			Serial.print(readRegister(CAP_RED + i));
			Serial.print("\t ");
		}
		Serial.println();
		Serial.print("Int: \t\t ");
		for (int i = 0; i < 4; i++) {
			Serial.print(readRegisterInt(INT_RED_LO + (i * 2)));
			Serial.print("\t ");
		}
		Serial.println();
		Serial.print("Offset: \t ");
		for (int i = 0; i < 4; i++) {
			Serial.print((signed char) readRegister(OFFSET_RED + i));
			Serial.print("\t ");
		}
		Serial.println();
	}

	void calibrate() {
		Serial.println("Calibrating color...");
		calibrateColor();  // This calibrates R, G, and B int registers
		Serial.println("Calibrating clean...");
		calibrateClear();  // This calibrates the C int registers
		Serial.println("Calibrating caps...");
		calibrateCapacitors();  // This calibrates the RGB, and C cap registers
	}

	/* initADJD_S311() - This function initializes the ADJD-S311 and its
	 capacitor and integration registers
	 The vaules for those registers are defined near the top of the code.
	 the colorCap[] array defines all capacitor values, colorInt[] defines
	 all integration values.
	 */
	void init() {

		/*sensor gain registers, CAP_...
		 to select number of capacitors.
		 value must be <= 15 */
		writeRegister(colorCap[RED] & 0xF, CAP_RED);
		writeRegister(colorCap[GREEN] & 0xF, CAP_GREEN);
		writeRegister(colorCap[BLUE] & 0xF, CAP_BLUE);
		writeRegister(colorCap[CLEAR] & 0xF, CAP_CLEAR);

		/* Write sensor gain registers INT_...
		 to select integration time
		 value must be <= 4096 */
		writeRegister((unsigned char) colorInt[RED], INT_RED_LO);
		writeRegister((unsigned char) ((colorInt[RED] & 0x1FFF) >> 8),
		INT_RED_HI);
		writeRegister((unsigned char) colorInt[BLUE], INT_BLUE_LO);
		writeRegister((unsigned char) ((colorInt[BLUE] & 0x1FFF) >> 8),
		INT_BLUE_HI);
		writeRegister((unsigned char) colorInt[GREEN], INT_GREEN_LO);
		writeRegister((unsigned char) ((colorInt[GREEN] & 0x1FFF) >> 8),
		INT_GREEN_HI);
		writeRegister((unsigned char) colorInt[CLEAR], INT_CLEAR_LO);
		writeRegister((unsigned char) ((colorInt[CLEAR] & 0x1FFF) >> 8),
		INT_CLEAR_HI);

		pinMode(ledPin, OUTPUT);  // Set the sensor's LED as output
		digitalWrite(ledPin, HIGH);  // Initially turn LED light source on

	}

	/* calibrateClear() - This function calibrates the clear integration registers
	 of the ADJD-S311.
	 */
	int calibrateClear() {
		int gainFound = 0;
		int upperBox = 4096;
		int lowerBox = 0;
		int half;

		while (!gainFound) {
			half = ((upperBox - lowerBox) / 2) + lowerBox;
			//no further halfing possbile
			if (half == lowerBox)
				gainFound = 1;
			else {
				writeInt(INT_CLEAR_LO, half);
				performMeasurement();
				int halfValue = readRegisterInt(DATA_CLEAR_LO);

				if (halfValue > 1000)
					upperBox = half;
				else if (halfValue < 1000)
					lowerBox = half;
				else
					gainFound = 1;
			}
		}
		return half;
	}

	/* calibrateColor() - This function clalibrates the RG and B
	 integration registers.
	 */
	int calibrateColor() {
		int gainFound = 0;
		int upperBox = 4096;
		int lowerBox = 0;
		int half;
		int step = 0;
		while (!gainFound) {
			step++;
			half = ((upperBox - lowerBox) / 2) + lowerBox;
			Serial.print(half);
			Serial.print("...");
			//no further halfing possbile
			if (half == lowerBox) {
				gainFound = 1;
			} else {
				writeInt(INT_RED_LO, half);
				writeInt(INT_GREEN_LO, half);
				writeInt(INT_BLUE_LO, half);

				performMeasurement();
				int halfValue = 0;

				halfValue = max(halfValue, readRegisterInt(DATA_RED_LO));
				halfValue = max(halfValue, readRegisterInt(DATA_GREEN_LO));
				halfValue = max(halfValue, readRegisterInt(DATA_BLUE_LO));

				if (halfValue > 950) {
					upperBox = half;
				} else if (halfValue < 950) {
					lowerBox = half;
				} else {
					gainFound = 1;
				}
			}
		}
		Serial.print("complete.");

		return half;
	}

	/* calibrateCapacitors() - This function calibrates each of the RGB and C
	 capacitor registers.
	 */
	void calibrateCapacitors() {
		int calibrationRed = 0;
		int calibrationBlue = 0;
		int calibrationGreen = 0;
		int calibrated = 0;

		//need to store detect better calibration
		int oldDiff = 5000;

		while (!calibrated) {
			// sensor gain setting (Avago app note 5330)
			// CAPs are 4bit (higher value will result in lower output)
			writeRegister(calibrationRed, CAP_RED);
			writeRegister(calibrationGreen, CAP_GREEN);
			writeRegister(calibrationBlue, CAP_BLUE);

			// int colorGain = _calibrateColorGain();
			int colorGain = readRegisterInt(INT_RED_LO);
			writeInt(INT_RED_LO, colorGain);
			writeInt(INT_GREEN_LO, colorGain);
			writeInt(INT_BLUE_LO, colorGain);

			int maxRead = 0;
			int minRead = 4096;
			int red = 0;
			int green = 0;
			int blue = 0;

			for (int i = 0; i < 4; i++) {
				performMeasurement();
				red += readRegisterInt(DATA_RED_LO);
				green += readRegisterInt(DATA_GREEN_LO);
				blue += readRegisterInt(DATA_BLUE_LO);
			}
			red /= 4;
			green /= 4;
			blue /= 4;

			maxRead = max(maxRead, red);
			maxRead = max(maxRead, green);
			maxRead = max(maxRead, blue);

			minRead = min(minRead, red);
			minRead = min(minRead, green);
			minRead = min(minRead, blue);

			int diff = maxRead - minRead;

			if (oldDiff != diff) {
				if ((maxRead == red) && (calibrationRed < 15))
					calibrationRed++;
				else if ((maxRead == green) && (calibrationGreen < 15))
					calibrationGreen++;
				else if ((maxRead == blue) && (calibrationBlue < 15))
					calibrationBlue++;
			} else
				calibrated = 1;

			oldDiff = diff;

//	    int rCal = calibrationRed;
//	    int gCal = calibrationGreen;
//	    int bCal = calibrationBlue;
		}

	}

};

#endif /* ADJD_S311_H_ */
