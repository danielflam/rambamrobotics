/*
 * appMain.h
 *
 *  Created on: Apr 11, 2014
 *      Author: Danny
 */

#ifndef APPMAIN_H_
#define APPMAIN_H_

#include "serialCommands.h"
#include <Arduino.h>

#include <stopwatch.h>

//#include <kalman.h>

#include <HC_SR04.h>
#include <softi2c.h>

#include <stdio.h>
#include <particleFilter.h>
#include <SoftI2cUtils.h>
//#include <adjd_s311.h>

#define ADJD_S311_ADDRESS (0x74 << 1)
#define ADXL435_ADDR (0x53 << 1)
#define ITG3200_ADDR (0x68 << 1)
#define HMC5883_ADDR (0x1E << 1)
//accel defines
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define FIFO_CTL 0x38
#define DATAX0 0x32
//gyro defines
#define POWER_MGMT 0x3E
#define SRD 0x15
#define DLPF 0x16
#define GYRO_XOUT_H 0x1D
#define GYRO_TO_RAD 0.001214142f //(14.375^-1 * pi/180)#define GYRO_TO_DEG 0.069565217f
//mag defines
#define CONFIG_REG_A (uint8_t)0x00
#define CONFIG_REG_B (uint8_t)0x01
#define MODE_REG (uint8_t)0x02
#define MAG_X_MSB (uint8_t)0x03

#define RED 0
#define GREEN 1
#define BLUE 2
#define CLEAR 3

// ADJD-S311's register list
#define ADJD_S311_CTRL 0x00
#define ADJD_S311_CONFIG 0x01
#define ADJD_S311_CAP_RED 0x06
#define ADJD_S311_CAP_GREEN 0x07
#define ADJD_S311_CAP_BLUE 0x08
#define ADJD_S311_CAP_CLEAR 0x09
#define ADJD_S311_INT_RED_LO 0xA
#define ADJD_S311_INT_RED_HI 0xB
#define ADJD_S311_INT_GREEN_LO 0xC
#define ADJD_S311_INT_GREEN_HI 0xD
#define ADJD_S311_INT_BLUE_LO 0xE
#define ADJD_S311_INT_BLUE_HI 0xF
#define ADJD_S311_INT_CLEAR_LO 0x10
#define ADJD_S311_INT_CLEAR_HI 0x11
#define ADJD_S311_DATA_RED_LO 0x40
#define ADJD_S311_DATA_RED_HI 0x41
#define ADJD_S311_DATA_GREEN_LO 0x42
#define ADJD_S311_DATA_GREEN_HI 0x43
#define ADJD_S311_DATA_BLUE_LO 0x44
#define ADJD_S311_DATA_BLUE_HI 0x45
#define ADJD_S311_DATA_CLEAR_LO 0x46
#define ADJD_S311_DATA_CLEAR_HI 0x47
#define ADJD_S311_OFFSET_RED 0x48
#define ADJD_S311_OFFSET_GREEN 0x49
#define ADJD_S311_OFFSET_BLUE 0x4A
#define ADJD_S311_OFFSET_CLEAR 0x4B



//#include <ADXL345.h>
//#include <HMC58X3.h>
//#include <ITG3200.h>

struct AppData {

	AppData() :
			particleUpdate(5), particleFilter(&soccerField), colorUpdate(2)

	{
		colorReadState = 0;
	}

	void performADJDMeasurement() {
		switch (colorReadState) {
		case 0:
			write(ADJD_S311_ADDRESS, (uint8_t) ADJD_S311_CTRL, 0x01); // start sensing
			colorReadState = 1;
			break;
		case 1:
			if (readRegister(ADJD_S311_ADDRESS, (uint8_t) ADJD_S311_CTRL)
					== 0) {
				observation.color.x = readLH(ADJD_S311_ADDRESS,
						ADJD_S311_DATA_RED_LO);
				observation.color.y = readLH(ADJD_S311_ADDRESS,
						ADJD_S311_DATA_GREEN_LO);
				observation.color.z = readLH(ADJD_S311_ADDRESS,
						ADJD_S311_DATA_BLUE_LO);
				observation.colorClear = readLH(ADJD_S311_ADDRESS,
						ADJD_S311_DATA_CLEAR_LO);
				colorReadState = 0;
			}
			break;
		}
	}
	void setup() {

		Serial.println("ParticleIMU 1.0V");

		initCommandProcessor();

		//turn on power to sonar 2
		pinMode(2, OUTPUT);
		digitalWrite(2, HIGH);

		Serial.println("Sonar init");
		sonar1.setup();
		//	sonar2.setup();

//		SonarBase::setup();

		delay(1); //

		i2c_init();
		delay(1); // Wait for ADJD reset sequence
		// speed up I2C to max...
		TWBR = 10;

		Setup_ADJD_S311();
		SetupIMU();

		//	colorSensor.init();
		//	delay(1);  //

		//	commandCalibrate();
		//	colorSensor.calibrate();
		//	colorSensor.getRGBC(); // After calibrating, we can get the first RGB and C data readings
		//	colorSensor.print(); // Formats and prints all important ADJD-S311 registers

		continuous = false;

		Serial.println("done.");
	}

	void writeReg(int addr, int reg, int val) {
		i2c_rep_start(addr);
		i2c_write(reg);
		i2c_write(val);
		i2c_stop();
	}

	void SetupIMU() {
		Serial.println("init ADXL345");

		writeReg(ADXL435_ADDR, BW_RATE, 0x0C);
		delay(5);
		writeReg(ADXL435_ADDR, POWER_CTL, 0x08);
		delay(5);
		writeReg(ADXL435_ADDR, DATA_FORMAT, 0x0B);

		writeReg(ITG3200_ADDR, POWER_MGMT, 0x80);
		delay(5);
		writeReg(ITG3200_ADDR, DLPF, 0x18);
		delay(5);

//		accelerometer.init(ADXL345_ADDR_ALT_LOW);

		Serial.println("init ITG3200");
		uint8_t HEX_ZERO = 0;
		writeReg(ITG3200_ADDR, SRD, HEX_ZERO);
		delay(5);

		writeReg(ITG3200_ADDR, POWER_MGMT, 0x03);

//		gyro.init(ITG3200_ADDR_AD0_LOW);

		delay(1000);
		// calibrate the ITG3200
//		Serial.println("Calibrate ITG3200");
		//gyro.zeroCalibrate(128, 5);

		// init HMC5843
		Serial.println("init HMC5886");
		write(HMC5883_ADDR, CONFIG_REG_A, (uint8_t) 0x1C);	//220Hz update rate
		write(HMC5883_ADDR, CONFIG_REG_B, (uint8_t) 0x60);		//+/- 2.5 gauss
		write(HMC5883_ADDR, MODE_REG, (uint8_t) 0x00);//continuous conversion mode

//		magnetometer.init(false); // Don't set mode yet, we'll do that later on.
//		// Calibrate HMC using self test, not recommended to change the gain after calibration.
//		magnetometer.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
//		// Single mode conversion was used in calibration, now set continuous mode
//		magnetometer.setMode(0);
//		delay(10);
//		magnetometer.setDOR(B110);

		Serial.println("Caibrating...");

		Vector3D curGyro;
		Vector3D curAccel;
		Vector3D curMagneto;
		Vector3D accGyro;
		Vector3D accAccel;
		Vector3D accMagneto;

		for (int i = 0; i < 64; i++) {
			GetGyro(curGyro);	//take samples to let the internal LPFs work
			GetAcc(curAccel);
			GetMag(curMagneto);
			delay(1);
		}
		for (int i = 0; i < 64; i++) {
			GetGyro(curGyro);
			accGyro += curGyro;

			GetAcc(curAccel);
			accAccel += curAccel;

			GetMag(curMagneto);
			accMagneto += curMagneto;

			delay(1);

		}
		accGyro >>= 6;
		gyroOffset = accGyro;
		accAccel >>= 6;
		accelOffset = accAccel;
		accMagneto >>= 6;

		observation.initialHeadingMagnetic = atan2(float(accMagneto.y),
				float(accMagneto.x));

		GetMag(curAccel); /// just for reading
		GetAcc(curAccel);

		updateSonar();
		updateIMU();

		observation.log();
		Serial.println("Imu initialized");
	}

	// Calibration data for color sensor
	//   				Red     Green   Blue    Clear
	//Data:            389     339     343     723
	//Caps:            15      12      7       5
	//Int:             201     201     201     111
	//Offset:          0       0       0       0
	//YUV:             354.41  121.21  152.57  370.60

	void Setup_ADJD_S311() {
		pinMode(6, OUTPUT); // LED
		digitalWrite(6, HIGH);

		Serial.println("Init ADJD S311");

		/*sensor gain registers, CAP_...
		 to select number of capacitors.
		 value must be <= 15 */
		write(ADJD_S311_ADDRESS, ADJD_S311_CAP_RED, 15);
		write(ADJD_S311_ADDRESS, ADJD_S311_CAP_GREEN, 12);
		write(ADJD_S311_ADDRESS, ADJD_S311_CAP_BLUE, 7);
		write(ADJD_S311_ADDRESS, ADJD_S311_CAP_CLEAR, 5);

		/* Write sensor gain registers INT_...
		 to select integration time
		 value must be <= 4096 */

		write(ADJD_S311_ADDRESS, ADJD_S311_INT_RED_LO, 201);
		write(ADJD_S311_ADDRESS, ADJD_S311_INT_RED_HI, 0);
		write(ADJD_S311_ADDRESS, ADJD_S311_INT_GREEN_LO, 201);
		write(ADJD_S311_ADDRESS, ADJD_S311_INT_GREEN_HI, 0);
		write(ADJD_S311_ADDRESS, ADJD_S311_INT_BLUE_LO, 201);
		write(ADJD_S311_ADDRESS, ADJD_S311_INT_BLUE_HI, 0);
		write(ADJD_S311_ADDRESS, ADJD_S311_INT_CLEAR_LO, 199);
		write(ADJD_S311_ADDRESS, ADJD_S311_INT_CLEAR_HI, 0);
		Serial.println("Complete");
	}

	void GetMag(Vector3D & vec) {
		uint8_t buf[6];

		if (!read(HMC5883_ADDR, MAG_X_MSB, buf, 6))
			return;

		vec.y = (((buf[0] << 8) | buf[1]));
		vec.z = -(((buf[2] << 8) | buf[3]));
		vec.x = (((buf[4] << 8) | buf[5]));
	}
	void GetAcc(Vector3D & vec) {
		uint8_t buf[6];

		if (!read(ADXL435_ADDR, DATAX0, buf, 6))
			return;

		vec.y = -(((buf[1] << 8) | buf[0]) + accelOffset.y);
		vec.x = -(((buf[3] << 8) | buf[2]) + accelOffset.x);
		vec.z = ((buf[5] << 8) | buf[4]) ;

	}

	void GetGyro(Vector3D & vec) {
		uint8_t buf[6];

		if (!read(ITG3200_ADDR, GYRO_XOUT_H, buf, 6))
			return;
		vec.y = (((buf[0] << 8) | buf[1]) - gyroOffset.y);
		vec.x = (((buf[2] << 8) | buf[3]) - gyroOffset.x);
		vec.z = -(((buf[4] << 8) | buf[5]) - gyroOffset.z);
	}

	void updateSonar() {
		//if (!sonar1.isRunning())
		//	sonar1.start();

		observation.sonar1 = sonar1.getRawDistance();
		if (observation.sonar1 >= -1.0) {
			// To cm based on measurement of the thing...
			observation.sonar1 = constrain(
					map(observation.sonar1, 700, 5900, 5, 150), 0, 244);
			//observation.sonar1 = constrain(observation.sonar1 * 0.043, 5, 200);
		}

	}

	void updateIMU() {
		GetAcc(observation.accelerometerRaw);
		GetGyro(observation.gyroRaw);
		GetGyro(observation.gyroRaw);
		GetMag(observation.magnetometerRaw);

		observation.processIMUReading();
	}

	void updateParticleFilter() {
		particleFilter.update(observation);

		particleFilter.getEstimatedLocation();
	}

	void update() {
		executeSerialCommands();

		if (colorUpdate.timeout(true)) {
			performADJDMeasurement();
		}

		if (particleUpdate.timeout(true)) {
			updateSonar();
			updateIMU();

			updateParticleFilter();
		}

	}

	bool continuous;

	byte colorReadState;
	Vector3D gyroOffset;
	Vector3D accelOffset;

	// IMU stuff
	//RambamIMU imu;

	//ADXL345 accelerometer;
	//ITG3200 gyro;
	//HMC58X3 magnetometer;

	WorldMap soccerField;
	ParticleFilter particleFilter;
	Observation observation;
	Stopwatch particleUpdate;
	Stopwatch colorUpdate;

	// sonar on pins 9 and 10
	HC_SR04<9, 10> sonar1;

	//HC_SR04<6,7> sonar2;

	//KALMAN colors[4];
	//ADJD_S311 colorSensor;

}
;

// could in as static member of class but calls would be too long!
extern AppData appData;

#endif /* APPMAIN_H_ */
