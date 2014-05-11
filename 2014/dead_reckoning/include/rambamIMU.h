/*
 * rambamIMU.h
 *
 *  Created on: Apr 6, 2014
 *      Author: Danny
 */

#ifndef RAMBAMIMU_H_
#define RAMBAMIMU_H_

#include <kalman.h>
#include <stdint.h>

class RambamIMU {
public:
	float pitch, roll, yaw;
	float q0, q1, q2, q3;

public:
	RambamIMU() {
		HEX_ZERO = 0x00;
		loopCount = 0;
	}

	void setup();
	void loop();
	void print();
	void GetEuler(void);
	void AHRSupdate();

private:
//	void Smoothing(int16_t *raw, float *smooth){
//	  *smooth = (*raw * (0.15)) + (*smooth * 0.85);
//	}

	void Init();
	float fastAtan2(float y, float x);
	float invSqrt(float number);
	void GetMag();
	void GetAcc();
	void GetGyro();

	void writeReg(int addr, int reg, int val);

	int16_t accX, accY, accZ;
	int16_t gyroX, gyroY, gyroZ;
	int16_t magX, magY, magZ;
	float scaledGyroX, scaledGyroY, scaledGyroZ;
	int16_t offsetX, offsetY, offsetZ;
	int32_t gyroSumX, gyroSumY, gyroSumZ;

	KALMAN smoothX, smoothY, smoothZ;

	//float accToFilterX, accToFilterY, accToFilterZ;
	float floatMagX, floatMagY, floatMagZ; //needs to be a float so the vector can be normalized



	//Wire does not like 0x00
	uint8_t HEX_ZERO;

	long timer, printTimer;
	float dt;
	uint32_t loopCount; // = 0;

//	float beta = betaDef;

	float magnitude;

	float gx;
	float gy;
	float gz;
	float ax;
	float ay;
	float az;
	float mx;
	float my;
	float mz;

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
			_2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2,
			q1q3, q2q2, q2q3, q3q3;

};

#endif /* RAMBAMIMU_H_ */
