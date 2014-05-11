/*
 * SoftSCCB.h
 *
 *  Created on: Mar 30, 2014
 *      Author: Danny
 */

#ifndef SOFTSCCB_H_
#define SOFTSCCB_H_


#include <Arduino.h>

#define SIO_C 2
#define SIO_D 4
#define SIO_CLOCK_DELAY 100

class SoftSCCB {
public:
	void Init(void)
			{
		pinMode(SIO_C, OUTPUT);
		pinMode(SIO_D, OUTPUT);
		digitalWrite(SIO_C, HIGH);
		digitalWrite(SIO_D, HIGH);
//		Serial.println("InitSCCB - PortDirectionSet & Set High OK");
	}

	void Start(void) {
		Serial.println("StartSCCB");

		digitalWrite(SIO_D, HIGH);
		delayMicroseconds (SIO_CLOCK_DELAY);
		digitalWrite(SIO_C, HIGH);
		delayMicroseconds(SIO_CLOCK_DELAY);
		digitalWrite(SIO_D, LOW);
		delayMicroseconds(SIO_CLOCK_DELAY);
		digitalWrite(SIO_C, LOW);
		delayMicroseconds(SIO_CLOCK_DELAY);
	}

	void Stop(void) //SCCBストップ
			{
		//Serial.println("StopSCCB");

		digitalWrite(SIO_D, LOW);
		delayMicroseconds (SIO_CLOCK_DELAY);
		digitalWrite(SIO_C, HIGH);
		delayMicroseconds(SIO_CLOCK_DELAY);
		digitalWrite(SIO_D, HIGH);
		delayMicroseconds(SIO_CLOCK_DELAY);
	}

	char Write(byte m_data) {
		unsigned char j, tem;

		//Serial.print("SCCBWrite 0x");
		//Serial.println(m_data,HEX);

		//Serial.print("SCCBWrite");
		for (j = 0; j < 8; j++)
				{
			if ((m_data << j) & 0x80) {
				digitalWrite(SIO_D, HIGH);
			} else {
				digitalWrite(SIO_D, LOW);
			}
			delayMicroseconds (SIO_CLOCK_DELAY);
			digitalWrite(SIO_C, HIGH);
			delayMicroseconds(SIO_CLOCK_DELAY);
			digitalWrite(SIO_C, LOW);
			delayMicroseconds(SIO_CLOCK_DELAY);
		}

		//Serial.println("");
		//delayMicroseconds(SIO_CLOCK_DELAY);
		digitalWrite(8, LOW); //debug
		pinMode(SIO_D, INPUT);
		digitalWrite(SIO_D, LOW);
		delayMicroseconds (SIO_CLOCK_DELAY);

		digitalWrite(8, HIGH); //debug
		digitalWrite(SIO_C, HIGH);
		delayMicroseconds(SIO_CLOCK_DELAY);

		//Serial.println(" Write done");
		digitalWrite(8, LOW); //debug
		if (digitalRead(SIO_D) == HIGH) {
			//SIO_D=Hなら失敗
			tem = 0;
			Serial.println("SCCBWrite NG");
		} else {
			//SIO_D=Lなら成功
			tem = 1;
			//Serial.println("SCCBWrite OK");
		}
		digitalWrite(SIO_C, LOW);
		delayMicroseconds(SIO_CLOCK_DELAY);
		pinMode(SIO_D, OUTPUT); //SIO_Dのバスをマスター(Arduino)に戻す
		//delayMicroseconds(SIO_CLOCK_DELAY);
		//digitalWrite(SIO_D,LOW);
		//delayMicroseconds(SIO_CLOCK_DELAY);

		//pinMode(SIO_C,OUTPUT); //SIO_Cのバスをマスター(Arduino)に戻す

		return tem;
	}
};

#endif /* SOFTSCCB_H_ */
