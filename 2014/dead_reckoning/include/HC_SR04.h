/*
 * HC_SR04.h
 *
 *  Created on: Apr 7, 2014
 *      Author: Danny
 */

#ifndef HC_SR04_H_
#define HC_SR04_H_

#include <Arduino.h>
#include <stopwatch.h>
#include <kalman.h>
#include <DigitalIO/DigitalPin.h>
#include <stdint.h>
#include <FrequencyTimer2.h>

class SonarBase {
public:

	SonarBase() :
			delay_uSec(10000 /*uSec*/) {
		state = HC_SR04_STATE_0_PRERUN;
		rawDistance = -1.0;
		running = false;
	}

	virtual ~SonarBase() {
	}

	enum State {

		HC_SR04_STATE_0_PRERUN,
		HC_SR04_STATE_1_TRIG,
		HC_SR04_STATE_2_ECHO_WAIT,
		HC_SR04_STATE_3_ECHO_HIGH,
		HC_SR04_STATE_4_TIMEOUT,
		HC_SR04_STATE_5_ECHO_PROBLEM,
		HC_SR04_STATE_6_READY
	};

//    virtual float getDistance()
	//   {
	//  	return 334.0 * distance()/2000000.0;
	//  }

	virtual float getRawDistance() {
		return rawDistance;
	}

	bool isRunning() const {
		return running;
	}

	void start() {
		running = true;
	}

	State getState() const {
		return state;
	}

	void setState(State _state) {
		state = _state;
	}

	virtual void update() = 0;

	virtual void setup();

	static void onSonarUpdate();

	void activate();

	void log() {
		Serial.print("sonar(");
		Serial.print(state);
		Serial.print("): - ");
		Serial.print(rawDistance);
		Serial.print(" timestamp ");
		Serial.println(delay_uSec.timeSinceReset());
	}

protected:
	StopwatchMicroseconds delay_uSec;

//    volatile KALMAN distance;
	volatile long rawDistance;
	volatile State state;
	volatile bool running;

	static bool sonarInit;
	static FrequencyTimer2 sonarTimer;
	static SonarBase * currentSonar;

};

template<byte trigpin, byte echopin>
class HC_SR04: public SonarBase {
	//byte echopin;//, trigpin;

public:
	HC_SR04() :
			SonarBase() {

	}

	virtual ~HC_SR04() {
	}

	void setup() {

		pinMode(trigpin, OUTPUT);
		digitalWrite(trigpin, LOW);
		pinMode(echopin, INPUT);

//		fastPinConfig(trigpin, OUTPUT, LOW);
//		fastPinMode(echopin, INPUT);
		delay_uSec.reset();

		SonarBase::setup();

	}

	virtual void update() {
		switch (state) {
		case HC_SR04_STATE_0_PRERUN:
			doState0();
			break;
		case HC_SR04_STATE_1_TRIG:
			doState1();
			break;
		case HC_SR04_STATE_2_ECHO_WAIT:
			doState2();
			break;
		case HC_SR04_STATE_3_ECHO_HIGH:
			doState3();
			break;
		case HC_SR04_STATE_4_TIMEOUT:
			doState4();
			break;
		case HC_SR04_STATE_5_ECHO_PROBLEM:
			doState5();
			break;
		case HC_SR04_STATE_6_READY:
			doState6();
			break;
		}
	}

protected:

	inline void doState0() __attribute__((always_inline))
	{
		if (delay_uSec.timeout(false)) {
			if (fastDigitalRead(echopin) == HIGH) {
				//		rawDistance = -1.0;
				state = HC_SR04_STATE_5_ECHO_PROBLEM;
			} else {
				delay_uSec.setInterval(8);
				delay_uSec.reset();
				fastDigitalWrite(trigpin, HIGH);
				state = HC_SR04_STATE_1_TRIG;
			}
		}
	}

	inline void doState1() __attribute__((always_inline))
	{
		if (delay_uSec.timeout(false)) {
			if (fastDigitalRead(echopin) == HIGH) {
				//	rawDistance = -1.0;
				state = HC_SR04_STATE_5_ECHO_PROBLEM;
			} else {
				delay_uSec.setInterval(50000);
				fastDigitalWrite(trigpin, LOW);
				delay_uSec.reset();
				state = HC_SR04_STATE_2_ECHO_WAIT;
			}
		}
	}

	inline void doState2() __attribute__((always_inline))
	{
		if (fastDigitalRead(echopin) == HIGH) {
			state = HC_SR04_STATE_3_ECHO_HIGH;
		} else if (delay_uSec.timeout(false)) {
			delay_uSec.setInterval(50000);
			delay_uSec.reset();
			rawDistance = -1.0;
			state = HC_SR04_STATE_4_TIMEOUT;
		}
	}

	inline void doState3() __attribute__((always_inline))
	{
		if (fastDigitalRead(echopin) == LOW) {
			rawDistance = delay_uSec.timeSinceReset();
			//distance.update(rawDistance);
			running = false;
			delay_uSec.setInterval(1000);
			delay_uSec.reset();
			state = HC_SR04_STATE_0_PRERUN;
		} else if (delay_uSec.timeout(false)) {
			// flush the ping
			delay_uSec.setInterval(50000);
			delay_uSec.reset();
			state = HC_SR04_STATE_4_TIMEOUT;
		}
	}

	inline void doState4() __attribute__((always_inline))
	{
		if (delay_uSec.timeout(false)) {
			//rawDistance = -1.0;
			state = HC_SR04_STATE_5_ECHO_PROBLEM;
		}
	}

	inline void doState5() __attribute__((always_inline))
	{
		if (fastDigitalRead(echopin) == LOW) {
			running = false;
			delay_uSec.setInterval(1000);
			delay_uSec.reset();
			state = HC_SR04_STATE_0_PRERUN;
		}
	}

	inline void doState6() __attribute__((always_inline))
	{
		if (running) {
			state = HC_SR04_STATE_0_PRERUN;
		}
	}
};

#endif /* HC_SR04_H_ */
