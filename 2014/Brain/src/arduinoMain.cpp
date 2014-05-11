/*
 ADJD-S311 Color Sensor Breakout Example Code
 by: Jim Lindblom
 SparkFun Electronics
 date: 8/9/11
 License: MIT license (http://www.opensource.org/licenses/mit-license.php)
 This code is slightly modified from that posted on bildr.org's
 excellent tutorial (http://bildr.org/2011/01/adjd-s371-tutorial/), which
 was taken from Marcus' great initial code
 (http://interactive-matter.eu/2008/08/tinkering-with-adjd-s371-q999/).
 Thanks to Adam and Marcus for the initial code! I'd definitely recommend checking out
 their tutorial/posts.

 This example code initializes and calibrates the sensor upon startup. When calibrating
 the code is assuming the sensor is faced with a stable WHITE source. To best calibrate
 the sensor's capacitor and integration registers, it needs to be looking at a usual white
 object at calibration.

 After start-up, the sensor will wait for serial input. Open up the serial
 monitor at 9600 bps. A space ' ', will prompt the sensor for all of it's register
 values. 'c' will perform calibration (make sure it's aimed at a white object. 'o'
 will get the offset values (these are not trimmed, see getOffset() function). And 'l'
 will continously get the sensor readings and output them to an RGB LED.

 You may want to try with the sensor's LED (connected to Arduino pin 2) both on and off.
 I usually get better results with the LED off, but it depends on the object being sensed.
 It does a really good job of sensing colors off my LCD monitor.

 the hookup:
 ADJD-S311 Breakout ------------- Arduino
 ----------------------------------------
 LED ---------------------------D2
 3.3V -------------------------3.3V
 GND -------------------------- GND
 SCL -------------------------- A5
 SDA -------------------------- A4
 GND -------------------------- GND
 SLP --------------------- Not connected
 CLK --------------------- Not connected
 */
#include <Arduino.h>
#include <CmdProcessor/cmdProcessor.h>
#include <stopwatch.h>
#include <Wire.h>
#include <stdint.h>
#include <Kalman.h>
#include <soccerPID.h>

//void commandDefault(const char * line);
//void commandTest();

struct ImagingInfo {
	uint8_t WiicamAvgX;
	uint8_t WiicamAvgY;
	uint8_t IRDetectorsDir;
	uint8_t ServoAngle;
} imagingInfo;

struct LocationInfo {
	uint8_t colorNotGreen; // seems to capture line (the V components of YUV)
	uint8_t colorClear; // the brightness
	uint8_t sonar; // in cm
	uint8_t heading; // 0-255 (remap to 0-360 if you want)
	uint8_t suspectedX; //
	uint8_t suspectedY; //
} navInfo;

void motorPower(bool onoff) {
	Wire.beginTransmission(6);
	Wire.write(0); // register 0
	Wire.write(onoff ? 0 : 1); // turn motors on // i screwed up on order 0 is on and 1 is off
	Wire.endTransmission();
}

void motorGo(float direction, float rotation, uint8_t power) {
	delay(10);  //
	Wire.beginTransmission(6);
	Wire.write(1); // register 0
	Wire.write(uint8_t(map(direction, 0, 359.9999, 0, 255.999)));
	Wire.write(uint8_t(map(rotation, 0, 359.9999, 0, 255.999)));
	Wire.write(power); // turn motors on
	Wire.endTransmission();

}

void activateImagingScanner(bool active) {
	Wire.beginTransmission(7);
	Wire.write(0); // register 0
	Wire.write(active);
	Wire.endTransmission();

}

void getImagingInfo(ImagingInfo & imagingInfo) {
	Wire.requestFrom(7, sizeof(imagingInfo));
	Wire.readBytes((char*) &imagingInfo, sizeof(imagingInfo));

}

void getLocationInfo(LocationInfo & loc) {
	Wire.requestFrom(5, sizeof(LocationInfo));
	Wire.readBytes((char*) &loc, sizeof(LocationInfo));

//      Serial.println(imagingInfo.WiicamAvgX);
//      Serial.println(imagingInfo.WiicamAvgY);
//      Serial.println(imagingInfo.ServoAngle);
}

void printImagingInfo() {
	Serial.println("Imaging Info");
	Serial.println(imagingInfo.IRDetectorsDir);
	Serial.println(imagingInfo.WiicamAvgX);
	Serial.println(imagingInfo.WiicamAvgY);
	Serial.println(imagingInfo.ServoAngle);
}

void printLocalInfo() {
	Serial.println("Localization Info");
	Serial.print(navInfo.colorNotGreen);
	Serial.print(" ");
	Serial.print(navInfo.colorClear);
	Serial.print(" ");
	Serial.print(navInfo.heading);
	Serial.print(" ");
	Serial.print(navInfo.sonar);
	Serial.print(" ");
	Serial.print(navInfo.suspectedX);
	Serial.print(" ");
	Serial.print(navInfo.suspectedY);
	Serial.print(" ");
	Serial.println();
}

void commandIMU();

SerialCommandCallback commandList[] = { { "i", commandIMU }, { NULL, NULL } };

SerialCommand commandProcessor(commandList);

void commandDefault(const char * line) {
	Serial.print("Unknown command '");
	Serial.print(line);
	Serial.println("'");
}

void commandIMU() {
	Serial.println("TEST IMU");

	getLocationInfo(navInfo);
	printLocalInfo();
}

void setup() {
}



void setupSequence() {

	pinMode(6,INPUT_PULLUP);
	commandProcessor.setDefaultHandler(&commandDefault);

	delayMicroseconds(1); // make sure the microsecond delay isnt optimised away!

	Serial.begin(9600);
	Serial.println("Initialized.");

	Wire.begin();

	delay(10);  //

	Serial.println("Initializing imaging unit");
	activateImagingScanner(true);
	delay(3000);  //
	activateImagingScanner(false);
	getImagingInfo(imagingInfo);
	printImagingInfo();

	Serial.println("Initializing localization");
	getLocationInfo(navInfo);
	printLocalInfo();

	Serial.println("Arming Motors");

	motorPower(false);
	delay(1000);  //

	motorPower(true);

	Serial.println("Init complete");
}

int state = 0;
Stopwatch initTimer(3000);
Stopwatch updateTimer(5);
KALMAN sonarDistance;

void updateSensors()
{
	if (updateTimer.timeout(true))
	{
		getImagingInfo(imagingInfo);
		getLocationInfo(navInfo);

		sonarDistance.update(navInfo.sonar);
	}
}

void doState0() {
	if (initTimer.timeout(false)) {
		setupSequence();
		activateImagingScanner(true);
		state = 3;
	}
}


void doState1() {
	if (imagingInfo.WiicamAvgX > 0 &&  imagingInfo.WiicamAvgX < 255)
	{
		state = 2;
	}
//	else if ( abs(int(imagingInfo.IRDetectorsDir) - 128 ) < 5  )
//	{
//		state = 2;
//	}
	else
	{
		motorGo(50, 250, 200);
	}
}

void doState2() {

	bool wiiSeesBall = (imagingInfo.WiicamAvgX > 0 &&  imagingInfo.WiicamAvgX < 255);
	bool irSeesBall = ( abs(int(imagingInfo.IRDetectorsDir) - 128 ) < 5 );

	bool seeBall = wiiSeesBall;

	if (!seeBall)
	{
		state = 1;
		return;
	}

	if (wiiSeesBall)
	{

		if (imagingInfo.WiicamAvgX > 100)
		{
			motorGo(30, 190, 130 );
		}
		else if (imagingInfo.WiicamAvgX <  150)
		{
			motorGo(30, 120, 130 );
		}
		else
		{
			motorGo(200, 0, 130 );
		}
	}
	else
	{
		int rot = imagingInfo.IRDetectorsDir;
		motorGo(5, rot, 180);
	}
}

void doState3() {
	if (digitalRead(6) == 0)
	{
		state = 1;
	}

}

void doState4() {

}

void doState5() {

}

void loop() {

	if (state != 0)
	{
		updateSensors();
	}

	switch (state) {
	case 0:
		doState0();
		break;

	case 1:
		doState1();
		break;

	case 2:
		doState2();
		break;
	case 3:
		doState3();
		break;

	case 4:
		doState4();
		break;
	case 5:
		doState5();
		break;
	}

	// process serial in...
//	while (Serial.available()) {
//		char c = Serial.read();
//		commandProcessor.update(c);
//	}

}

