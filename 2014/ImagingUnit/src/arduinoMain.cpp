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
#include <Servo/servo.h>
#include <math.h>
#include <kalman.h>
#include <WiiCam.h>
#include <soccerPID.h>
#include <Wire.h>

#include <SoftI2C.h>

KALMAN direction[10];
Stopwatch reportingDelay(300);
Servo camServo;
KALMAN curpos;
WiiCam cam;
PID pid;

struct ImagingInfo
{
	uint8_t WiicamAvgX;
	uint8_t WiicamAvgY;
	uint8_t IRDetectorsDir;
	uint8_t ServoAngle;
};

float servoStep = 0.3;

float currentServo = 140;

bool active = false;

void commandOn();
void commandServo();
void commandOff();
void commandDefault(const char * line);
void commandTest();
void commandTestRandom();

SerialCommandCallback commandList[] = {
		{ "test", commandTest },
		{ "random", commandTestRandom },
		{ "on", commandOn },
		{ "off", commandOff },
		{ "s", commandServo },
		{ "off", commandOff },
		{ NULL,NULL } };

SerialCommand commandProcessor(commandList);


void commandOn() {

	active = true;
}

void commandServo() {
	camServo.write(commandProcessor.getParam(0));
}

void commandOff() {
	active = false;
}

void commandDefault(const char * line) {

}

void commandTest() {
}

void commandTestRandom() {
}

Stopwatch infoDelay;

void onWireReceive(int bytes)
{
	byte regAddr = Wire.read();
	byte data = Wire.read();

	switch (regAddr)
	{
	case 0:
		active = data != 0;

	}
}

void onWireRequest()
{
	ImagingInfo info;
	info.IRDetectorsDir = map(curpos.getX(), 0,10, 0,255);
	info.ServoAngle = currentServo;
	info.WiicamAvgX = map(cam.avgX, 0, 1023, 0, 255);
	info.WiicamAvgX = map(cam.avgY, 0, 1023, 0, 255);
	Wire.write((uint8_t*) &info, sizeof(info));
}

void setup() {

	currentServo = 80;

	Serial.begin(9600);

	pinMode(A0, INPUT);
	pinMode(A1, INPUT);

	i2c_init();

	cam.setup();
	pid.mySetpoint = 512;
	pid.SetOutputLimits(-1000,1000);
	pid.setup(5,0.1,0.0001,DIRECT, 0.001);
	pid.SetMode(AUTOMATIC);

	commandProcessor.setDefaultHandler(&commandDefault);

	delayMicroseconds(1); // make sure the microsecond delay isnt optimised away!

	//i2cCam.

	camServo.attach(12, 10,170);
	camServo.write(160);  //horizontal up.

	for (int i = 0; i<10;i++)
	{
		pinMode(i + 2, INPUT);
	}

	Wire.begin(7);
	Wire.onReceive(onWireReceive);
	Wire.onRequest(onWireRequest);
	delay(1);  //

}

void updateIR()
{
	// update the IR string
	for (int i = 0; i<10; i++)
	{
		float pinVal = 1.0-digitalRead(i+2);

		direction[i].update(pinVal);
	}


	double maxval = 0;
	int maxpos = 0;

	for (int i=0;i<10;i++)
	{
		if (direction[i].getX() > maxval )
		{
			maxval = direction[i].getX();
			maxpos = i;
		}
	}
	curpos.update(float(maxpos));
}

void loop() {
	// process serial in...
	while (Serial.available()) {
		char c = Serial.read();
		commandProcessor.update(c);
	}

	if (cam.update())
	{
		// we are not doing much here
		cam.afterNewDataProcessed();
	}


	pid.Compute(cam.avgY);

	if (cam.avgY == 0)
	{
		currentServo += servoStep;
		if (currentServo >= 170)
		{
			servoStep = -0.4;
		}
		else if (currentServo < 10)
		{
			servoStep = 0.4;
		}

	}
	else
	{
		currentServo += pid.myOutput;
		if (currentServo > 170)
			currentServo  = 170;
		else if (currentServo < 10)
			currentServo  = 10;
	}
	if (active)
		camServo.write( currentServo );

	updateIR();

	if (reportingDelay.timeout(true))
	{

		pid.log();
		Serial.println(currentServo);
//		cam.log();

		/*
		for (int i=0;i<10;i++)
		{
			Serial.print(direction[i].getX());
			Serial.print(" ");
		}


		Serial.print("| ");
		 */

		//float angle = 20.0 + curpos.getX()*30.0 - 180.0;
		//Serial.println(angle);
	}
}
