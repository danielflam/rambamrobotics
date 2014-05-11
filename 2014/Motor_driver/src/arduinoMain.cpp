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
#include <soccerMotors.h>
#include <CmdProcessor/cmdProcessor.h>
#include <stopwatch.h>
#include <Wire.h>
SoccerMotors motors;

void commandOn();
void commandPower();
void commandOff();
void commandDefault(const char * line);
void commandTest();
void commandTestRandom();
void commandDriveRaw();
void commandDriveHolonomic();
void commandDriveDirectional();

SerialCommandCallback commandList[] = { { "power", commandPower }, { "test",
		commandTest },
		//{ "random", commandTestRandom },
		{ "on", commandOn }, { "off", commandOff }, { "r", commandDriveRaw }, {
				"h", commandDriveHolonomic }, { "d", commandDriveDirectional },
		{ NULL, NULL } };

SerialCommand commandProcessor(commandList);

void commandOn() {
	Serial.println("ON");
	digitalWrite(2, LOW);
}

void commandPower() {

	if (commandProcessor.getParamCount() != 1) {
		Serial.println("Syntax: power <val>");
	} else {
		Serial.print("Motor power set to ");
		Serial.println(commandProcessor.getParam(0));
		motors.setMotorPower(commandProcessor.getParam(0));
	}
}

void commandOff() {
	Serial.println("OFF");
	digitalWrite(2, HIGH);
}

void commandDefault(const char * line) {
	Serial.print("Unknown command '");
	Serial.print(line);
	Serial.println("'");
}

void commandTest() {
	Serial.println("TEST");
	motors.testMotors();
}

void commandTestRandom() {
	Serial.println("RANDOM");
	motors.moveRandom();
}

void commandDriveRaw() {
	Serial.println("RAW");
	if (commandProcessor.getParamCount() != 4
			|| commandProcessor.getParamCount() != 1) {
		Serial.println("r <m1> <m2> <m3> <m4>");
		Serial.println("r <power>");
		return;
	}

	if (commandProcessor.getParamCount() == 4) {
		motors.rawDrive(commandProcessor.getParam(0),
				commandProcessor.getParam(1), commandProcessor.getParam(2),
				commandProcessor.getParam(3));
	} else {
		motors.rawDrive(commandProcessor.getParam(0),
				commandProcessor.getParam(0), commandProcessor.getParam(0),
				commandProcessor.getParam(0));
	}
}

void commandDriveHolonomic() {
	Serial.println("Holonomic");
	if (commandProcessor.getParamCount() != 4) {
		Serial.println("h <X> <Y> <Dir> <Power>");
		return;
	}

	motors.holonomicDrive(commandProcessor.getParam(0),
			commandProcessor.getParam(1), commandProcessor.getParam(2),
			commandProcessor.getParam(3));
}

void commandDriveDirectional() {
	Serial.println("Directional");
	if (commandProcessor.getParamCount() != 3) {
		Serial.println("d <heading> <power> <rotation>");
		return;
	}

	motors.holonomicDrive(commandProcessor.getParam(0),
			commandProcessor.getParam(1), commandProcessor.getParam(2),
			commandProcessor.getParam(3));
}

struct MotorInfo {
	uint8_t power;
	uint8_t heading;
	uint8_t rot;
};

volatile bool newDataRecieved = false;
volatile uint8_t heading;
volatile uint8_t rot;
volatile uint8_t power;

void onWireReceive(int bytes) {
	volatile byte regAddr = Wire.read();
	volatile byte data;
	switch (regAddr) {
	case 0:
		data = Wire.read();
		digitalWrite(2, data == 0 ? LOW : HIGH);
		break;
	case 1:
		heading = Wire.read();
		rot = Wire.read();
		power = Wire.read();
		newDataRecieved = true;
		break;
	}
}

void onWireRequest() {
}

void setup() {
	pinMode(2, OUTPUT);
	digitalWrite(2, HIGH);

	motors.setup();

	commandProcessor.setDefaultHandler(&commandDefault);

	delayMicroseconds(1); // make sure the microsecond delay isnt optimised away!

	Serial.begin(9600);
	Serial.println("Initialized.");

	Wire.begin(6);
	Wire.onReceive(onWireReceive);
	Wire.onRequest(onWireRequest);

	delay(1);  //
}

void loop() {
	// process serial in...
	while (Serial.available()) {
		char c = Serial.read();
		commandProcessor.update(c);
	}

	if (newDataRecieved) {
		newDataRecieved = false;
		float h = float(heading);
		float r = float(rot);
		motors.directionalDrive(map(h, 0, 255, 0, 359.9999), power,
				r);

	}
}
