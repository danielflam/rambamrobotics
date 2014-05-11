

#include "appData.h"
#include "fastRandom.h"
#include "Wire.h"

// Actual data instance
AppData appData;

Stopwatch reportDelay(5000); // 300 ms

struct NAVInfo
{
	uint8_t colorNotGreen; // seems to capture line (the V components of YUV)
	uint8_t colorClear; // the brightness
	uint8_t sonar; // in cm
	uint8_t heading; // 0-255 (remap to 0-360 if you want)
	uint8_t suspectedX; //
	uint8_t suspectedY; //
};

volatile NAVInfo info;
volatile bool twiRequestProcessed = false;

void onWireRequest()
{
 // Wire.write("hello "); // respond with message of 6 bytes
 // as expected by master
	info.colorNotGreen = appData.observation.colorNotGreen;
	info.colorClear = appData.observation.colorClear;
	float absHeading = map(appData.observation.headingMagnetic - appData.observation.initialHeadingMagnetic, 0, 2*PI, 0, 255);
	while (absHeading < 0)
		absHeading += 255.0;
	while (absHeading > 255)
		absHeading += 255.0;

	info.heading = uint8_t(absHeading);
	info.sonar = uint8_t(appData.observation.sonar1);
	// not good yet

	info.suspectedX = appData.particleFilter.currentLocationSortOf.x;
	info.suspectedY = appData.particleFilter.currentLocationSortOf.y;

	twiRequestProcessed = true;
	Wire.write((uint8_t*)&info, sizeof(info));

}

void setup() {

	pinMode(13, OUTPUT); // LED
	digitalWrite(13, LOW);
	long s = random();
	while (s == 0)
	{
		s = random();
	}

	fastRandomSeed(s);

	Serial.begin(9600);

	// power up sonar.
	pinMode(8, OUTPUT);
	pinMode(11, OUTPUT);
	digitalWrite(11, LOW);
	digitalWrite(8, HIGH);


	// BEGIN BUG
	// These next commands are here to overcome a bug in linker!
	delayMicroseconds(1); // make sure the microsecond delay isnt optimised away!
	micros();
	millis();
	delay(1);
	// END BUG

	appData.setup();
	//do NOT call before the sonar is setup!!
	//appData.sonarTimer.setOnOverflow(&onSonarUpdate);

	Serial.println("Init complete.");


	Wire.begin(5);
	Wire.onRequest(onWireRequest);

	digitalWrite(13, HIGH);
}


void loop() {
	//executeSerialCommands();
	appData.update();

//	if (twiRequestProcessed)
//	{
//		twiRequestProcessed = false;
//		Serial.println("TWI Processed...");
//		Serial.println(info.sonar);
//	}
//	if (reportDelay.timeout(true)){

//		appData.observation.log();
//		appData.sonar1.log();

	//}

}
