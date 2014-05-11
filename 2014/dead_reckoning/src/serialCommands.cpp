/*
 * serialCommands.cpp
 *
 *  Created on: Apr 11, 2014
 *      Author: Danny
 */


#include "serialCommands.h"
#include "appData.h"


// forward declaration
extern SerialCommand commandProcessor;


void commandHelp()
{
	Serial.println(F("h => help"));
}

void commandDefault(const char * line)
{
	Serial.print("Syntax Error '");
	Serial.print(line);
	Serial.print("'");

	commandHelp();
}

void initCommandProcessor()
{
	commandProcessor.setDefaultHandler(commandDefault);
}

void printParticles()
{
	appData.particleFilter.log();
}

void printObservation()
{
	appData.observation.log();
}

void printSonar()
{
	appData.sonar1.log();
}

//void commandImuEuler() {
//	//imu.print();
//}
//
//void commandImuVersion() {
////	char str[128] = ;
//
////	snprintf_P(str, sizeof(str),
////			PSTR("FreeIMU library by %s, FREQ:%s, LIB_VERSION: %s, IMU: %s"),
////			FREEIMU_DEVELOPER, FREEIMU_FREQ, FREEIMU_LIB_VERSION, FREEIMU_ID
////	);
//
////	Serial.println(F("RambamIMU 1.0V"));
//}
//
//void commandMeasure() {
//	//imu.print();
//
//	//colorSensor.getYUV(); // After calibrating, we can get the first RGB and C data readings
//	//colorSensor.print(); // Formats and prints all important ADJD-S311 registers
//
////	for (int i=0;i<4;i++)
////			colors[i].update(colorSensor.colorYUV[i]);
////	Serial.print("YUV: \t\t ");
////	for (int i = 0; i < 4; i++) {
////		Serial.print(colors[i]());
////		Serial.print("\t ");
////	}
//
////	Serial.print(sonar1.getDistance());
////	Serial.print(" ");
//	Serial.println(appData.sonar1.getDistance());
//
//	Serial.println();
//
//}
//
//void commandCalibrate() {
////	Serial.println(
////			"\nHold up a white object in front of the sensor, then press any key to calibrate...\n");
////	while (!Serial.available())
////		;  // Wait till a key is pressed
////	Serial.flush();
////
////	Serial.println("\nCalibrating...this may take a moment\n");
////
////	//colorSensor.calibrate();
////
////	Serial.println("Calibration complete... measuring...");
////	commandMeasure();
//
////	Serial.println(
////			"\nAll values should be under 1000. If they're not, try calibrating again, or decreasing the ambient brightness somehow. ");
////	Serial.println(
////			"\nPress SPACE to read, \"c\" to calibrate, \"o\" to get offset, \"l\" to go to LED mode");
//
//}
//
////
//void commandContinuous() {
////	Serial.println(F("Press 's' to stop"));
////
////	continuous = true;
//}
////
//
//void commandContinuousStop() {
////	continuous = false;
//}


SerialCommandCallback commands[] = {
		{"p", printParticles},
		{"o", printObservation},
		{"h", commandHelp},
		{"s", printSonar},
		{NULL, NULL}
};


SerialCommand commandProcessor(commands);


void executeSerialCommands()
{
	while (Serial.available()) {
		char c = Serial.read();
		Serial.print(c); // echo
		commandProcessor.update(c);
	}
}
