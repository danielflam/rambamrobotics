/*
 * HC_SR04.cpp
 *
 *  Created on: Apr 11, 2014
 *      Author: Danny
 */

#include <HC_SR04.h>

FrequencyTimer2 SonarBase::sonarTimer;
SonarBase * SonarBase::currentSonar = NULL;

void SonarBase::activate() {
	currentSonar = this;
}

void SonarBase::onSonarUpdate() {
	currentSonar->update();
}

void SonarBase::setup() {
	if (!currentSonar) {
		currentSonar = this;

		// Update Sonar every 8uSec
		sonarTimer.setPeriod(80); // 7 uSecs - about 3mm minimum break
		sonarTimer.setOnOverflow(SonarBase::onSonarUpdate);
	}
}

