/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#include "Arduino.h"
#include "MotorDriver.h"
#include "BotMemory.h"



void MotorDriverHerkulexImpl::setup(int motorNumber) {
	// initialize Herkulex servo

	HkxPrint* printout = new HkxPrint();  // No printout with Arduino UNO
	HkxCommunication* communication = new HkxCommunication(HKX_115200, Serial1, *printout);  // Communication with the servo on Serial1
	servo = new HkxPosControl(253, *communication, *printout);  // control position for the servo ID=253 (factory default value)
	
	// check if servo is reacting
	float currentAngle = getRawAngle();
	
	MotorDriver::setup(motorNumber);
} //MotorDriver


void MotorDriverHerkulexImpl::setRawAngle(float pAngle, long pDuration_ms, float pNextAngle, long pNextDuration_ms) {
	/*Serial.print(F("M["));
	Serial.print(myMotorNumber);
	Serial.print(F("]="));
	Serial.print(pNextAngle,1);
	Serial.println();
	*/
	float calibratedAngle = pAngle + config->nullAngle;
	if ((calibratedAngle >= -160.0) || (calibratedAngle <= 160.0)) {
		if (pAngle == pNextAngle)
			servo->movePosition(pAngle*10.0, pDuration_ms, HKX_LED_BLUE, false);  
		else
			servo->movePosition(pNextAngle*10.0, pDuration_ms, HKX_LED_BLUE, false); // keep moving according to next loop
	}
}

float MotorDriverHerkulexImpl::getRawAngle() {
	int16_t position;
	 servo->getBehaviour(HKX_NO_VALUE, HKX_NO_VALUE, &position, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE);
	return float(position)/10.0;
}

void MotorDriverHerkulexImpl::loop() {
	MotorDriver::loop();
}




