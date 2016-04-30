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

	HkxPrint* printout = new HkxPrint(Serial, CONNECTION_BAUD_RATE);  // No printout with Arduino UNO
	HkxCommunication* communication = new HkxCommunication(HKX_115200, Serial1, *printout);  // Communication with the servo on Serial1
	servo = new HkxPosControl(253, *communication, *printout);  // control position for the servo ID=253 (factory default value)
	delay(50);
	servo->reboot();
	delay(100);

	// update current angle
	currentAngle = 0;
	lastAngleSetting = 0;
	updateCurrentAngle();
	
	MotorDriver::setup(motorNumber);
} //MotorDriver


void MotorDriverHerkulexImpl::setRawAngle(float pAngle, uint32_t pDuration_ms) {
	/*
		Serial.print("setRawAngle(");
		Serial.println(pAngle);
		Serial.print(",");
		Serial.print(pDuration_ms);
		Serial.println(")");
	*/
	float calibratedAngle = pAngle + config->nullAngle;
	if ((calibratedAngle >= -160.0) || (calibratedAngle <= 160.0)) { 
		servo->movePosition(pAngle*10.0, pDuration_ms, HKX_LED_BLUE, false); 
		lastAngleSetting = pAngle; 
	}
}

float MotorDriverHerkulexImpl::getRawAngle() {
	return currentAngle;
}

void MotorDriverHerkulexImpl::updateCurrentAngle() {
	int16_t position;
	uint16_t inputVoltage;
	servo->getBehaviour(&inputVoltage, HKX_NO_VALUE, &position, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE);
	currentAngle = float(position)/10.0;
}

void MotorDriverHerkulexImpl::loop() {
	if (!movement.isNull()) {
		float toBeAngle = movement.getCurrentAngle(millis());
		setRawAngle(toBeAngle, MOTOR_SAMPLE_RATE); // stay at same position after this movement
		currentAngle = toBeAngle;	
	}
}




