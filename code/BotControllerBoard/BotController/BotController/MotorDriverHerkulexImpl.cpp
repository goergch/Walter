/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#include "Arduino.h"
#include "MotorDriver.h"
#include "BotMemory.h"

#define HERKULEX_MOTOR_ID 1 //motor ID - verify your ID !!!!



void MotorDriverHerkulexImpl::setup(int motorNumber, long baudrate) {
	// initialize Herkulex servo
	herkulexServo.beginSerial1(baudrate); 
	herkulexServo.reboot(HERKULEX_MOTOR_ID); //reboot first motor
	delay(500);
	herkulexServo.initialize(); //initialize motors
	delay(200);
	
	MotorDriver::setup(motorNumber);
} //MotorDriver


void MotorDriverHerkulexImpl::setAngle(float pAngle, long pDuration_ms) {
	float calibratedAngle = pAngle + config->nullAngle;
	if ((calibratedAngle >= -160.0) || (calibratedAngle <= 160.0)) {
		currentAngle = pAngle;
		herkulexServo.moveOneAngle(HERKULEX_MOTOR_ID, calibratedAngle, pDuration_ms, LED_BLUE); 
	}
}

float MotorDriverHerkulexImpl::getAngle() {
	return currentAngle;	
}

void MotorDriverHerkulexImpl::loop() {
}




