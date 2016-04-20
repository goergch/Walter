/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: SuperJochenAlt
*/


#include "MotorDriver.h"

#define HERKULEX_MOTOR_ID 1 //motor ID - verify your ID !!!!



void MotorDriverHerkulexImpl::setup(long baudrate) {
	// initialize Herkulex servo
	herkulexServo.beginSerial1(baudrate); 
	herkulexServo.reboot(HERKULEX_MOTOR_ID); //reboot first motor
	delay(500);
	herkulexServo.initialize(); //initialize motors
	delay(200);
} //MotorDriver


void MotorDriverHerkulexImpl::setAngle(float pAngle, long pDuration_ms) {
	if ((pAngle >= -160.0) || (pAngle <= 160.0)) {
		currentAngle = pAngle;
		herkulexServo.moveOneAngle(HERKULEX_MOTOR_ID, pAngle, pDuration_ms, LED_BLUE); 
	}
}

float MotorDriverHerkulexImpl::getAngle() {
	return currentAngle;	
}
