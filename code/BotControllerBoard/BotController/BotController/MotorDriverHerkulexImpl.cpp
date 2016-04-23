/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#include "Arduino.h"
#include "MotorDriver.h"
#include "BotMemory.h"

#define HERKULEX_MOTOR_ID 1 // out motor is 1(first)
#define HERKULEX_BROADCAST_ID 0xfe



void MotorDriverHerkulexImpl::setup(int motorNumber, long baudrate) {
	// initialize Herkulex servo
	herkulexServo.beginSerial1(baudrate);		// baud rate is detected automatically by herkulex servo
	herkulexServo.reboot(HERKULEX_MOTOR_ID);	//reboot our motor
	delay(500);
	herkulexServo.initialize(); //initialize motors
	delay(200);
	// check if servo is reacting
	float currentAngle = herkulexServo.getAngle(HERKULEX_MOTOR_ID);
	Serial.print("herkulex Servo as angle");
	Serial.print(currentAngle,1);
	Serial.println();
	
	MotorDriver::setup(motorNumber);
} //MotorDriver


void MotorDriverHerkulexImpl::setRawAngle(float pAngle, long pDuration_ms) {
	Serial.print(F("M["));
	Serial.print(myMotorNumber);
	Serial.print(F("]="));
	Serial.print(pAngle,1);
	Serial.println();

	float calibratedAngle = pAngle + config->nullAngle;
	if ((calibratedAngle >= -160.0) || (calibratedAngle <= 160.0)) {
		herkulexServo.moveOneAngle(HERKULEX_MOTOR_ID, calibratedAngle, pDuration_ms, LED_BLUE); 
	}
}

float MotorDriverHerkulexImpl::getRawAngle() {
	return herkulexServo.getAngle(HERKULEX_MOTOR_ID);
}

void MotorDriverHerkulexImpl::loop() {
	MotorDriver::loop();
}




