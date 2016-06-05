/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#include "Arduino.h"
#include "Actuator.h"
#include "BotMemory.h"



void MotorDriverHerkulexImpl::setup(int motorNumber) {
	// initialize Herkulex servo

	HkxPrint* printout = new HkxPrint(Serial, CONNECTION_BAUD_RATE);  // No printout with Arduino UNO
	HkxCommunication* communication = new HkxCommunication(HKX_115200, Serial1, *printout);  // Communication with the servo on Serial1
	servo = new HkxPosControl(HERKULEX_MOTOR_ID, *communication, *printout);  // control position for the servo ID=253 (factory default value)
	delay(50);
	servo->reboot();

	// update current angle
	mostRecentAngle = 0;
	
	Actuator::setup(motorNumber);
} //MotorDriver

void MotorDriverHerkulexImpl::setAngle(float pAngle,uint32_t pAngleTargetDuration) {
	uint32_t now = millis();
	static float lastAngle = 0;
	if (abs(lastAngle-pAngle)> 1) {
#ifdef DEBUG_HERKULEX		
		Serial.print("Herkulex.setAngle(");
		Serial.print(pAngle);
		Serial.print(" duration=");
		Serial.print(pAngleTargetDuration);
		Serial.println(") ");
#endif
		lastAngle = pAngle;
	}
	if (beforeFirstMove) {
		movement.set(pAngle, pAngle, now, pAngleTargetDuration);
		beforeFirstMove = true;
	}
	else {
		movement.set(getCurrentAngle(), pAngle, now, pAngleTargetDuration);
	}
}

void MotorDriverHerkulexImpl::moveToAngle(float pAngle, uint32_t pDuration_ms) {
#ifdef DEBUG_HERKULEX
static float lastAngle = 0;
	if (abs(lastAngle-pAngle)>0.1) {
		Serial.print("herkulex.moveToAngle(");
		Serial.print(pAngle);
		Serial.print(",");
		Serial.print(pDuration_ms);
		Serial.println(")");
		lastAngle = pAngle;
	}
#endif
	float calibratedAngle = pAngle + config->nullAngle;
	if ((calibratedAngle >= -160.0) || (calibratedAngle <= 160.0)) { 
		servo->movePosition(pAngle*10.0, pDuration_ms, HKX_LED_BLUE, false); 
		mostRecentAngle = pAngle;
	}
}

float MotorDriverHerkulexImpl::getCurrentAngle() {
	return mostRecentAngle;
}

void MotorDriverHerkulexImpl::loop(uint32_t now) {
	if (!movement.isNull()) {
		// PID controller is already built in the servo
		float toBeAngle = movement.getCurrentAngle(now);
		moveToAngle(toBeAngle, SERVO_SAMPLE_RATE); // stay at same position after this movement
	}
}




