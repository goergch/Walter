/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#include "Arduino.h"
#include "Actuator.h"
#include "BotMemory.h"

float HerkulexServoDrive::readCurrentAngleFromServo() {
	uint16_t inputVoltage = 0;
	int16_t position = 0.0;
	if (servo)
		servo->getBehaviour(&inputVoltage, HKX_NO_VALUE, &position, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE);
	float voltage = (float)inputVoltage/1000;
	return (float)position/10;
}

void HerkulexServoDrive::setup(ServoConfig& pConfigData, ServoSetupData& pSetupData) {
	configData = &pConfigData;
	setupData = &pSetupData;

	movement.setNull();

	// initialize Herkulex servo
	HkxPrint* printout = new HkxPrint(Serial, CONNECTION_BAUD_RATE);  // No printout with Arduino UNO
	HkxCommunication* communication = new HkxCommunication(HKX_115200, Serial1, *printout);  // Communication with the servo on Serial1
	servo = new HkxPosControl(pSetupData.herkulexMotorId, *communication, *printout);  // control position for the servo ID=253 (factory default value)
	delay(50);
	servo->reboot();
	delay(200);

	// update current angle
	mostRecentAngle = readCurrentAngleFromServo();
	
} //setup

void HerkulexServoDrive::changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) {
	// this methods works even when no current Angle has been measured
	movement.set(getCurrentAngle(), getCurrentAngle()+pAngleChange, millis(), pAngleTargetDuration);
}


void HerkulexServoDrive::setAngle(float pAngle,uint32_t pAngleTargetDuration) {
	uint32_t now = millis();
	
	// limit angle
	pAngle = constrain(pAngle, configData->minAngle,configData->maxAngle);

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

void HerkulexServoDrive::moveToAngle(float pAngle, uint32_t pDuration_ms) {
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
	float calibratedAngle = pAngle + configData->nullAngle;
	if ((calibratedAngle >= -160.0) || (calibratedAngle <= 160.0)) { 
		servo->movePosition(pAngle*10.0, pDuration_ms, HKX_LED_BLUE, false); 
		mostRecentAngle = pAngle;
	}
}

float HerkulexServoDrive::getCurrentAngle() {
	return mostRecentAngle;
}

void HerkulexServoDrive::loop(uint32_t now) {
	if (!movement.isNull()) {
		// PID controller is already built in the servo
		float toBeAngle = movement.getCurrentAngle(now);
		moveToAngle(toBeAngle, SERVO_SAMPLE_RATE); // stay at same position after this movement
	}
}




