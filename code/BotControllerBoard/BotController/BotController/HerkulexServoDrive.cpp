/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#include "Arduino.h"
#include "Actuator.h"
#include "BotMemory.h"


void HerkulexServoDrive::setup(ServoConfig* pConfigData, ServoSetupData* pSetupData) {
	configData = pConfigData;
	setupData = pSetupData;

#ifdef DEBUG_HERKULEX
	Serial.println(F("setup servo"));
	Serial.print(F("   "));
	setupData->print();
	Serial.print(F("   "));
	configData->print();
#endif
	movement.setNull();

	delay(50);
	Herkulex.beginSerial1(115200);
	delay(10);
	Herkulex.reboot(pSetupData->herkulexMotorId); //reboot first motor
	delay(500);
	Herkulex.initialize(); //initialize motors
	delay(200);
	
	// update current angle
	float feedbackAngle;
	readFeedback(feedbackAngle, torque,overloadDetected, anyHerkulexError);
	currentAngle = feedbackAngle - configData->nullAngle;
	Serial.print("currentAngle=");
	Serial.print(currentAngle);
	setAngle(currentAngle, SERVO_SAMPLE_RATE);
} //setup

void HerkulexServoDrive::changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) {
#ifdef DEBUG_HERKULEX
	Serial.print("Herkulex.changeAngle(");
	Serial.print(pAngleChange);
	Serial.print(" duration=");
	Serial.print(pAngleTargetDuration);
	Serial.println(") ");
#endif

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

void HerkulexServoDrive::setNullAngle(float pRawAngle /* uncalibrated */) {
	if (configData)
		configData->nullAngle = pRawAngle;
}

void HerkulexServoDrive::moveToAngle(float pAngle, uint32_t pDuration_ms) {
#ifdef DEBUG_HERKULEX
	if (abs(lastAngle-pAngle)>0.1) {
		Serial.print("servo(");
		printActuator(configData->id),
		Serial.print(") a=");
		Serial.print(pAngle);
		Serial.print(",");
		Serial.print(pDuration_ms);
		lastAngle = pAngle;
	}
#endif
	float calibratedAngle  = constrain(pAngle, configData->minAngle,configData->maxAngle) ;
	Herkulex.moveOneAngle(setupData->herkulexMotorId, (calibratedAngle + configData->nullAngle), pDuration_ms, LED_BLUE);
	currentAngle = calibratedAngle;

	// get feedback	
	float feedbackAngle;
	readFeedback(feedbackAngle, torque,overloadDetected,anyHerkulexError);
	if (isOk()) {
		// currentAngle = feedbackAngle - configData->nullAngle;
	}
#ifdef DEBUG_HERKULEX
		Serial.print(F("a="));
		Serial.print(feedbackAngle,1);
		Serial.print(F("t="));
		Serial.print(torque,1);
		Serial.print(F("ol="));
		Serial.print(overloadDetected);
		Serial.print(F("ae"));
		Serial.print(anyHerkulexError);
		Serial.println(F("}"));
#endif
}

float HerkulexServoDrive::getCurrentAngle() {
	return currentAngle;
}

float HerkulexServoDrive::getRawAngle() {
	return currentAngle + configData->nullAngle;
}


void HerkulexServoDrive::loop(uint32_t now) {
	if (!movement.isNull()) {
		float toBeAngle = movement.getCurrentAngle(now);
		moveToAngle(toBeAngle, SERVO_SAMPLE_RATE); // stay at same position after this movement
	}
}

bool HerkulexServoDrive::isOk() {
	// return true if latest feedback indicated proper voltage and no overload
	return (!overloadDetected);
}



void HerkulexServoDrive::readFeedback(float &angle, float &torque /* [Nm) */, bool& overLoad, bool& anyerror ){
	uint16_t inputVoltage = 0;
	int16_t position = 0;
	int16_t pwm = 0;
	byte status = Herkulex.stat(setupData->herkulexMotorId);
	overLoad = (status & H_ERROR_OVERLOAD) != 0;
	anyerror = (status != H_STATUS_OK);
	angle = Herkulex.getAngle(setupData->herkulexMotorId);
	pwm = Herkulex.getSpeed(setupData->herkulexMotorId);
	torque = float (pwm);	
}


