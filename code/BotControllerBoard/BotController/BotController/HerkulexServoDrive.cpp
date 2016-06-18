/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#include "Arduino.h"
#include "Actuator.h"
#include "BotMemory.h"


void HerkulexServoDrive::readFeedback(float &angle, float &voltage, float &torque /* [Nm) */, boolean& overLoad ){
	uint16_t inputVoltage = 0;
	int16_t position = 0;
	uint16_t pwm = 0;
	float rawAngle;
	if (servo)
		servo->getBehaviour(&inputVoltage, HKX_NO_VALUE, &position, HKX_NO_VALUE, &pwm, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE);
	voltage = (float)inputVoltage/1000;
	rawAngle = (float)position/10;
		
#define HERKULEX_MAX_TORQUE (12.0*1.0*9.81/100.0) // max torque is 12 kg/cm, that is 
	torque = float(pwm)/1024.0* HERKULEX_MAX_TORQUE;

	HkxStatus statusED;
	servo->getStatus(statusED, true);
	overLoad = statusED.isError(HKX_STAT_OVERLOAD);
	if (voltage > 5.0)
		angle = rawAngle;

}

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
	delay(50); // herkulex servos need that startup time. Better do something else after startup before initializing

	// initialize Herkulex servo
	HkxPrint* printout = new HkxPrint(Serial, CONNECTION_BAUD_RATE);  // No printout with Arduino UNO
	HkxCommunication* communication = new HkxCommunication(HKX_115200, Serial1, *printout);  // Communication with the servo on Serial1
	servo = new HkxPosControl(pSetupData->herkulexMotorId, *communication, *printout);  // control position for the servo ID=253 (factory default value)

	delay(50);

	servo->reboot();
	delay(300);
	
	// update current angle
	float feedbackAngle;
	readFeedback(feedbackAngle, voltage,torque,overloadDetected);
	if (isOk())
		mostRecentAngle = feedbackAngle - configData->nullAngle;
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
static float lastAngle = 0;
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
	servo->movePosition((calibratedAngle + configData->nullAngle)*10.0, pDuration_ms, HKX_LED_BLUE, false); 
	mostRecentAngle = calibratedAngle;

	// get feedback	
	float feedbackAngle;
	readFeedback(feedbackAngle, voltage,torque,overloadDetected);
	if (isOk())
		mostRecentAngle = feedbackAngle - configData->nullAngle;
		
#ifdef DEBUG_HERKULEX
		Serial.print(F("a="));
		Serial.print(mostRecentAngle,1);
		Serial.print(F("v="));
		Serial.print(voltage,1);
		Serial.print(F("t="));
		Serial.print(torque,1);
		Serial.print(F("ol="));
		Serial.print(overloadDetected);
		Serial.println(F("}"));
#endif
}

float HerkulexServoDrive::getCurrentAngle() {
	return mostRecentAngle-configData->nullAngle;
}

float HerkulexServoDrive::getRawAngle() {
	return mostRecentAngle;
}


void HerkulexServoDrive::loop(uint32_t now) {
	if (!movement.isNull()) {
		float toBeAngle = movement.getCurrentAngle(now);
		moveToAngle(toBeAngle, SERVO_SAMPLE_RATE); // stay at same position after this movement
	}
}

bool HerkulexServoDrive::isOk() {
	// return true if latest feedback indicated proper voltage and no overload
	return ((voltage > 5.0) && !overloadDetected);
}





