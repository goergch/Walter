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
	if (!communicationEstablished) {
		fatalError(F("HerkuleX communication not ready"));
	}

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

	maxTorque = setupData->maxTorque;
	
	// Herkulex.reboot(pSetupData->herkulexMotorId); //reboot first motor
	// delay(500);
	startTime = millis();

	// Herkulex.set_ID(pSetupData->herkulexMotorId,pSetupData->herkulexMotorId-1);
} //setup

void HerkulexServoDrive::enable() {
	int32_t delayTime = startTime+100-millis();
	delay(max(delayTime,0)); // wait at least 100ms after initialization

	// update current angle
	float feedbackAngle = Herkulex.getAngle(setupData->herkulexMotorId);
	currentAngle = feedbackAngle - configData->nullAngle;
	setAngle(currentAngle, SERVO_SAMPLE_RATE); // set this angle, so the servo does not jump
	
	int saturator = Herkulex.getSaturatorSlope(setupData->herkulexMotorId);
	Serial.print("saturatorSlope=");
	Serial.print(saturator);
}

bool HerkulexServoDrive::communicationEstablished = false;
void HerkulexServoDrive::setupCommunication() {
	
	Herkulex.beginSerial1(115200);	// default baud rate of Herkulex.
	Herkulex.initialize();			//initialize all motors

	communicationEstablished  = true;
}

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
	}
#endif
	float calibratedAngle  = constrain(pAngle, configData->minAngle,configData->maxAngle) ;
	Herkulex.moveOneAngle(setupData->herkulexMotorId, (calibratedAngle + configData->nullAngle)-torqueExceededAngleCorr, pDuration_ms, LED_BLUE);
	currentAngle = calibratedAngle;

	// read torque
	torque = readServoTorque();
	
	// if torque is too high, release it
	bool maxTorqueReached = (abs(torque) > maxTorque);
	
	if (maxTorqueReached) {
		// increase amount of torque correction
		if (torqueExceededAngleCorr  == 0) {
			torqueExceededAngleCorr = sgn(torque);
		} else {
			torqueExceededAngleCorr = sgn(torqueExceededAngleCorr) * (abs(torqueExceededAngleCorr)+1);			
		}
		torqueExceededAngleCorr = constrain(torqueExceededAngleCorr,-30,30);		
	} else {
		if (torqueExceededAngleCorr != 0) {
			// reduce absolute value of angle correction
			torqueExceededAngleCorr = sgn(torqueExceededAngleCorr)*(abs(torqueExceededAngleCorr) - 1);
		} else {
			// no torque, no correction, do nothing
		}
	}
	
	if (getConfig().id == GRIPPER)	{
		Serial.print("tor=");
		Serial.print(torque);

		Serial.print("mtr=");
		Serial.print(maxTorqueReached);
		Serial.print("teac=");
		Serial.print(torqueExceededAngleCorr);
		Serial.println();
		
	}

#ifdef DEBUG_HERKULEX
	if (abs(lastAngle-pAngle)>0.1) {
		Serial.print(F("t="));
		Serial.print(torque,1);
		Serial.println(F("}"));
		lastAngle = pAngle;
	}
#endif
}

float HerkulexServoDrive::getTorque() {
	return torque;
}


// set maximum torque. when 0 is passed, switch off torque controll
void HerkulexServoDrive::setMaxTorque(float pTorque) {
	int torque = pTorque;
	if ((torque > setupData->maxTorque) || (torque == 0))
		torque = setupData->maxTorque;
	maxTorque = torque;
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
	int16_t position = 0;
	int16_t pwm = 0;
	byte status = Herkulex.stat(setupData->herkulexMotorId);
	overLoad = (status & H_ERROR_OVERLOAD) != 0;
	anyerror = (status != H_STATUS_OK);
	angle = Herkulex.getAngle(setupData->herkulexMotorId);
	pwm = Herkulex.getPWM(setupData->herkulexMotorId); // pwm is proportional to torque
	torque = float (pwm);	
}

float HerkulexServoDrive::readServoTorque(){
	int16_t pwm = Herkulex.getPWM(setupData->herkulexMotorId); // pwm is proportional to torque
	float torque = float (pwm);
	return torque;
}