/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#include "Arduino.h"
#include "Actuator.h"
#include "BotMemory.h"
#include <avr/wdt.h>


void HerkulexServoDrive::setup(ServoConfig* pConfigData, ServoSetupData* pSetupData) {
	if (!communicationEstablished) {
		fatalError(F("HerkuleX communication not ready"));
	}

	configData = pConfigData;
	setupData = pSetupData;

	if (logServo) {
		logger->println(F("setup servo"));
		logger->print(F("   "));
		setupData->print();
		logger->print(F("   "));
		configData->print();
	}
	movement.setNull();

	maxTorque = setupData->maxTorque;
	
	// Herkulex.reboot(pSetupData->herkulexMotorId); //reboot first motor
	// delay(500);
	startTime = millis();
	if (logServo) {
		logger->print(F("torque off"));
	}
	// switch off torque, wait for real action until enable is called
	Herkulex.torqueOFF(setupData->herkulexMotorId);

	// find out if servo is connected
	if (logServo) {
		logger->print(F("get stat"));
	}
	byte stat = Herkulex.stat(setupData->herkulexMotorId);
	if (stat == H_STATUS_OK) {
		connected = true;
	} else {
		logger->print(F("stat="));
		logger->println(stat,HEX);
		fatalError(F("Herkulex not connected"));
	}
} //setup

void HerkulexServoDrive::disable() {
	Herkulex.torqueOFF(setupData->herkulexMotorId);
	enabled = false;
}

bool HerkulexServoDrive::isEnabled() {
	return enabled;
}

void HerkulexServoDrive::enable() {
	if (!isConnected())
		return;
	uint32_t now = millis();
	int32_t delayTime = startTime+100-now;
	delay(max(delayTime,0)); // wait at least 100ms after initialization
	now = millis();

	// fetch current angle of servo
	float feedbackAngle = Herkulex.getAngle(setupData->herkulexMotorId);
	currentAngle = feedbackAngle - configData->nullAngle;

	// startup procedure. If angle is not within range, 
	// slowly drive to the nearest boundary. Otherwise, stay at the current position
	float startAngle  = constrain(currentAngle, configData->minAngle,configData->maxAngle);
	float toBeAngle = currentAngle;
	uint32_t duration = abs(currentAngle-startAngle)*(1000.0/setupData->setupSpeed);
	duration = constrain(duration,0,2000);
	movement.set(currentAngle, startAngle, now, duration);

	Herkulex.torqueON(setupData->herkulexMotorId);
	while (millis() < now + duration)  {		
		wdt_reset(); // this loop can take a second
		toBeAngle = movement.getCurrentAngle(millis());
		moveToAngle(toBeAngle, SERVO_SAMPLE_RATE, false); // stay at same position after this movement
	}

	// now servo is in a valid angle range. Set this angle as starting point
	setAngle(startAngle,SERVO_SAMPLE_RATE);
	
	enabled = true;
}

bool HerkulexServoDrive::communicationEstablished = false;
void HerkulexServoDrive::setupCommunication() {
	
	Herkulex.beginSerial1(115200);	// default baud rate of Herkulex.

	Herkulex.initialize();			//initialize all motors

	communicationEstablished  = true;
}

void HerkulexServoDrive::changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) {
	if (logServo) {
		logger->print(F("Herkulex.changeAngle("));
		logger->print(pAngleChange);
		logger->print(F(" duration="));
		logger->print(pAngleTargetDuration);
		logger->println(") ");
	}
	
	// this methods works even when no current Angle has been measured
	movement.set(getCurrentAngle(), getCurrentAngle()+pAngleChange, millis(), pAngleTargetDuration);
}


void HerkulexServoDrive::setAngle(float pAngle,uint32_t pAngleTargetDuration) {
	uint32_t now = millis();
	
	// limit angle
	pAngle = constrain(pAngle, configData->minAngle,configData->maxAngle);

	static float lastAngle = 0;
	if (abs(lastAngle-pAngle)> 1) {
		if (logServo) {		
			logger->print(F("Herkulex.setAngle("));
			logger->print(pAngle);
			logger->print(F(" duration="));
			logger->print(pAngleTargetDuration);
			logger->println(") ");
		}
		lastAngle = pAngle;
	}
	movement.set(getCurrentAngle(), pAngle, now, pAngleTargetDuration);
}

void HerkulexServoDrive::setNullAngle(float pRawAngle /* uncalibrated */) {
	if (configData)
		configData->nullAngle = pRawAngle;
}

void HerkulexServoDrive::moveToAngle(float pAngle, uint32_t pDuration_ms, bool limitRange) {
	if (logServo) {
		if (abs(lastAngle-pAngle)>0.1) {
			logger->print(F("servo("));
			printActuator(configData->id),
			logger->print(F(") ang="));
			logger->print(pAngle);
			logger->print(",");
			logger->print(pDuration_ms);
		}
	}
	float 	calibratedAngle = pAngle;
	if (limitRange) 
		calibratedAngle = constrain(calibratedAngle, configData->minAngle,configData->maxAngle) ;
		
	// add one sample slot to the time, otherwise the servo does not run smooth but in steps	
	Herkulex.moveOneAngle(setupData->herkulexMotorId, (calibratedAngle + configData->nullAngle)-torqueExceededAngleCorr, pDuration_ms+SERVO_TARGET_TIME_ADDON, LED_BLUE);
	currentAngle = calibratedAngle;

	bool maxTorqueReached; 
	if (getConfig().id == GRIPPER)	{
		// read torque
		torque = readServoTorque();
	
		// if torque is too high, release it
		maxTorqueReached = (abs(torque) > maxTorque);
	
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
	}


	if (logServo) {
		if (abs(lastAngle-pAngle)>0.1) {
			if (getConfig().id == GRIPPER)	{
				logger->print(F("tor="));
				logger->print(torque);

				logger->print(F("mtr="));
				logger->print(maxTorqueReached);
				logger->print(F("teac="));
				logger->print(torqueExceededAngleCorr);
			}
			logger->print(F("t="));
			logger->print(torque,1);
			logger->println(F("}"));
			lastAngle = pAngle;
		}
	}
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
		moveToAngle(toBeAngle, (SERVO_SAMPLE_RATE), true); // stay at same position after this movement
	}
}

bool HerkulexServoDrive::isOk() {
	// return true if latest feedback indicated proper voltage and no overload
	return (!overloadDetected);
}



void HerkulexServoDrive::readFeedback(float &angle, float &torque /* [Nm) */, bool& overLoad, bool& anyerror ){
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