/* 
* MotorDriver.cpp
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/

#include "Arduino.h"
#include "Actuator.h"
#include "BotMemory.h"
#include "utilities.h"
#include "watchdog.h"
#include "core.h"
#include "pins.h"

bool HerkulexServoDrive::communicationEstablished = false; // communication is shared across all servos

bool HerkulexServoDrive::setup(ServoConfig* pConfigData, ServoSetupData* pSetupData) {
	if (!communicationEstablished) {
		logFatal(F("HerkuleX communication not ready"));
		setError(HERKULEX_COMMUNICATION_FAILED);
	}

	configData = pConfigData;
	setupData = pSetupData;

	if (memory.persMem.logServo) {
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
	if (memory.persMem.logServo) {
		logger->print(F("torque off"));
	}
	// switch off torque, wait for real action until enable is called
	Herkulex.torqueOFF(setupData->herkulexMotorId);

	// find out if servo is connected
	if (memory.persMem.logServo) {
		logger->print(F("get stat"));
	}
	byte stat = Herkulex.stat(setupData->herkulexMotorId);
	if (stat == H_STATUS_OK) {
		connected = true;
		if (memory.persMem.logServo) {
			logger->println(F("Herkulex connection ok"));
		}
		return true;
	} else {
		logger->print(F("stat="));
		logger->println(stat,HEX);
		logFatal(F("Herkulex not connected"));
		setError(HERKULEX_STATUS_FAILED);
		return false;
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
	delay(std::max(delayTime,(int32_t)0)); // wait at least 100ms after initialization
	now = millis();

	// fetch current angle of servo
	float feedbackAngle = Herkulex.getAngle(setupData->herkulexMotorId);
	currentAngle = feedbackAngle - configData->nullAngle;
	if (memory.persMem.logServo) {
		logger->print(F("current angle"));
		logger->println(currentAngle);
	}

	// startup procedure. If angle is not within range, 
	// slowly drive to the nearest boundary. Otherwise, stay at the current position
	float startAngle  = constrain(currentAngle, configData->minAngle,configData->maxAngle);
	float toBeAngle = currentAngle;
	uint32_t duration = abs(currentAngle-startAngle)*(1000.0/setupData->setupSpeed);
	duration = constrain(duration,(uint32_t)0,(uint32_t)2000);
	movement.set(currentAngle, startAngle, now, duration);

	Herkulex.torqueON(setupData->herkulexMotorId);
	while (millis() < now + duration)  {		
		watchdogReset();
		toBeAngle = movement.getCurrentAngle(millis()+SERVO_SAMPLE_RATE);
		float asIsAngle = movement.getCurrentAngle(millis());

		moveToAngle(toBeAngle, SERVO_SAMPLE_RATE, false, abs(toBeAngle-asIsAngle)/SERVO_SAMPLE_RATE); // stay at same position after this movement
	}

	// now servo is in a valid angle range. Set this angle as starting point
	setAngle(startAngle,SERVO_SAMPLE_RATE);

	enabled = true;
}

void HerkulexServoDrive::setupCommunication() {
	// Herkulex servos are connected via Serial1
	// establish logging output
	Herkulex.beginSerial(servoComm,HERKULEX_BAUD_RATE);	// default baud rate of Herkulex.

	Herkulex.initialize();			// initialize all motors

	communicationEstablished  = true;
}

void HerkulexServoDrive::changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) {
	if (memory.persMem.logServo) {
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

	if (abs(lastAngle-pAngle)> 0.1) {
		if (memory.persMem.logServo) {		
			logger->print(F("Herkulex.setAngle("));
			logger->print(pAngle);
			logger->print(F(" duration="));
			logger->print(pAngleTargetDuration);
			logger->println(") ");
		}
		lastAngle = pAngle;
	}

	movement.set(movement.getCurrentAngle(now), pAngle, now, pAngleTargetDuration);
}

void HerkulexServoDrive::setNullAngle(float pRawAngle /* uncalibrated */) {
	if (configData)
		configData->nullAngle = pRawAngle;
}

void HerkulexServoDrive::moveToAngle(float pAngle, uint32_t pDuration_ms, bool limitRange, float speed /* degrees per ms */) {
	if (memory.persMem.logServo) {
		float actualAngle = readCurrentAngle();

		// if (abs(lastAngle-pAngle)>0.1)
		{
			logger->print(F("servo("));
			logActuator(configData->id),
			logger->print(F(") ang="));
			logger->print(pAngle);
			logger->print("°,");
			logger->print(pDuration_ms);
			logger->print(",");
			logger->print(actualAngle);
			logger->print("° ");
		}
	}
	float calibratedAngle = pAngle;
	if (limitRange) 
		calibratedAngle = constrain(calibratedAngle, configData->minAngle,configData->maxAngle) ;
		
	// add one sample slot to the time, otherwise the servo does not run smooth but in steps	
	Herkulex.moveOneAngle(setupData->herkulexMotorId, (calibratedAngle + configData->nullAngle)-torqueExceededAngleCorr, pDuration_ms, LED_BLUE);
	currentAngle = calibratedAngle;

	bool maxTorqueReached = false;
	if ((getConfig().id == GRIPPER)) 	{
		// read torque
		torque = readServoTorque();
		// logger->print("t=");
		// logger->print(torque);

		// if torque is too high, release it
		maxTorqueReached = (abs(torque) > maxTorque);
		if (maxTorqueReached) {
			// increase amount of torque correction by 1 with each call 
			if (torqueExceededAngleCorr  == 0) {
				torqueExceededAngleCorr = sgn(torque);
			} else {
				// compute an arbitrary correction factor, that corrects by 3° at low speeds, i.e. when the grip is too tight.
				// high speed reduces this effect in order to let the gripper move fast without effect.
				float corrected = 3.0/(1.0+abs(speed)*100);
				// logger->print(" c=");
				// logger->print(corrected);
				// logger->print(" s=");
				// logger->print(speed);
				torqueExceededAngleCorr = sgn(torqueExceededAngleCorr) * (abs(torqueExceededAngleCorr)+corrected);
			}
			torqueExceededAngleCorr = constrain(torqueExceededAngleCorr,-30,30); // limit torque correction to 30°

		} else {
			if (torqueExceededAngleCorr != 0) {
				// reduce absolute value of angle correction until 0
				torqueExceededAngleCorr = sgn(torqueExceededAngleCorr)*(abs(torqueExceededAngleCorr) - std::min(abs(torqueExceededAngleCorr),1.0));
			} else {
				// no torque, no correction, do nothing
			}
		}
	}

	if (memory.persMem.logServo) {
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
		}
	}

	lastAngle = pAngle;
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

float HerkulexServoDrive::readCurrentAngle() {
	byte status = Herkulex.stat(setupData->herkulexMotorId);
	bool anyerror = (status != H_STATUS_OK);
	if (anyerror) {
		logger->print("status=");
		logger->print(status);
		logger->print(" ");
		Herkulex.clearError(setupData->herkulexMotorId);
	}
	float feedbackAngle = Herkulex.getAngle(setupData->herkulexMotorId);
	return feedbackAngle - configData->nullAngle;
}

float HerkulexServoDrive::getRawAngle() {
	return currentAngle + configData->nullAngle;
}

void HerkulexServoDrive::loop(uint32_t now) {
	if (!movement.isNull()) {
		float toBeAngle = movement.getCurrentAngle(now+SERVO_SAMPLE_RATE);
		float asIsAngle = movement.getCurrentAngle(now);

		float speed = (toBeAngle-asIsAngle)/SERVO_SAMPLE_RATE;

		currentAngle = toBeAngle;
		moveToAngle(toBeAngle, (SERVO_SAMPLE_RATE + SERVO_MOVE_DURATION), true, speed); // stay at same position after this movement
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
