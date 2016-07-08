/* 
* MotorDriverStepperImpl.cpp
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/

#include "Arduino.h"
#include "GearedStepperDrive.h"
#include "digitalWriteFast.h"
#include "setup.h"

void forwardstep(void* obj) {
	GearedStepperDrive* driver = (GearedStepperDrive*)obj;
	driver->direction(false,true);
	driver->performStep();
}
void backwardstep(void* obj) {
	GearedStepperDrive* driver = (GearedStepperDrive*)obj;
	driver->direction(false,false);
	driver->performStep();
}

void GearedStepperDrive::setup(	StepperConfig* pConfigData, StepperSetupData* pSetupData) {

	movement.setNull();

	configData = pConfigData;
	setupData = pSetupData;
	if (logSetup) {
		logger->println(F("setup stepper"));
		logger->print(F("   "));
		pConfigData->print();
		logger->print(F("   "));
		pSetupData->print();
	}
	pinMode(getPinClock(), OUTPUT);
	pinMode(getPinDirection(), OUTPUT);
	pinMode(getPinEnable(), OUTPUT);

	
	// set to default direction
	direction(true,currentDirection);
	
	// no movement currently
	movement.setNull();

	configData->degreePerMicroStep = getDegreePerFullStep()/getMicroSteps();
	
	configData->maxStepRatePerSecond  = (360.0/configData->degreePerMicroStep) *(float(getMaxRpm())/60.0);
	long maxAcceleration = (360.0/configData->degreePerMicroStep) *(float(getMaxAcc())/60.0); // [ steps/s^2 ]
	currentMotorAngle = 0.0;
	// setMeasuredAngle(0.0);
	accel.setup(this, forwardstep, backwardstep);
	accel.setMaxSpeed(configData->maxStepRatePerSecond);    // [steps/s]
	accel.setAcceleration(maxAcceleration);
}


void GearedStepperDrive::changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) {
	// this methods works even when no current Angle has been measured
	uint32_t now = millis();
	movement.set(getCurrentAngle(), getCurrentAngle()+pAngleChange, now, pAngleTargetDuration);
}


void GearedStepperDrive::setAngle(float pAngle,uint32_t pAngleTargetDuration) {
	if (currentAngleAvailable) {
		// limit angle
		pAngle = constrain(pAngle, configData->minAngle,configData->maxAngle);
		uint32_t now = millis();
		static float lastAngle = 0;
		if (abs(lastAngle-pAngle)> 0.1) {
			if (logStepper) {
				logger->print("stepper.setAngle[");
				printActuator(configData->id);
				logger->print("](");
				logger->print(pAngle);
				logger->print(" is=");
				logger->print(getCurrentAngle());
				logger->print(" now=");
				logger->print(now);
				logger->print(" duration=");
				logger->print(pAngleTargetDuration);
				logger->println(")");
			}
			lastAngle = pAngle;
			// set actuator angle (not motor angle)
			movement.set(getCurrentAngle(), pAngle, now, pAngleTargetDuration);
		}
	}
}


void GearedStepperDrive::performStep() {
	uint8_t clockPIN = getPinClock();
#ifdef USE_FAST_DIGITAL_WRITE
	digitalWriteFast(clockPIN, LOW);
	digitalWriteFast(clockPIN, HIGH);
#else
	digitalWrite(clockPIN, LOW);  // This LOW to HIGH change is what creates the
	digitalWrite(clockPIN, HIGH);
#endif

	if (currentDirection) {
		currentMotorAngle += configData->degreePerMicroStep;
	}
	else {
		currentMotorAngle -= configData->degreePerMicroStep;
	}
}

void GearedStepperDrive::setStepperDirection(bool forward) {
	bool dir = forward?LOW:HIGH;
	uint8_t pin = getPinDirection();
#ifdef USE_FAST_DIGITAL_WRITE
	digitalWriteFast(pin, dir);
#else
	digitalWrite(pin, dir);
#endif
}

void GearedStepperDrive::direction(bool dontCache,bool forward) {
	bool toBeDirection = forward;

	if ((toBeDirection != currentDirection) || dontCache)
	{
		if (!getDirection())
			forward=!forward;
		setStepperDirection(forward);
		currentDirection = toBeDirection;
	}
}

void GearedStepperDrive::enable() {
	enableDriver(true);
}

bool GearedStepperDrive::isEnabled() {
	return enabled;
}

void GearedStepperDrive::disable() {
	enableDriver(false);
}

void GearedStepperDrive::enableDriver(bool ok) {
	digitalWrite(getPinEnable(), ok?HIGH:LOW);  // This LOW to HIGH change is what creates the
	enabled = ok;
}

 
// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop(uint32_t now) {	
	 accel.run();	
}

float GearedStepperDrive::getToBeAngle() {
	return movement.getCurrentAngle(millis());
}


float GearedStepperDrive::getCurrentAngle() {
	return currentMotorAngle / getGearReduction();
}

void GearedStepperDrive::setCurrentAngle(float angle) {
	currentMotorAngle = angle*getGearReduction();
}

void GearedStepperDrive::setMeasuredAngle(float pMeasuredAngle) { 
	currentMotorAngle = pMeasuredAngle*getGearReduction();
	currentAngleAvailable = true;
	
	if (!movement.isNull()) {
		float toBeMotorAngle = movement.getCurrentAngle(millis())*getGearReduction();
		float diff = toBeMotorAngle  - currentMotorAngle;
		long steps = diff/configData->degreePerMicroStep;
		accel.move(steps );		
	}

}
