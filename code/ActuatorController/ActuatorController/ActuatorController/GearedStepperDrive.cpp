/* 
* GearedStepperDrive.cpp
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/

#include "Arduino.h"
#include "GearedStepperDrive.h"
#include "digitalWriteFast.h"
#include "BotMemory.h"
#include "utilities.h"

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

void GearedStepperDrive::setup(	StepperConfig* pConfigData, ActuatorConfiguration* pActuatorConfig, StepperSetupData* pSetupData) {

	movement.setNull();

	actuatorConfig = pActuatorConfig;
	configData = pConfigData;
	setupData = pSetupData;
	if (memory.persMem.logSetup) {
		logger->print(F("setup stepper"));
		logger->print(F(" pin(EN,DIR,CLK)=("));
		logPin(getPinEnable());
		logger->print(",");
		logPin(getPinDirection());
		logger->print(",");
		logPin(getPinClock());
		logger->print(F(") 1:"));
		logger->println(getGearReduction(),1);

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
	
	long maxStepRatePerSecond  = (360.0/configData->degreePerMicroStep) *(float(getMaxRpm())/60.0);
	long maxAcceleration = (360.0/configData->degreePerMicroStep) *(float(getMaxAcc())/60.0); // [ steps/s^2 ]
	currentMotorAngle = 0.0;
	accel.setup(this, forwardstep, backwardstep);
	accel.setMaxSpeed(maxStepRatePerSecond);    // [steps/s]
	accel.setAcceleration(maxAcceleration);

	if (memory.persMem.logSetup) {
		logger->print(F("   "));
		pConfigData->print();
		logger->print(F("   "));
	}

	logger->print(F("degreePerMicroStep="));
	logger->print(configData->degreePerMicroStep);
	
	logger->print(F(" getMaxAcc="));
	logger->print(getMaxAcc());

	logger->print(F(" maxAcc="));
	logger->print(maxAcceleration);
	logger->print(F(" accel="));
	logger->print((long)&accel);
	logger->print(F(" stepper="));
	logger->print((long)this);
	logger->println();
}

void GearedStepperDrive::changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) {
	// this methods works even when no current Angle has been measured
	uint32_t now = millis();
	movement.set(getCurrentAngle(), getCurrentAngle()+pAngleChange, now, pAngleTargetDuration);
}

void GearedStepperDrive::setAngle(float pAngle,uint32_t pAngleTargetDuration) {
	// setting an absolute angle is allowed only if the encoder has called setMeasuredAngle already
	// if we do not yet know the current angle, we cannot set the angle and return
	if (currentAngleAvailable) {
		// limit angle
		pAngle = constrain(pAngle, configData->minAngle,configData->maxAngle);
		uint32_t now = millis();
		static float lastAngle = 0;
		if (abs(lastAngle-pAngle)> 0.1) {
			if (memory.persMem.logStepper) {
				logger->print(F("stepper.setAngle["));
				logActuator(configData->id);
				logger->print(F("]("));
				logger->print(pAngle);
				logger->print(F(" is="));
				logger->print(getCurrentAngle());
				logger->print(F(" now="));
				logger->print(now);
				logger->print(F(" duration="));
				logger->print(pAngleTargetDuration);
				logger->println(")");
			}
			lastAngle = pAngle;
			// set actuator angle (which is not the motor angle)
			movement.set(getCurrentAngle(), pAngle, now, pAngleTargetDuration);
		}
	}
}

void GearedStepperDrive::performStep() {
	uint8_t clockPIN = getPinClock();
	// This LOW to HIGH change is what creates the step
	digitalWriteFast(clockPIN, LOW);
	digitalWriteFast(clockPIN, HIGH);
	
	// adapt last motor position according the step, if motor is enabled
	if (enabled) { 
		bool direction = currentDirection; // currently selected direction
		if (getDirection())					// mechanical direction of motor
			direction = !direction;
		if (direction) {
			// logger->print("-");
			currentMotorAngle += configData->degreePerMicroStep;
		}
		else {
			// logger->print("+");
			currentMotorAngle -= configData->degreePerMicroStep;
		}
	}
}

void GearedStepperDrive::setStepperDirection(bool forward) {
	bool dir = forward?LOW:HIGH;
	uint8_t pin = getPinDirection();

	digitalWriteFast(pin, dir);
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
	return (currentMotorAngle / getGearReduction());
}

void GearedStepperDrive::setCurrentAngle(float angle) {
	currentMotorAngle = (angle)*getGearReduction();
}

void GearedStepperDrive::setMeasuredAngle(float pMeasuredAngle) { 
	currentMotorAngle = pMeasuredAngle*getGearReduction();
	currentAngleAvailable = true;
	
	if (!movement.isNull()) {
		float toBeMotorAngle = movement.getCurrentAngle(millis())*getGearReduction();

		float diff = toBeMotorAngle  - currentMotorAngle;
		long steps = diff/configData->degreePerMicroStep;
		steps /= 2;
		// logger->print("move");
		// logger->print(diff);

		/*
		logger->print(" diff=");
		logger->print(diff,1);
		logger->print(" dpms=");
		logger->print(configData->degreePerMicroStep,1);

		logger->print(" steps=");
		logger->println(steps);
		*/
		
		accel.move(steps );		
	} 
}
