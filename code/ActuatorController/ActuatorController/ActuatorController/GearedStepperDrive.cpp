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
		logger->print(F("   setup stepper"));
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
	accel.setMaxSpeed(maxStepRatePerSecond);    // [steps/s], with 24Mhz up to 6000 steps/s is possible
	accel.setAcceleration(maxAcceleration);		
	accel.setMinPulseWidth(10);					// default is 20ms, but PiBot Steppers work smoother with 10ms

	if (memory.persMem.logSetup) {
		logger->print(F("   "));
		pConfigData->print();
		logger->print(F("   "));
	}

	logger->print(F("degreePerMicroStep="));
	logger->print(configData->degreePerMicroStep);
	
	logger->print(F(" getMaxAcc="));
	logger->print(getMaxAcc());

	logger->print(F(" stepAcceleration="));
	logger->print(maxAcceleration);

	logger->print(F(" getMaxRPM="));
	logger->print(getMaxRpm());

	logger->print(F(" maxAcc="));
	logger->print(maxAcceleration);
	logger->print(F(" maxSpeed="));
	logger->print(maxStepRatePerSecond);
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

		if (abs(lastAngle-pAngle)> 0.1) {
			lastAngle = pAngle;
			if ( (configData->id == 0) && memory.persMem.logStepper) {
					logger->print(F("stepper.setAngle["));
					logActuator(configData->id);
					logger->print(F("]("));
					logger->print(pAngle);
					logger->print(F(" is="));
					logger->print(movement.getCurrentAngle(now));
					logger->print(F(" now="));
					logger->print(now);
					logger->print(F(" duration="));
					logger->print(pAngleTargetDuration);
					logger->println(")");
			}
		}

		// set actuator angle (which is not the motor angle)
		movement.set(movement.getCurrentAngle(now), pAngle, now, pAngleTargetDuration);
	}
}

void GearedStepperDrive::performStep() {
	uint8_t clockPIN = getPinClock();
	// This LOW to HIGH change is what creates the step
	digitalWriteFast(clockPIN, LOW);
	digitalWriteFast(clockPIN, HIGH);
	
	// adapt last motor position according the step, if motor is enabled
	if (enabled) { 
		bool direction = currentDirection;	// currently selected direction
		if (getDirection())					// mechanical direction of motor
			direction = !direction;
		if (direction) {
			currentMotorAngle += configData->degreePerMicroStep;
		}
		else {
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

// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop() {
	accel.run();
}


float GearedStepperDrive::getCurrentAngle() {
	return (currentMotorAngle / getGearReduction());
}

void GearedStepperDrive::setCurrentAngle(float angle) {
	currentMotorAngle = (angle)*getGearReduction();
}

void GearedStepperDrive::setMeasuredAngle(float pMeasuredAngle, uint32_t now) { 
	currentMotorAngle = pMeasuredAngle*getGearReduction();
	currentAngleAvailable = true;
	
	if (!movement.isNull()) {

		float toBeMotorAngle = movement.getCurrentAngle(now)*getGearReduction();
		float angleError= toBeMotorAngle  - currentMotorAngle;
		
		// proportional part of PD controller
		float Pout = configData->kP * angleError;
		
		// derivative part of PD controller. Needs to be calibrated together with the
		// stepper library that limits the acceleration
		const float rezi_dT = 1000.0/float(ENCODER_SAMPLE_RATE);
		float Dout = configData->kD * (angleError - pid_pre_error) * rezi_dT;		
		pid_pre_error = angleError;

		float output = Pout + Dout ;

		// compute steps out of degrees	
		accel.move(output/configData->degreePerMicroStep);		
		
		if ((configData->id == 0) && memory.persMem.logStepper) {
			logger->print(F("stepper.setMeasurement["));
			logActuator(configData->id);
			logger->print(F("](tobe="));
			logger->print(toBeMotorAngle);
			logger->print(F(" is="));
			logger->print(currentMotorAngle);
			logger->print(F(" angleError="));
			logger->print(angleError);
			logger->print(F(" steps="));
			logger->print(output/configData->degreePerMicroStep);
			logger->println(")");
		}
	} 
}
