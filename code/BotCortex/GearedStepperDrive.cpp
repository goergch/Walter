/* 
* GearedStepperDrive.cpp
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/

#include "Arduino.h"
#include "GearedStepperDrive.h"
#include "BotMemory.h"
#include "utilities.h"

void forwardstep(void* obj) {
	GearedStepperDrive* driver = (GearedStepperDrive*)obj;
	driver->direction(true);
	driver->performStep();
}
void backwardstep(void* obj) {
	GearedStepperDrive* driver = (GearedStepperDrive*)obj;
	driver->direction(false);
	driver->performStep();
}

void GearedStepperDrive::setup(	StepperConfig* pConfigData, ActuatorConfiguration* pActuatorConfig, StepperSetupData* pSetupData, RotaryEncoder* pEncoder) {

	movement.setNull();

	actuatorConfig = pActuatorConfig;
	configData = pConfigData;
	setupData = pSetupData;
	encoder =  pEncoder;

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

	direction(!currentDirection); // force setting the PIN by setting the other direction than the current one

	// no movement currently
	movement.setNull();

	accel.setup(this, forwardstep, backwardstep);
	accel.setMaxSpeed(getMaxStepsPerSecond());    			// [steps/s]
	accel.setAcceleration(getMaxStepAccPerSecond());

	if (memory.persMem.logSetup) {
		logger->print(F("   "));
		pConfigData->print();
		logger->print(F("   "));
	}

	logger->print(F("microsteps="));
	logger->print(getMicroSteps());

	logger->print(F(" getMaxAcc="));
	logger->print(getMaxAcc());

	logger->print(F(" getMaxRPM="));
	logger->print(getMaxRpm());

	logger->print(F(" maxAcc="));
	logger->print(getMaxStepAccPerSecond());
	logger->print(F(" maxSpeed="));
	logger->print(getMaxStepsPerSecond());
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
		currentAngle = pAngle;
		uint32_t now = millis();

		/*
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
			*/

		// set actuator angle (which is not the motor angle)
		movement.set(movement.getCurrentAngle(now), pAngle, now, pAngleTargetDuration);
	}
}

void GearedStepperDrive::performStep() {
	uint8_t clockPIN = getPinClock();
	// This LOW to HIGH change is what creates the step
	digitalWrite(clockPIN, HIGH);
	delayMicroseconds(PIBOT_PULSE_WIDTH_US);
	digitalWrite(clockPIN, LOW);

	
	// adapt last motor position according the step, if motor is enabled
	if (enabled) { 
		bool direction = currentDirection;	// currently selected direction
		if (direction) {
			currentAngle += getAnglePerMicroStep();
		}
		else {
			currentAngle -= getAnglePerMicroStep();
		}
	}
}

void GearedStepperDrive::setStepperDirection(bool forward) {
	digitalWrite(getPinDirection(), forward?LOW:HIGH);
}


inline void GearedStepperDrive::direction(bool forward) {
	bool toBeDirection = forward;

	if ((toBeDirection != currentDirection))
	{
		if (!setupData->direction)
			forward=!forward;

		setStepperDirection(forward);
		currentDirection = toBeDirection;
	}
}

void GearedStepperDrive::enable() {
	enableDriver(true);
	integral = 0.0;
}

bool GearedStepperDrive::isEnabled() {
	return enabled;
}

void GearedStepperDrive::disable() {
	enableDriver(false);
}

void GearedStepperDrive::enableDriver(bool ok) {
	enabled = ok; // do this first to switch of the stepper loop
	// set the clock to low to avoid switch-on-tick due to low/high impulse
	digitalWrite(getPinClock(), LOW);
	delayMicroseconds(100);
	digitalWrite(getPinEnable(), ok?HIGH:LOW);
}

// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop(uint32_t now) {	
	loop();
}


// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop() {
	if (accel.runSpeed()) {
		accel.computeNewSpeed();

		/*
		float resonanceDistance = (accel.speed() - getResonanceSpeed()); // distance of current speed and resonance speed
		resonanceDistance *= resonanceDistance;
		if (!resonanceMode && (resonanceDistance < 10)) {
			// switch resonance mode on:
			// double the allowed acceleration in order to go faster through the critical speed and
			// double the sample rate in order to have a bigger amplitude in the speed wave avoiding the resonance speed
			// increase speed by 1/7
			resonanceMode = true;
			accel.setAcceleration(getMaxStepAccPerSecond()*2);
			configData->sampleRate = 2*configData->sampleRate;
			accel.setSpeed(1.14*accel.speed());
		}
		if (resonanceMode && (resonanceDistance >= 10)) {
			// switch off resonance mode
			resonanceMode = false;
			accel.setAcceleration(	getMaxStepAccPerSecond());
			configData->sampleRate = configData->sampleRate/2;
		}
		*/
	}
}

float GearedStepperDrive::getCurrentAngle() {
	return currentAngle;
}

void GearedStepperDrive::setCurrentAngle(float angle) {
	currentAngle = angle;
}

void GearedStepperDrive::setMeasuredAngle(float pMeasuredActuatorAngle, uint32_t now) { 

	currentAngle = pMeasuredActuatorAngle;
	if (!currentAngleAvailable) {
		lastToBeAngle = pMeasuredActuatorAngle;
		currentAngleAvailable = true;
	}

	if (!movement.isNull()) {
		// compute steps resulting from trajectorys speed and the
		// error when comparing the to-be position with the measured position
		float dT = sampleTime();
		float frequency = sampleFrequency();

		float toBeAngle = 			movement.getCurrentAngle(now);
		float nextToBeAngle = 		movement.getCurrentAngle(now+dT);

		float anglePerSample = 		toBeAngle-lastToBeAngle;
		float nextAnglePerSample = 	nextToBeAngle - toBeAngle;

		float currStepsPerSample = getMicroStepsByAngle(anglePerSample);
		float nextStepsPerSample = getMicroStepsByAngle(nextAnglePerSample);
		float stepErrorPerSample = getMicroStepsByAngle(toBeAngle  - currentAngle);		// current error, i.e. to-be-angle compared with encoder's angle

		// the step error is going through a PI-controller and added to the to-be speed (=stepsPerSample)
		float maxAcc = getMaxStepAccPerSecond();

		float Pout = configData->kP * stepErrorPerSample;
		integral += stepErrorPerSample * dT;
		float Iout = configData->kI * integral;
		float PIDoutput = Pout + Iout;
		float accelerationPerSample = PIDoutput;

		float distanceToNextSample = accelerationPerSample + currStepsPerSample;
		// distanceToNextSample = constrain(distanceToNextSample, -maxAcc*dT,maxAcc*dT);

		// move to target with to-be acceleration, defined max speed
		float sampleAcc = ((currStepsPerSample-nextStepsPerSample) + stepErrorPerSample)*frequency;

		// deceleration is as double as high as acceleration
		if (((distanceToNextSample > 0) && (sampleAcc > 0)) ||
			((distanceToNextSample < 0) && (sampleAcc < 0))) {
			sampleAcc = constrain(sampleAcc*2.0, -maxAcc, maxAcc);
		} else {
			sampleAcc = constrain(sampleAcc, -maxAcc, maxAcc);

		}

		accel.setAcceleration(fabs(sampleAcc)+1000);
		accel.move(distanceToNextSample);

		if ((configData->id == 4) && memory.persMem.logStepper) {
			logger->print("t=");
			logger->print(millis());
			logger->print(" a=");
			logger->print(toBeAngle);
			logger->print(" last=");
			logger->print(lastToBeAngle);
			logger->print(" curr=");
			logger->print(currentAngle);
			logger->print(" rpm=");
			logger->print(getRPMByAnglePerSample(anglePerSample));
			logger->print(" ms=");
			logger->println(getMicroSteps());
			logger->print(" serror=");
			logger->print(stepErrorPerSample);
			logger->print(" o=");
			logger->print(PIDoutput);
			logger->print(" acc=");
			logger->print(sampleAcc);
			logger->print(" av=");
			logger->println(accel.speed());
			logger->print(" var=");
			logger->println(encoder->getVariance());
		}
		lastToBeAngle = toBeAngle;
	}
}




