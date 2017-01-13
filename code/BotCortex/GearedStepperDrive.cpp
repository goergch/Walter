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
	driver->direction(false,true);
	driver->performStep();
}
void backwardstep(void* obj) {
	GearedStepperDrive* driver = (GearedStepperDrive*)obj;
	driver->direction(false,false);
	driver->performStep();
}

void GearedStepperDrive::setup(	StepperConfig* pConfigData, ActuatorConfiguration* pActuatorConfig, StepperSetupData* pSetupData, RotaryEncoder* encoder) {

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

	anglePerMicroStep = getDegreePerMicroStep()/getGearReduction();
	dT = float(sampleRate)/1000.0;
	rezi_dT = 1.0/dT;
	maxStepsPerSecond = configData->maxSpeed*(360/60)/getDegreePerMicroStep();
	maxStepAccPerSecond = configData->maxAcc*(360/60)/getDegreePerMicroStep();
	maxAccPerSample = maxStepAccPerSecond*1000/sampleRate;

	currentMotorAngle = 0.0;
	lastStepErrorPerSample = 0;
	accel.setup(this, forwardstep, backwardstep);
	accel.setMaxSpeed(maxStepsPerSecond);    // [steps/s], with 24Mhz up to 6000 steps/s is possible
	accel.setAcceleration(maxStepAccPerSecond);

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
	logger->print(maxStepAccPerSecond);
	logger->print(F(" maxSpeed="));
	logger->print(maxStepsPerSecond);
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
	digitalWrite(clockPIN, LOW);
	delayMicroseconds(PIBOT_PULSE_WIDTH_US);
	digitalWrite(clockPIN, HIGH);
	
	// adapt last motor position according the step, if motor is enabled
	if (enabled) { 
		bool direction = currentDirection;	// currently selected direction
		if (direction) {
			currentAngle += anglePerMicroStep;
			currentMotorAngle += getDegreePerMicroStep();
		}
		else {
			currentAngle -= anglePerMicroStep;
			currentMotorAngle -= getDegreePerMicroStep();
		}

		if (currentMotorAngle> 180.0)
			currentMotorAngle -= 360.0;
		if (currentMotorAngle< -180.0)
			currentMotorAngle += 360.0;
	}
}

void GearedStepperDrive::setStepperDirection(bool forward) {
	bool dir = forward?LOW:HIGH;
	uint8_t pin = getPinDirection();

	digitalWrite(pin, dir);
}

inline void GearedStepperDrive::direction(bool dontCache,bool forward) {
	bool toBeDirection = forward;

	if ((toBeDirection != currentDirection) || dontCache)
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
	currentSpeed = 0.0;
	currentFilteredSpeed = 0.0;
}

bool GearedStepperDrive::isEnabled() {
	return enabled;
}

void GearedStepperDrive::disable() {
	enableDriver(false);
}

void GearedStepperDrive::enableDriver(bool ok) {
	// set the clock to low
	digitalWrite(getPinClock(), LOW);
	digitalWrite(getPinEnable(), ok?HIGH:LOW);
	enabled = ok;
}

// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop(uint32_t now) {	
	loop();
}


// computes the next speed on base of currentSpeed
void GearedStepperDrive::computeNewSpeed() {
	// complementary filter to get from currentFilteredSpeed to currentSpeed
	// (maybe linear ramp moves smoother?)
	/*
	const float tau = ENCODER_SAMPLE_RATE*5;
	const float alpha = tau/(tau+ENCODER_SAMPLE_RATE);
*/
	// currentFilteredSpeed = (1.0-alpha) * currentSpeed + alpha * currentFilteredSpeed;
	// accel.setSpeed(currentSpeed);
	accel.move(currentSpeed);
	// accel.move(currentSpeed);
}

// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop() {
	if (accel.runSpeed()) {
		accel.computeNewSpeed();
	}
}

float GearedStepperDrive::getCurrentAngle() {
	return currentAngle;
}

void GearedStepperDrive::setCurrentAngle(float angle) {
	currentMotorAngle = angle*getGearReduction();
	currentAngle = angle;
}

void GearedStepperDrive::setMeasuredAngle(float pMeasuredActuatorAngle, uint32_t now) { 
	currentMotorAngle = pMeasuredActuatorAngle*getGearReduction();
	currentAngleAvailable = true;
	currentAngle = pMeasuredActuatorAngle;

	
	if (!movement.isNull()) {

		float toBeMotorAngle = movement.getCurrentAngle(now)*getGearReduction();

		// compute error in steps per sample
		float stepErrorPerSample= (toBeMotorAngle  - currentMotorAngle) / getDegreePerMicroStep();

		// PID controller works with set point of position, i.e. it computes a correction of the position
		// which is converted in to change of speed (=acceleration)
		float Pout = configData->kP * stepErrorPerSample;
		integral += stepErrorPerSample * dT;
		float Iout = configData->kI * integral;
		float Dout = configData->kD * (stepErrorPerSample - lastStepErrorPerSample) * rezi_dT;
		lastStepErrorPerSample = stepErrorPerSample;
		float output = Pout + Iout + Dout;
		float accelerationPerSample = output;

		accelerationPerSample = constrain(accelerationPerSample, -maxAccPerSample,maxAccPerSample);
		currentSpeed = constrain(accelerationPerSample, -maxStepsPerSecond, maxStepsPerSecond);
		computeNewSpeed();

		// computeNewSpeed(); // filter currentSpeed

		if ((configData->id == 4) && memory.persMem.logStepper) {
			logger->print("angle=");
						logger->print(toBeMotorAngle);
						logger->print("curr=");
						logger->print(currentMotorAngle);

						logger->print(" steperror=");
						logger->print(stepErrorPerSample);
						logger->print(" output=");
						logger->print(output);
						logger->print(" accelerationPerSample=");
						logger->print(accelerationPerSample);
						logger->print(" currentSpeed=");
						logger->println(currentSpeed);

						logger->println(millis());
		}
	} 
}
