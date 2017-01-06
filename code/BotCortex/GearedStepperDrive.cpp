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
	lastStepErrorPerSample = 0;
	accel.setup(this, forwardstep, backwardstep);
	accel.setMaxSpeed(maxStepRatePerSecond);    // [steps/s], with 24Mhz up to 6000 steps/s is possible
	accel.setAcceleration(maxAcceleration);		

	if (memory.persMem.logSetup) {
		logger->print(F("   "));
		pConfigData->print();
		logger->print(F("   "));
	}

	logger->print(F("microsteps="));
	logger->print(getMicroSteps());

	logger->print(F(" degreePerMicroStep="));
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
			currentMotorAngle += configData->degreePerMicroStep;
		}
		else {
			currentMotorAngle -= configData->degreePerMicroStep;
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
	const float tau = ENCODER_SAMPLE_RATE;
	const float alpha = tau/(tau+ENCODER_SAMPLE_RATE);
	currentFilteredSpeed = (1.0-alpha) * currentSpeed + alpha * currentFilteredSpeed;
	accel.setSpeed(currentFilteredSpeed);
}

// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop() {
#ifdef NEWSTEPPERCONTROL
	if (accel.runSpeed())
		computeNewSpeed();
#else
	accel.run();
#endif
}

float GearedStepperDrive::getCurrentAngle() {
	return currentMotorAngle / getGearReduction();
}

void GearedStepperDrive::setCurrentAngle(float angle) {
	currentMotorAngle = angle*getGearReduction();
}

void GearedStepperDrive::setMeasuredAngle(float pMeasuredActuatorAngle, uint32_t now) { 
	currentMotorAngle = pMeasuredActuatorAngle*getGearReduction();
	currentAngleAvailable = true;
	
	if (!movement.isNull()) {
		float toBeMotorAngle = movement.getCurrentAngle(now)*getGearReduction();

		// compute error in steps per sample
		float stepErrorPerSample= (toBeMotorAngle  - currentMotorAngle) / configData->degreePerMicroStep;
		
		// PID controller works with set point of position, i.e. it computes a correction of the position
		// which is converted in to change of speed (=acceleration)
		const float dT = float(ENCODER_SAMPLE_RATE)/1000.0;
		const float rezi_dT = 1.0/dT;
		float Pout = configData->kP * stepErrorPerSample;
		integral += stepErrorPerSample * dT;
		float Iout = configData->kG * integral;
		float Dout = configData->kD * (stepErrorPerSample - lastStepErrorPerSample) * rezi_dT;
		lastStepErrorPerSample = stepErrorPerSample;
		float output = Pout + Iout + Dout;
		float accelerationPerSample = output;


		// compute steps out of degrees	
		/*
		if (configData->id == 4) {
			logger->print("steps");
			logger->print(steps);
			logger->print(" speed");
			logger->print(angleError/configData->degreePerMicroStep*1000/ENCODER_SAMPLE_RATE);
			logger->print(" ");

		logger->println(millis());
		}
		*/

#ifdef NEWSTEPPERCONTROL
		float maxAccPerSample = getMaxStepAccPerSeconds()*1000/ENCODER_SAMPLE_RATE;
		float maxSpeed = getMaxStepsPerSeconds();
		accelerationPerSample = constrain(accelerationPerSample, -maxAccPerSample,maxAccPerSample);
		currentSpeed += accelerationPerSample; // equals the I in PID controller. But lets keep it simple.
		currentSpeed = constrain(currentSpeed, -maxSpeed, maxSpeed);
		computeNewSpeed(); // filter currentSpeed
#else
		accel.move(accelerationPerSample);
#endif

		if ((configData->id == 4) && memory.persMem.logStepper) {
			logger->print(F("stepper.setMeasurement["));
			logActuator(configData->id);
			logger->print(F("](t="));
			logger->print(movement.getRatioDone(now));

			logger->print(F("](tobe="));
			logger->print(toBeMotorAngle);
			logger->print(F(" meas="));
			logger->print(pMeasuredActuatorAngle);
			logger->print(F(" is="));
			logger->print(currentMotorAngle);
			logger->print(F(" stepError="));
			logger->print(accelerationPerSample);
			logger->println(")");
		}
	} 
}
