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

		if (abs(lastAngle-pAngle)> 0.1) {
			lastAngle = pAngle;
			/*
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
			*/
		}

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

// called very often to execute one stepper step. Dont do complex operations here.
unsigned long GearedStepperDrive::loop() {
	return accel.run();
}


unsigned long GearedStepperDrive::getNextStepTime() {
	return accel.nextStepTime();
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
		if (fabs(toBeMotorAngle) > 10000.0) {
			logger->println("BUG");
			logger->print("now=");
			logger->print(now);
			logger->print("start");
			logger->print(movement.startTime);
			logger->print("end");
			logger->print(movement.endTime);
			logger->print("angleend");
			logger->print(movement.angleEnd);
			logger->print("angleStart");
			logger->print(movement.angleStart);

			movement.print(configData->id);
		}
		float angleError= toBeMotorAngle  - currentMotorAngle;
		
		// proportional part of PD controller
		float Pout = configData->kP * angleError;
		
		// derivative part of PD controller. Needs to be calibrated together with the
		// stepper library that limits the acceleration
		const float dT = float(ENCODER_SAMPLE_RATE)/1000.0;
		const float rezi_dT = 1.0/dT;

		integral += angleError * dT;
		float Iout = configData->kG * integral;
		float Dout = configData->kD * (angleError - pid_pre_error) * rezi_dT;		

		pid_pre_error = angleError;

		float output = Pout + Iout + Dout  ;

		// compute steps out of degrees	
		accel.move(output/configData->degreePerMicroStep);		
		
		if ((configData->id == 4) && memory.persMem.logStepper) {
			logger->print(F("stepper.setMeasurement["));
			logActuator(configData->id);
			logger->print(F("](t="));
			logger->print(movement.getRatioDone(now));

			logger->print(F("](tobe="));
			logger->print(toBeMotorAngle);
			logger->print(F(" meas="));
			logger->print(pMeasuredAngle);
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
