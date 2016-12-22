
/*
 * MotorDriver.cpp
 *
 * Created: 21.04.2016 11:52:07
 *  Author: JochenAlt
 */ 


#include "Arduino.h"
#include "Actuator.h"
#include "BotMemory.h"
#include <avr/wdt.h>
#include "stdint.h"
#include "utilities.h"


Actuator::Actuator() {
	hasBeenInitialized = false;
	encoder = NULL;
}

void Actuator::setup(ActuatorConfig* pConfigData, GearedStepperDrive* pStepper, RotaryEncoder* pEncoder) {
	encoder = pEncoder;
	stepperDrive = pStepper;
	configData = pConfigData;
	servoDrive = NULL;
	setup();
}

void Actuator::setup(ActuatorConfig* pConfigData, HerkulexServoDrive* servo) {
	encoder = NULL;
	stepperDrive = NULL;
	configData = pConfigData;
	servoDrive = servo;
	setup();
}

void Actuator::setup() {
	hasBeenInitialized = true;
	previousLoopCall = 0;	
}


void Actuator::printName() {
	logActuator(configData->id);		
}

void Actuator::printConfiguration() {
	logger->print(F("angle["));
	logger->print(configData->id);
	logger->print(F("]="));
	logger->print(getCurrentAngle(),1);
}

void Actuator::setMaxAngle(float angle) {
	if (configData) {
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			configData->config.stepperArm.stepper.maxAngle = angle;
		if (configData->actuatorType == SERVO_TYPE)
			configData->config.servoArm.servo.maxAngle = angle;
	}
}
void Actuator::setMinAngle(float angle) {
	if (configData) {
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			configData->config.stepperArm.stepper.minAngle = angle;
		if (configData->actuatorType == SERVO_TYPE)
			configData->config.servoArm.servo.minAngle= angle;
	}
}
void Actuator::setNullAngle(float angle) {
	if (configData) {
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			configData->config.stepperArm.encoder.nullAngle = angle;
		if (configData->actuatorType == SERVO_TYPE)
			configData->config.servoArm.servo.nullAngle = angle;
	}
}

float Actuator::getNullAngle() {
	if (configData) {
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			return configData->config.stepperArm.encoder.nullAngle;
		if (configData->actuatorType == SERVO_TYPE)
			return configData->config.servoArm.servo.nullAngle;
	}
	return 0.;
}


float Actuator::getMaxAngle() {
	if (configData) {
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			return configData->config.stepperArm.stepper.maxAngle;
		if (configData->actuatorType == SERVO_TYPE)
			return configData->config.servoArm.servo.maxAngle;
	}
	return 0.0;
}

float Actuator::getMinAngle() {
	if (configData) {
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			return configData->config.stepperArm.stepper.minAngle;
		if (configData->actuatorType == SERVO_TYPE)
			return configData->config.servoArm.servo.minAngle;
	}
	return 0.0;
}

void Actuator::setD(float D) {
	if (configData) 
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			configData->config.stepperArm.stepper.kD = D;
}

void Actuator::setP(float P) {
	if (configData->actuatorType == STEPPER_ENCODER_TYPE) {
		if (configData)
			configData->config.stepperArm.stepper.kP = P;
	}
}

void Actuator::setMaxSpeed(float maxSpeed) {
	if (configData)
		if (configData->actuatorType == STEPPER_ENCODER_TYPE) 
			configData->config.stepperArm.stepper.maxSpeed= maxSpeed;
}

void Actuator::setMaxAcc(float maxAcc) {
	if (configData)
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			configData->config.stepperArm.stepper.maxAcc= maxAcc;
}

bool Actuator::setCurrentAsNullPosition() {
	float avr, variance;
	if (configData) {
		if (configData && configData->actuatorType == STEPPER_ENCODER_TYPE) {
			if (getEncoder().isOk() && getEncoder().fetchSample(avr, variance)) {
				// get currrent angle of motor in order to set null position of motor and encoder. 
				getEncoder().setNullAngle(avr);
				getStepper().setCurrentAngle(0);		
				return true; // success	
			}
		}
		if (configData && configData->actuatorType == SERVO_TYPE) {
			if (getServo().isOk()) {
				getServo().setNullAngle(getServo().getCurrentAngle());	
				getServo().setAngle(0,1);
				
				return true;	
			}
		}
	}
	return false;
}
