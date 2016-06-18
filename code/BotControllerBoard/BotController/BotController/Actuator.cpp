
/*
 * MotorDriver.cpp
 *
 * Created: 21.04.2016 11:52:07
 *  Author: JochenAlt
 */ 


#include "Arduino.h"
#include "setup.h"
#include "Actuator.h"
#include "BotMemory.h"
#include <avr/wdt.h>
#include "stdint.h"


Actuator::Actuator() {
	hasBeenInitialized = false;
	encoder = NULL;
}

void Actuator::setup(ActuatorConfigurator* pConfigData, ActuatorSetupData* pSetupData, GearedStepperDrive* pStepper, RotaryEncoder* pEncoder) {
	encoder = pEncoder;
	stepperDrive = pStepper;
	configData = pConfigData;
	setupData = pSetupData;
	servoDrive = NULL;
	setup();
}

void Actuator::setup(ActuatorConfigurator* pConfigData, ActuatorSetupData* pSetupData, HerkulexServoDrive* servo) {
	encoder = NULL;
	stepperDrive = NULL;
	configData = pConfigData;
	setupData = pSetupData;
	servoDrive = servo;
	setup();
}

void Actuator::setup() {
	hasBeenInitialized = true;
	previousLoopCall = 0;	
}


void Actuator::printName() {
	printActuator(configData->id);		
}

void Actuator::printConfiguration() {
	Serial.print("angle[");
	Serial.print(configData->id);
	Serial.print("]=");
	Serial.print(getCurrentAngle(),1);
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
float Actuator::getMaxAngle() {
	if (configData) {
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			return configData->config.stepperArm.stepper.maxAngle;
		if (configData->actuatorType == SERVO_TYPE)
			return configData->config.servoArm.servo.maxAngle;
	}
}

float Actuator::getMinAngle() {
	if (configData) {
		if (configData->actuatorType == STEPPER_ENCODER_TYPE)
			return configData->config.stepperArm.stepper.minAngle;
		if (configData->actuatorType == SERVO_TYPE)
			return configData->config.servoArm.servo.minAngle;
	}
}

bool Actuator::setCurrentAsNullPosition() {
	float avr, variance;
	if (configData) {
		if (configData && configData->actuatorType == STEPPER_ENCODER_TYPE) {
			if (getEncoder().isOk() && getEncoder().fetchSample(true, avr, variance)) {
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
