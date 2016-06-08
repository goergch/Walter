
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
	mostRecentAngle = 0;
	encoder = NULL;
}

void Actuator::setup(ActuatorConfigurator& pConfigData, ActuatorSetupData& pSetupData, GearedStepperDrive& pStepper, RotaryEncoder& pEncoder) {
	encoder = &pEncoder;
	stepperDrive = &pStepper;
	configData = &pConfigData;
	setupData = &pSetupData;
	servoDrive = NULL;
	setup();
}

void Actuator::setup(ActuatorConfigurator& pConfigData, ActuatorSetupData& pSetupData, HerkulexServoDrive& servo) {
	encoder = NULL;
	stepperDrive = NULL;
	configData = &pConfigData;
	setupData = &pSetupData;
	servoDrive = &servo;
	setup();
}

void Actuator::setup() {
	hasBeenInitialized = true;
	previousLoopCall = 0;	

	setPIVParams();
	pivController.setControllerDirection(DIRECT);
	pivController.setSampleTime(ANGLE_SAMPLE_RATE);
	
	mostRecentAngle = getCurrentAngle();
}


void Actuator::setPIVParams() {
    pivController.setTunings(	
		configData->config.stepperArm.stepper.pivKp,
	    configData->config.stepperArm.stepper.pivKi,
	    configData->config.stepperArm.stepper.pivKd);
	pivController.setOutputLimits(
		configData->config.stepperArm.stepper.minAngle,
		configData->config.stepperArm.stepper.maxAngle);

	
}

void Actuator::printName() {
	Serial.print(getName_P(configData->id));
}

void Actuator::printConfiguration() {
	Serial.print("angle[");
	Serial.print(getName_P(configData->id));
	Serial.print("]=");
	Serial.print(getCurrentAngle(),1);
}

void Actuator::setMaxAngle(float angle) {
	configData->config.stepperArm.stepper.maxAngle = angle;
}
void Actuator::setMinAngle(float angle) {
	configData->config.stepperArm.stepper.minAngle = angle;
}
float Actuator::getMaxAngle() {
	configData->config.stepperArm.stepper.maxAngle;
}
float Actuator::getMinAngle() {
	configData->config.stepperArm.stepper.minAngle;
}
