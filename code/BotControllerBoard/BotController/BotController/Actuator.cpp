
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
	beforeFirstMove  = true;
	encoder = NULL;
}

void Actuator::setup(int number) { 
	hasBeenInitialized = true;
	config = &(memory.persMem.armConfig[number]);
	myActuatorNumber = number;
	previousLoopCall = 0;
	encoder = NULL;

	setPIVParams();
    pivController.setControllerDirection(DIRECT);
	pivController.setSampleTime(ANGLE_SAMPLE_RATE);
	
	mostRecentAngle = getCurrentAngle();
	beforeFirstMove = true;
	movement.setNull();
}


void Actuator::setPIVParams() {
    pivController.setTunings(	
		memory.persMem.armConfig[myActuatorNumber].pivKp,
	    memory.persMem.armConfig[myActuatorNumber].pivKi,
	    memory.persMem.armConfig[myActuatorNumber].pivKd);
	pivController.setOutputLimits(
		memory.persMem.armConfig[myActuatorNumber].minAngle,
		memory.persMem.armConfig[myActuatorNumber].maxAngle);

	
}


	
void ArmConfig::print() {
	Serial.print(F(" encoderNullAngle="));
	Serial.print(encoderNullAngle,1);
	Serial.print(F(" maxSpeed="));
	Serial.print(maxSpeed,1);
	Serial.print(F(" minAngle="));
	Serial.print(minAngle,1);
	Serial.print(F(" maxAngle="));
	Serial.print(maxAngle,1);
	Serial.print(F(" Kp="));
	Serial.print(pivKp,1);
	Serial.print(F(" Ki="));
	Serial.print(pivKi,1);
	Serial.print(F(" Kv="));
	Serial.print(pivKd,1);

}

void ArmConfig::setDefaults() {
	// set default for all
	for (int i = 0;i<MAX_MOTORS;i++) {
		memory.persMem.armConfig[i].maxSpeed= 720.0/1000.0; // 		
		memory.persMem.armConfig[i].pivKp = 0.8;
		memory.persMem.armConfig[i].pivKi = 0.5;
		memory.persMem.armConfig[i].pivKd = 0.0;
		memory.persMem.armConfig[i].nullAngle = 0.0;
		memory.persMem.armConfig[i].minAngle= -160.0;
		memory.persMem.armConfig[i].maxAngle= +160.0;
		memory.persMem.armConfig[i].encoderNullAngle= 0.0;
	}		
	
	// overwrite defauls where necessary
	// Wrist Turn (herkulex Servo)
	memory.persMem.armConfig[0].nullAngle = 0.0;
	memory.persMem.armConfig[0].minAngle= -120.0;
	memory.persMem.armConfig[0].maxAngle= +120.0;

	// Wrist Nick (stepper/Encoder)
	memory.persMem.armConfig[1].nullAngle = 0.0;
	memory.persMem.armConfig[1].minAngle= -100.0;
	memory.persMem.armConfig[1].maxAngle= +100.0;
	memory.persMem.armConfig[1].encoderNullAngle= -286.0;

}


void Actuator::printConfiguration() {
	Serial.print("angle[");
	Serial.print(myActuatorNumber);
	Serial.print("]=");
	Serial.print(getCurrentAngle(),1);
}

void Actuator::setMaxAngle(float angle) {
	memory.persMem.armConfig[myActuatorNumber].maxAngle = angle;
}
void Actuator::setMinAngle(float angle) {
		memory.persMem.armConfig[myActuatorNumber].minAngle = angle;
}
float Actuator::getMaxAngle() {
	return memory.persMem.armConfig[myActuatorNumber].maxAngle;
}
float Actuator::getMinAngle() {
	return memory.persMem.armConfig[myActuatorNumber].minAngle;;
}
