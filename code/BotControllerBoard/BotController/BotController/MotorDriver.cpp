
/*
 * MotorDriver.cpp
 *
 * Created: 21.04.2016 11:52:07
 *  Author: JochenAlt
 */ 


#include "Arduino.h"
#include "setup.h"
#include "MotorDriver.h"
#include "BotMemory.h"
#include <avr/wdt.h>
#include "stdint.h"




MotorDriver::MotorDriver() {
	hasBeenInitialized = false;
	angleTarget = 0;
	angleTargetStartTime = 0;
	angleTargetEndTime = 0;

	currentAngle = 0.0;
}

void MotorDriver::setup(int number) { 
	hasBeenInitialized = true;
	config = &(memory.persistentMem.motorConfig[number]);
	myMotorNumber = number;
}

	
void MotorDriverConfig::println() {
	Serial.print(nullAngle,1);
	Serial.println();
}

void MotorDriverConfig::setDefaults() {
	for (int i = 0;i<MAX_MOTORS;i++) {
		memory.persistentMem.motorConfig[i].nullAngle = 0.0;
		memory.persistentMem.motorConfig[i].maxSpeed= 360.0/1000.0;
		memory.persistentMem.motorConfig[i].maxAcceleration= 10*360.0/1000.0;
		memory.persistentMem.motorConfig[i].reverse = false;
		memory.persistentMem.motorConfig[i].maxAngle= +160.0;
		memory.persistentMem.motorConfig[i].maxAngle= -160.0;
	}		
}


void MotorDriver::setAngleTarget(float pAngle,long pAngleTargetDuration) {
	// in case there has been a target already, overwrite it
	angleTargetTime = millis() + pAngleTargetDuration;
	angleTarget = pAngle;
	currentAngle = getAngle();
}


void MotorDriver::addToNullPosition(float addToNullAngle) {
	memory.persistentMem.motorConfig[myMotorNumber].nullAngle += addToNullAngle;
}

float MotorDriver::getNullPosition() {
	return memory.persistentMem.motorConfig[myMotorNumber].nullAngle;
}

void MotorDriver::print() {
	Serial.print("angle[");
	Serial.print(myMotorNumber);
	Serial.print("]=");
	Serial.print(getAngle(),1);
}

void MotorDriver::println() {
	print();
}

void MotorDriver::loop() {
	if (angleTargetTime != 0) {
		uint32_t now = millis();
		uint32_t durationToGo_ms = angleTargetTime-now;
		float toBeAngle = float(now - angleTargetStartTime)/float(angleTargetEndTime-angleTargetStartTime);
		float currentAngle = getAngle();
		setAngle()
	}
}