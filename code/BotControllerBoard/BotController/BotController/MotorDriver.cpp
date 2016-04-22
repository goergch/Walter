
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
	angleTargetStart = 0;
	angleTargetEnd = 0;
	angleTargetStartTime = 0;
	angleTargetEndTime = 0;
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
	angleTargetStartTime = millis();
	angleTargetEndTime = angleTargetStartTime + pAngleTargetDuration;
	angleTargetStart = getAngle();
	angleTargetEnd =  pAngle;
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
	if (angleTargetStartTime != 0) {
		uint32_t now = millis();
		float t = float(now - angleTargetStartTime)/float(angleTargetEndTime-angleTargetStartTime); // ratio in time, 0..1
		float toBeAngle = angleTargetStart + t*(angleTargetEnd-angleTargetStart);
		float asIsAngle = getAngle();
		// compute value to set the angle by error 
		float setControlledAngle = toBeAngle;
		
		// now compute the future value in MOTOR_SAMPLE_RATE[ms], i.e. take the setControlledAngle and add the difference
		float nextT = float(now - angleTargetStartTime+MOTOR_SAMPLE_RATE)/float(angleTargetEndTime-angleTargetStartTime); // ratio in time, 0..1
		float nextToBeAngle = angleTargetStart + nextT*(angleTargetEnd-angleTargetStart);
		setControlledAngle += (nextToBeAngle-toBeAngle);
		
		// set it
		setAngle(setControlledAngle, MOTOR_SAMPLE_RATE);
	}
}