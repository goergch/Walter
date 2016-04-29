
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
	currentAngle = 0;
}

void MotorDriver::setup(int number) { 
	hasBeenInitialized = true;
	config = &(memory.persistentMem.motorConfig[number]);
	myMotorNumber = number;
	previousLoopCall = 0;
	setPIVParams();
    pivController.setControllerDirection(DIRECT);
	pivController.setSampleTime(MOTOR_SAMPLE_RATE);
	
	currentAngle = getRawAngle();
}

void MotorDriver::setPIVParams() {
    pivController.setTunings(	
		memory.persistentMem.motorConfig[myMotorNumber].pivKp,
	    memory.persistentMem.motorConfig[myMotorNumber].pivKi,
	    memory.persistentMem.motorConfig[myMotorNumber].pivKd);
	pivController.setOutputLimits(
		memory.persistentMem.motorConfig[myMotorNumber].minAngle,
		memory.persistentMem.motorConfig[myMotorNumber].maxAngle);

	
}


	
void MotorDriverConfig::println() {
	Serial.print(nullAngle,1);
	Serial.println();
}

void MotorDriverConfig::setDefaults() {
	for (int i = 0;i<MAX_MOTORS;i++) {
		memory.persistentMem.motorConfig[i].nullAngle = 0.0;
		memory.persistentMem.motorConfig[i].maxSpeed= 360.0/1000.0;
		memory.persistentMem.motorConfig[i].reverse = false;
		memory.persistentMem.motorConfig[i].minAngle= -160.0;
		memory.persistentMem.motorConfig[i].maxAngle= +160.0;
		
		memory.persistentMem.motorConfig[i].pivKp = 2.0;
		memory.persistentMem.motorConfig[i].pivKi = 5.0;
		memory.persistentMem.motorConfig[i].pivKd = 1.0;
	}		
}


void MotorDriver::setAngleTarget(float pAngle,long pAngleTargetDuration) {
	uint32_t now = millis();
	movement.set(getRawAngle(),now, pAngle, pAngleTargetDuration);
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
	Serial.print(getRawAngle(),1);
}

void MotorDriver::println() {
	print();
}

void MotorDriver::loop() {
	// control loop for a single motor. We have the  macro movement and execute it by applying a PIV controller.
	// Major difference to PID controller is, that the derivative part is not derived by the position error but by 
	// the position itself.
	// See also http://controlguru.com/pid-control-and-derivative-on-measurement/ and
	// http://www.parkermotion.com/whitepages/ServoFundamentals.pdf

	uint32_t now = millis();
	uint32_t sampleRate = now - previousLoopCall;
	movement.setTime(now);

	if (!movement.isNull()) {
		// is time over of this movement?
		float toBeAngle = movement.getCurrentAngle(now);
		currentAngle = getRawAngle();
		float asIsAngle = currentAngle;

		// apply PIV controller
		float newAngle = 0;
		pivController.compute(asIsAngle, toBeAngle, newAngle);
		
		// limit my max speed
		float maxAngleDiff = memory.persistentMem.motorConfig[myMotorNumber].maxSpeed*sampleRate;
		if ((newAngle > currentAngle + maxAngleDiff) || (newAngle < currentAngle - maxAngleDiff)) {
			Serial.print("MaxSpeed[");Serial.print(myMotorNumber);Serial.println("]");
		}
		newAngle = constrain(newAngle, currentAngle-maxAngleDiff,currentAngle+maxAngleDiff);
	
		// now compute the future value in MOTOR_SAMPLE_RATE[ms], i.e. take the new angle and add the difference
		float nextSampleDiff = 0;
		if (movement.timeInMovement(now+sampleRate))
			nextSampleDiff = movement.getCurrentAngle(now+sampleRate);
		newAngle += nextSampleDiff;
		
		// set the new angle according to the next loop
		if (previousLoopCall > 0) // start at second loop
		{ 
			if (nextSampleDiff == 0.0)
				setRawAngle(newAngle, sampleRate, newAngle,sampleRate+sampleRate); // stay at same position after this movement
			else
				setRawAngle(newAngle,sampleRate,newAngle+nextSampleDiff, sampleRate+sampleRate);			
		}
	}
	previousLoopCall = now;
}