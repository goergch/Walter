
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
    pivController.setTunings(2.0, 5.0,1.0);
    pivController.setControllerDirection(DIRECT);
	pivController.setSampleTime(MOTOR_SAMPLE_RATE);
	
	currentAngle = getRawAngle();
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
		
		// now compute the future value in MOTOR_SAMPLE_RATE[ms], i.e. take the new angle and add the difference
		if (movement.timeInMovement(now+sampleRate))
			newAngle += movement.getCurrentAngle(now+sampleRate);
			
		// set it
		if (previousLoopCall > 0)
			setRawAngle(newAngle, previousLoopCall);
	}
	previousLoopCall = now;
}