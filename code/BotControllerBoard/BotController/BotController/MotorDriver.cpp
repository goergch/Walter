
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
	mostRecentAngle = 0;
}

void MotorDriver::setup(int number) { 
	hasBeenInitialized = true;
	config = &(memory.persistentMem.motorConfig[number]);
	myMotorNumber = number;
	previousLoopCall = 0;
	setPIVParams();
    pivController.setControllerDirection(DIRECT);
	pivController.setSampleTime(STEPPER_SAMPLE_RATE);
	
	mostRecentAngle = getCurrentAngle();
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


	
void MotorDriverConfig::print() {
	Serial.print(F(" nullAngle="));
	Serial.print(nullAngle,1);
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

void MotorDriverConfig::setDefaults() {
	for (int i = 0;i<MAX_MOTORS;i++) {
		memory.persistentMem.motorConfig[i].nullAngle = 0.0;
		memory.persistentMem.motorConfig[i].maxSpeed= 720.0/1000.0; // 
		memory.persistentMem.motorConfig[i].reverse = false;
		memory.persistentMem.motorConfig[i].minAngle= -160.0;
		memory.persistentMem.motorConfig[i].maxAngle= +160.0;
		
		memory.persistentMem.motorConfig[i].pivKp = 0.8;
		memory.persistentMem.motorConfig[i].pivKi = 0.5;
		memory.persistentMem.motorConfig[i].pivKd = 0.0;
	}		
}


void MotorDriver::setAngle(float pAngle,uint32_t pAngleTargetDuration) {
	uint32_t now = millis();
	static float lastAngle = 0;
	if (abs(lastAngle-pAngle)> 1) {
		Serial.print("setAngle(");
		Serial.print(pAngle);
		Serial.print(" now=");
		Serial.print(now);
		Serial.print(" duration=");
		Serial.print(pAngleTargetDuration);
		Serial.println(") ");
		lastAngle = pAngle;
	}
	movement.set(getCurrentAngle(), pAngle, now, pAngleTargetDuration);
	// movement.print();
	// Serial.println();
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
	Serial.print(getCurrentAngle(),1);
}


void MotorDriver::loop() {
	// control loop for a single motor. We have the  macro movement and execute it by applying a PIV controller.
	// Major difference to PID controller is, that the derivative part is not derived by the position error but by 
	// the position itself.
	// See also http://controlguru.com/pid-control-and-derivative-on-measurement/ and
	// http://www.parkermotion.com/whitepages/ServoFundamentals.pdf
	uint32_t now = millis();
	uint32_t sampleRate = now - previousLoopCall;

	if (!movement.isNull()) {
		// movement.setTime(now);
		// is time over of this movement?
		/*
		Serial.print("now=");
		Serial.print(now);
		Serial.print(" ");
		movement.print();
		Serial.println();
		*/
		float toBeAngle = movement.getCurrentAngle(now);
		float asIsAngle = getCurrentAngle();

		// apply PIV controller
		float newAngle = 0;
		/*
		Serial.print("asis=");
		Serial.print(asIsAngle);
		Serial.print("tobe=");
		Serial.print(toBeAngle);
		*/
		
		pivController.compute(asIsAngle, toBeAngle, newAngle);
		// newAngle = toBeAngle;
		
		// Serial.print("newAnglepiv");
		// Serial.print(newAngle);
		
		// limit by max speed and by max angle
		float maxAngleDiff = memory.persistentMem.motorConfig[myMotorNumber].maxSpeed*SERVO_SAMPLE_RATE;
		newAngle = constrain(newAngle, mostRecentAngle-maxAngleDiff,mostRecentAngle+maxAngleDiff); // limit max speed
		newAngle = constrain(newAngle, memory.persistentMem.motorConfig[myMotorNumber].minAngle, memory.persistentMem.motorConfig[myMotorNumber].maxAngle); 
			
		// set the new angle according to the next loop
		if (previousLoopCall > 0) // start at second loop
		{ 
			moveToAngle(newAngle, SERVO_SAMPLE_RATE); // stay at same position after this movement
			mostRecentAngle = newAngle; 
		}
	}
	previousLoopCall = now;
}