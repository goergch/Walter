
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
}

void Actuator::setup(int number) { 
	hasBeenInitialized = true;
	config = &(memory.persMem.armConfig[number]);
	myMotorNumber = number;
	previousLoopCall = 0;
	setPIVParams();
    pivController.setControllerDirection(DIRECT);
	pivController.setSampleTime(ANGLE_SAMPLE_RATE);
	
	mostRecentAngle = getCurrentAngle();
	beforeFirstMove = true;
	movement.setNull();
}

void Actuator::setPIVParams() {
    pivController.setTunings(	
		memory.persMem.armConfig[myMotorNumber].pivKp,
	    memory.persMem.armConfig[myMotorNumber].pivKi,
	    memory.persMem.armConfig[myMotorNumber].pivKd);
	pivController.setOutputLimits(
		memory.persMem.armConfig[myMotorNumber].minAngle,
		memory.persMem.armConfig[myMotorNumber].maxAngle);

	
}


	
void ArmConfig::print() {
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

void ArmConfig::setDefaults() {
	for (int i = 0;i<MAX_MOTORS;i++) {
		memory.persMem.armConfig[i].maxSpeed= 720.0/1000.0; // 
		memory.persMem.armConfig[i].reverse = false;

		memory.persMem.armConfig[i].nullAngle = 0.0;		
		memory.persMem.armConfig[i].minAngle= -160.0;
		memory.persMem.armConfig[i].maxAngle= +160.0;
		
		memory.persMem.armConfig[i].pivKp = 0.8;
		memory.persMem.armConfig[i].pivKi = 0.5;
		memory.persMem.armConfig[i].pivKd = 0.0;
	}		
}

void Actuator::addToNullPosition(float addToNullAngle) {
	memory.persMem.armConfig[myMotorNumber].nullAngle += addToNullAngle;
}

float Actuator::getNullPosition() {
	return memory.persMem.armConfig[myMotorNumber].nullAngle;
}

void Actuator::printConfiguration() {
	Serial.print("angle[");
	Serial.print(myMotorNumber);
	Serial.print("]=");
	Serial.print(getCurrentAngle(),1);
}

/*
void MotorDriver::loop() {
	// control loop for a single motor. We have the  macro movement and execute it by applying a PIV controller.
	// Major difference to PID controller is, that the derivative part is not derived by the position error but by 
	// the position itself.
	// See also http://controlguru.com/pid-control-and-derivative-on-measurement/ and
	// http://www.parkermotion.com/whitepages/ServoFundamentals.pdf
	uint32_t now = millis();

	if (!movement.isNull()) {
		// movement.setTime(now);
		// is time over of this movement?
		// Serial.print("now=");
		// Serial.print(now);
		// Serial.print(" ");
		// movement.print();
		// Serial.println();
		float toBeAngle = movement.getCurrentAngle(now);
		float asIsAngle = getCurrentAngle();

		// apply PIV controller
		float newAngle = 0;
		Serial.print("asis=");
		Serial.print(asIsAngle);
		Serial.print("tobe=");
		Serial.print(toBeAngle);
		
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

*/