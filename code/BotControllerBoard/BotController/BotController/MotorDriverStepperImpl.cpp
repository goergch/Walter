/* 
* MotorDriverStepperImpl.cpp
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/

#include "Arduino.h"
#include "MotorDriverStepperImpl.h"
#include "digitalWriteFast.h"


void MotorDriverStepperImpl::setup(int motorNumber) {
	if ((motorNumber < 1) || (motorNumber>MAX_MOTORS)) {
		Serial.print(F("setup stepper error"));
		delay(100);
		exit(0);
	};
	MotorDriver::setup(motorNumber);

	pinMode(getPinClock(), OUTPUT);
	pinMode(getPinDirection(), OUTPUT);
	pinMode(getPinEnable(), OUTPUT);
	enabled = false;
	enable(true);	
	
	// set to default direction
	direction(currentDirection);
	
	// no movement currently
	movement.setNull();

	// define max speed in terms of ticks per step	
	minTicksPerStep = (1000000L/STEPPER_SAMPLE_RATE_US)/getMaxStepRate()*getMicroSteps();
	if (minTicksPerStep<1)
		minTicksPerStep = 1;
	Serial.print("ticks");
	Serial.println(minTicksPerStep );

	degreePerActualSteps = getDegreePerStep()/getMicroSteps();
	
	Serial.print("degreePersteps");
	Serial.println(getActualDegreePerStep() );
	
}

void MotorDriverStepperImpl::performStep() {
	digitalWrite(getPinClock(), LOW);  // This LOW to HIGH change is what creates the
	digitalWrite(getPinClock(), HIGH);


/*	uint8_t clockPIN = getPinClock();
	digitalWriteFast(clockPIN, LOW);
	digitalWriteFast(clockPIN, HIGH);
*/
	if (currentDirection) {
		currentAngle += degreePerActualSteps;
	}
	else {
		currentAngle -= degreePerActualSteps;
	}
}

void MotorDriverStepperImpl::direction(bool forward) {
	bool toBeDirection = forward;

	if (toBeDirection != currentDirection) {
		bool dir = toBeDirection?LOW:HIGH;
		if (!getDirection())
			dir=!dir;
		uint8_t pin = getPinDirection();
		digitalWriteFast(pin, dir);
		// digitalWrite(getPinDirection(), toBeDirection?LOW:HIGH);
		currentDirection = toBeDirection;
	}
}

void MotorDriverStepperImpl::enable(bool ok) {
	if (enabled != ok) {
		digitalWrite(getPinEnable(), ok?HIGH:LOW);  // This LOW to HIGH change is what creates the
		enabled = ok;
	}
}

// called very often to execute one stepper step. Dont do complex operations here.
void MotorDriverStepperImpl::loop() {
	tickCounter++;

	bool allowedToMove = tickCounter>=getMinTicksPerStep();
	
	if (allowedToMove && !movement.isNull()) {
		uint32_t now = millis();
		if (movement.timeInMovement(now)) {
			// interpolate tobe angle
			float toBeAngle = movement.getCurrentAngle(now);
		
			// carry out one step per loop 
			if ((abs(toBeAngle-currentAngle) > getActualDegreePerStep())) {
				// select direction
				direction(toBeAngle>currentAngle);

				// one step
				performStep();	
				
				// reset counter that ensures that max speed is not exceeded
				tickCounter = 0;
			}
		} else {
			movement.setNull();
		}
	}
}
void MotorDriverStepperImpl::moveToAngle(float angle, uint32_t pDuration_ms) {
	movement.set(currentAngle, angle, millis(), pDuration_ms);
}

float MotorDriverStepperImpl::getCurrentAngle() {
	return currentAngle;
}

void MotorDriverStepperImpl::setMeasuredAngle(float pMeasuredAngle) { 
	measuredAngle = pMeasuredAngle;
	currentAngle = measuredAngle;
}
