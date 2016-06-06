/* 
* MotorDriverStepperImpl.cpp
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/

#include "Arduino.h"
#include "GearedStepperDrive.h"
#include "digitalWriteFast.h"


void GearedStepperDrive::setup(int motorNumber) {
	if ((motorNumber < 1) || (motorNumber>MAX_MOTORS)) {
		Serial.print(F("setup stepper error"));
		delay(100);
		exit(0);
	};
	Actuator::setup(motorNumber);

	pinMode(getPinClock(), OUTPUT);
	pinMode(getPinDirection(), OUTPUT);
	pinMode(getPinEnable(), OUTPUT);
	enabled = false;
	enable(true);	
	
	// set to default direction
	direction(currentDirection);
	
	// no movement currently
	movement.setNull();

	degreePerActualSteps = getDegreePerStep()/getMicroSteps();
	
	maxStepRatePerSecond  = (360.0/degreePerActualSteps) *(float(getMaxRpm())/60.0);
	
	// define max speed in terms of ticks per step	
	minTicksPerStep = (1000000L/STEPPER_SAMPLE_RATE_US)/getMaxStepRatePerSecond();
	if (minTicksPerStep<1)
		minTicksPerStep = 1;

	currentMotorAngle = 0.0;
	setMeasuredAngle(0.0);
}

void GearedStepperDrive::printConfiguration() {
	Serial.print(F("motor["));
	Serial.print(myActuatorNumber);

	Serial.print(F("]={degreePerSteps="));
		Serial.print(degreePerActualSteps);
		Serial.print(F(",minTicksPerStep="));
		Serial.print(minTicksPerStep);
		Serial.print(F(",maxStepRate="));
		Serial.print(maxStepRatePerSecond);
		Serial.print(F("]"));
	Serial.println();
}

void GearedStepperDrive::changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) {
	// this methods works even when no current Angle has been measured
	movement.set(getCurrentAngle(), getCurrentAngle()+pAngleChange, millis(), pAngleTargetDuration);
}


void GearedStepperDrive::setAngle(float pAngle,uint32_t pAngleTargetDuration) {
	if (currentAngleAvailable) {
		// limit angle
		pAngle = constrain(pAngle, getMinAngle(),getMaxAngle());
		uint32_t now = millis();
		static float lastAngle = 0;
		if (abs(lastAngle-pAngle)> 0.1) {
#ifdef DEBUG_STEPPER			
			Serial.print("stepper.setAngle[");
			Serial.print(myActuatorNumber);
			Serial.print("](");
			Serial.print(pAngle);
			Serial.print(" now=");
			Serial.print(now);
			Serial.print(" duration=");
			Serial.print(pAngleTargetDuration);
			Serial.println(")");
#endif
			lastAngle = pAngle;
			// set actuator angle (not motor angle)
			movement.set(getCurrentAngle(), pAngle, now, pAngleTargetDuration);
		}
	}
}


void GearedStepperDrive::performStep() {
	uint8_t clockPIN = getPinClock();
#ifdef USE_FAST_DIGITAL_WRITE
	digitalWriteFast(clockPIN, LOW);
	digitalWriteFast(clockPIN, HIGH);
#else
	digitalWrite(clockPIN, LOW);  // This LOW to HIGH change is what creates the
	digitalWrite(clockPIN, HIGH);
#endif



	if (currentDirection) {
		currentMotorAngle += degreePerActualSteps;
	}
	else {
		currentMotorAngle -= degreePerActualSteps;
	}
}

void GearedStepperDrive::direction(bool forward) {
	bool toBeDirection = forward;

	if (toBeDirection != currentDirection) {
		bool dir = toBeDirection?LOW:HIGH;
		if (!getDirection())
			dir=!dir;
		uint8_t pin = getPinDirection();
#ifdef USE_FAST_DIGITAL_WRITE		
		digitalWriteFast(pin, dir);
#else
		digitalWrite(pin, dir);
#endif
		currentDirection = toBeDirection;
	}
}

void GearedStepperDrive::enable(bool ok) {
	if (enabled != ok) {
		digitalWrite(getPinEnable(), ok?HIGH:LOW);  // This LOW to HIGH change is what creates the
		enabled = ok;
	}
}

// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop(uint32_t now) {
	
	// this method is called every SERVO_SAMPLE_RATE us.
	// Depending on the maximum speed of the stepper, we count how often we
	// do nothing in order to not increase maximum speed of the stepper such that it does not stall.
	tickCounter++;

	bool allowedToMove = tickCounter>=getMinTicksPerStep(); // true, if we can execute one step

	if (allowedToMove && !movement.isNull()) {
		
		// movement.print(0);
		// Serial.print("now=");
		// Serial.print(now)	;
		float toBeMotorAngle = movement.getCurrentAngle(now)*getGearReduction();
		if ((abs(toBeMotorAngle-currentMotorAngle) > getActualDegreePerStep()/2.0)) {
				// select direction
				direction(toBeMotorAngle>currentMotorAngle);
				// Serial.print("#");
				// Serial.println(toBeAngle);
				/*
				Serial.print("/");
				Serial.print(currentAngle);
				*/
				// one step
				performStep();	
				

				// Serial.print("/");
				//Serial.println(currentAngle);
				
				// reset counter that ensures that max speed is not exceeded
				tickCounter = 0;
		} 
		else {
			if (!movement.timeInMovement(now))
				movement.setNull();
			}
	}
}
void GearedStepperDrive::moveToAngle(float angle, uint32_t pDuration_ms) {
	movement.set(getCurrentAngle(), angle, millis(), pDuration_ms);
}

float GearedStepperDrive::getCurrentAngle() {
	return currentMotorAngle / getGearReduction();
}


void GearedStepperDrive::setMeasuredAngle(float pMeasuredAngle) { 
	currentMotorAngle = pMeasuredAngle*getGearReduction();
	currentAngleAvailable = true;
}
