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

	currentAngle = 0.0;
	setMeasuredAngle(0.0);
}

void MotorDriverStepperImpl::printConfiguration() {
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



void MotorDriverStepperImpl::setAngle(float pAngle,uint32_t pAngleTargetDuration) {
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
			// motor angle (having gearbox in mind)
			float motorAngle = pAngle * getGearReduction();
			movement.set(getCurrentAngle(), motorAngle, now, pAngleTargetDuration);
		}
		

	}
}


void MotorDriverStepperImpl::performStep() {
	uint8_t clockPIN = getPinClock();
#ifdef USE_FAST_DIGITAL_WRITE
	digitalWriteFast(clockPIN, LOW);
	digitalWriteFast(clockPIN, HIGH);
#else
	digitalWrite(clockPIN, LOW);  // This LOW to HIGH change is what creates the
	digitalWrite(clockPIN, HIGH);
#endif



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
#ifdef USE_FAST_DIGITAL_WRITE		
		digitalWriteFast(pin, dir);
#else
		digitalWrite(pin, dir);
#endif
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
void MotorDriverStepperImpl::loop(uint32_t now) {
	
	// this method is called every SERVO_SAMPLE_RATE us.
	// Depending on the maximum speed of the stepper, we count how often we
	// do nothing in order to not increase maximum speed of the stepper such that it does not stall.
	tickCounter++;

	bool allowedToMove = tickCounter>=getMinTicksPerStep(); // true, if we can execute one step
/*
	if (allowedToMove && !movement.isNull()) {
		float toBeAngle = movement.getCurrentAngle(now);
			if ((abs(toBeAngle-currentAngle) > getActualDegreePerStep()/2.0)) {

				direction(toBeAngle>currentAngle);
				performStep();	
				tickCounter = 0;
				return;
			}
	}
	*/
	if (allowedToMove && !movement.isNull()) {
		
		// movement.print(0);
		// Serial.print("now=");
		// Serial.print(now)	;
		float toBeAngle = movement.getCurrentAngle(now);
		if ((abs(toBeAngle-currentAngle) > getActualDegreePerStep()/2.0)) {
				// select direction
				direction(toBeAngle>currentAngle);
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
void MotorDriverStepperImpl::moveToAngle(float angle, uint32_t pDuration_ms) {
	movement.set(currentAngle, angle, millis(), pDuration_ms);
}

float MotorDriverStepperImpl::getCurrentAngle() {
	return currentAngle;
}

void MotorDriverStepperImpl::setMeasuredAngle(float pMeasuredAngle) { 
	currentAngle = pMeasuredAngle;
	currentAngleAvailable = true;
}
