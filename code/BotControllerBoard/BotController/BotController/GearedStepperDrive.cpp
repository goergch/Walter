/* 
* MotorDriverStepperImpl.cpp
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/

#include "Arduino.h"
#include "GearedStepperDrive.h"
#include "digitalWriteFast.h"


void GearedStepperDrive::setup(	StepperConfig& pConfigData, StepperSetupData& pSetupData) {

	movement.setNull();

	configData = &pConfigData;
	setupData = &pSetupData;

	pinMode(getPinClock(), OUTPUT);
	pinMode(getPinDirection(), OUTPUT);
	pinMode(getPinEnable(), OUTPUT);
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
	Serial.print(getName_P(configData->id));

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
		pAngle = constrain(pAngle, configData->minAngle,configData->maxAngle);
		uint32_t now = millis();
		static float lastAngle = 0;
		if (abs(lastAngle-pAngle)> 0.1) {
#ifdef DEBUG_STEPPER			
			Serial.print("stepper.setAngle[");
			// SerialPrintLn_P(setupData->name_P);
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
	digitalWrite(getPinEnable(), ok?HIGH:LOW);  // This LOW to HIGH change is what creates the
}

#define TICKS_PER_SPEED_SAMPLE (STEPPER_SPEED_SAMPLE_RATE*1000L/STEPPER_SAMPLE_RATE_US)

// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop(uint32_t now) {
	
	// this method is called every SERVO_SAMPLE_RATE us.
	// Depending on the maximum speed of the stepper, we count how often we
	// do nothing in order to not increase maximum speed of the stepper such that it does not stall.
	allowedToMoveTickCounter = (allowedToMoveTickCounter+1)%65535;
	
	
	bool allowedToMove = allowedToMoveTickCounter >= minTicksPerStep; // true, if we can execute one step		
	
	if (allowedToMove) {
		if (!movement.isNull()) {
		
		// movement.print(0);
		// Serial.print("now=");
		// Serial.print(now)	;
		// compute acceleration by computing speed of step
		
		
		float toBeMotorAngle = movement.getCurrentAngle(now)*getGearReduction();
		if ((abs(toBeMotorAngle-currentMotorAngle) > degreePerActualSteps/2.0)) { // is there enough movement for one step?
			
				// select direction
				bool forward = toBeMotorAngle>currentMotorAngle;
				direction(forward);
									
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
				allowedToMoveTickCounter = 0;
		} 
		else {
			if (!movement.timeInMovement(now))
				movement.setNull();
			}
		}
	} // if (allowedToMove) 
}


float GearedStepperDrive::getCurrentAngle() {
	return currentMotorAngle / getGearReduction();
}


void GearedStepperDrive::setMeasuredAngle(float pMeasuredAngle) { 
	currentMotorAngle = pMeasuredAngle*getGearReduction();
	currentAngleAvailable = true;
}
