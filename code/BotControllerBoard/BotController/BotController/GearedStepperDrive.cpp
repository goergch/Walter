/* 
* MotorDriverStepperImpl.cpp
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/

#include "Arduino.h"
#include "GearedStepperDrive.h"
#include "digitalWriteFast.h"
#include "setup.h"


void GearedStepperDrive::setup(	StepperConfig* pConfigData, StepperSetupData* pSetupData) {

	movement.setNull();
	targetTicksPerStep = 0;
	currentTicksPerSteps = 0;
	lastTicksPerStep = 0;
	
	configData = pConfigData;
	setupData = pSetupData;
#ifdef DEBUG_SETUP
	Serial.println(F("setup stepper"));
	Serial.print(F("   "));
	pConfigData->print();
	Serial.print(F("   "));
	pSetupData->print();
#endif
	pinMode(getPinClock(), OUTPUT);
	pinMode(getPinDirection(), OUTPUT);
	pinMode(getPinEnable(), OUTPUT);
	enable(true);	
	
	// set to default direction
	direction(true,currentDirection);
	
	// no movement currently
	movement.setNull();

	configData->degreePerActualSteps = getDegreePerStep()/getMicroSteps();
	
	configData->maxStepRatePerSecond  = (360.0/configData->degreePerActualSteps) *(float(getMaxRpm())/60.0);
	
	
	// define max speed in terms of ticks per step	
	configData->minTicksPerStep = (1000000L/STEPPER_SAMPLE_RATE_US)/getMaxStepRatePerSecond();
	if (configData->minTicksPerStep<1)
		configData->minTicksPerStep = 1;

	currentMotorAngle = 0.0;
	// setMeasuredAngle(0.0);
}


void GearedStepperDrive::changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) {
	// this methods works even when no current Angle has been measured
	uint32_t now = millis();
	movement.set(getCurrentAngle(), getCurrentAngle()+pAngleChange, now, pAngleTargetDuration);
	computeTickLength(now);

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
			printActuator(configData->id);
			Serial.print("](");
			Serial.print(pAngle);
			Serial.print(" is=");
			Serial.print(getCurrentAngle());
			Serial.print(" now=");
			Serial.print(now);
			Serial.print(" duration=");
			Serial.print(pAngleTargetDuration);
			Serial.println(")");
#endif
			lastAngle = pAngle;
			// set actuator angle (not motor angle)
			movement.set(getCurrentAngle(), pAngle, now, pAngleTargetDuration);
			computeTickLength(now);
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
		currentMotorAngle += configData->degreePerActualSteps;
	}
	else {
		currentMotorAngle -= configData->degreePerActualSteps;
	}
}

void GearedStepperDrive::setStepperDirection(bool forward) {
	bool dir = forward?LOW:HIGH;
	uint8_t pin = getPinDirection();
#ifdef USE_FAST_DIGITAL_WRITE
	digitalWriteFast(pin, dir);
#else
	digitalWrite(pin, dir);
#endif
}

void GearedStepperDrive::direction(bool dontCache,bool forward) {
	bool toBeDirection = forward;

	if ((toBeDirection != currentDirection) || dontCache)
	{
		if (!getDirection())
			forward=!forward;
		setStepperDirection(forward);
		currentDirection = toBeDirection;
	}
}

void GearedStepperDrive::enable(bool ok) {
	digitalWrite(getPinEnable(), ok?HIGH:LOW);  // This LOW to HIGH change is what creates the
}

void GearedStepperDrive::computeTickLength(uint32_t now) {
	if (movement.timeInMovement(now)) {
		lastTicksPerStep = currentTicksPerSteps;

		// how many ticks do we have until the end of the movement?
		uint32_t msToGo =(movement.endTime-now);
		uint32_t ticksToGo = (msToGo*1000L)/STEPPER_SAMPLE_RATE_US;

		// whats the angle to move
		float angleDiff = movement.angleEnd*getGearReduction()-currentMotorAngle;

		// how many steps are that?
		uint32_t stepsToMove = float(abs(angleDiff)/configData->degreePerActualSteps);

		// how many ticks per step?
		uint32_t ticksPerStep = ticksToGo/stepsToMove;

		// how many ticks on top of min ticks?
		targetTicksPerStep = ticksPerStep - configData->minTicksPerStep;
		if (targetTicksPerStep < 0)
			targetTicksPerStep = 0;
			
		Serial.print("#");
		Serial.print(targetTicksPerStep);
	} 
}

 
// called very often to execute one stepper step. Dont do complex operations here.
void GearedStepperDrive::loop(uint32_t now) {	
	// this method is called every SERVO_SAMPLE_RATE us.

	// Depending on the current speed of the stepper, we count the number of ticks we can wait until to do carry out one step
	allowedToMoveTickCounter++;
	bool allowedToMove = allowedToMoveTickCounter >= (configData->minTicksPerStep+getCurrentTicksPerStep()); // true, if we can execute one step		
	if (allowedToMove) {
		if (!movement.timeInMovement(now)) {
			setDefaultTicksPerStep();
		}
		if (!movement.isNull()) {	
			 {
				// amend the number of ticks per step coming out of getCurrentTicksPerStep( accelerate or break)
				adaptTicksPerStep();
					
				// compare 	position with to be position for last minute changes
				float toBeMotorAngle = movement.getCurrentAngle(now)*getGearReduction();
		
				// apply speed profile in order to not rapidly accelerate by accident. More a safety feature, since
				// trajectory profile is not done here
				float diffAngle = toBeMotorAngle-currentMotorAngle;
				/*

				if (abs(diffAngle) > configData->degreePerActualSteps*1.0) {
					if (diffAngle > 0)
						decTicksPerStep();
					else
						incTicksPerStep();
				}
				*/
				
				if ((abs(diffAngle) > configData->degreePerActualSteps*0.5)) 
				{ // is there enough movement for one step?
					// select direction
					direction(false,toBeMotorAngle>currentMotorAngle);
				
					// one step
					performStep();	
						
					// reset counter that ensures that max speed is not exceeded
					allowedToMoveTickCounter = 0;
				}
				
				
			}
		} // if !movement.isNull()
	} // if (allowedToMove) 
}

float GearedStepperDrive::getToBeAngle() {
	return movement.getCurrentAngle(millis());
}


float GearedStepperDrive::getCurrentAngle() {
	return currentMotorAngle / getGearReduction();
}

void GearedStepperDrive::setCurrentAngle(float angle) {
	currentMotorAngle = angle*getGearReduction();
}

void GearedStepperDrive::setMeasuredAngle(float pMeasuredAngle) { 
	currentMotorAngle = pMeasuredAngle*getGearReduction();
	currentAngleAvailable = true;
}
