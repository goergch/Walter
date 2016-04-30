/* 
* MotorDriverStepperImpl.cpp
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/

#include "Arduino.h"
#include "MotorDriverStepperImpl.h"



void MotorDriverStepperImpl::setup(int motorNumber) {

	if ((motorNumber < 1) || (motorNumber>MAX_MOTORS)) {
		Serial.print(F("motor "));
		Serial.print(motorNumber);
		Serial.println(F(" is not a stepper."));
		delay(100);
		exit(0);
	};
	MotorDriver::setup(motorNumber);

	Serial.println("motor init");
	pinMode(getPinClock(), OUTPUT);
	pinMode(getPinDirection(), OUTPUT);
	pinMode(getPinEnable(), OUTPUT);
	enabled = false;
	enable(false);	
	
	// set to default direction
	direction(currentDirection);
	
	// no movement currently
	movement.setNull();
}

void MotorDriverStepperImpl::performStep() {
	digitalWrite(getPinClock(), LOW);  // This LOW to HIGH change is what creates the
	digitalWrite(getPinClock(), HIGH);
	if (currentDirection) {
		currentAngle += getDegreePerStep();
		Serial.print("+");
	}
	else {
		currentAngle -= getDegreePerStep();
		Serial.print("-");
	}
}

void MotorDriverStepperImpl::direction(bool forward) {
	bool toBeDirection = (forward==getDirection());
	if (toBeDirection != currentDirection) {
		Serial.print("~");
		digitalWrite(getPinDirection(), toBeDirection?LOW:HIGH);
		currentDirection = toBeDirection;
	}
}

void MotorDriverStepperImpl::enable(bool ok) {
	if (enabled != ok) {
		if (ok)
			Serial.print("1");
		else
			Serial.print("0");

		digitalWrite(getPinEnable(), ok?HIGH:LOW);  // This LOW to HIGH change is what creates the
		enabled = ok;
	}
}
void MotorDriverStepperImpl::loop() {
	if (!movement.isNull()) {
		uint32_t now = millis();
		if (movement.timeInMovement(now)) {
			enable(true);
			// interpolate tobe angle
			float toBeAngle = movement.getCurrentAngle(now);

		
			// carry out one step per loop (therefore max speed is 1000/MOTOR_SAMPLE_RATE*degreePerStep = 36° /s 
			if (abs(toBeAngle-currentAngle) > getDegreePerStep()) {
				// select direction
				direction(toBeAngle>currentAngle);

				performStep();	
			}
		} else {
			movement.setNull();
		}
	}
	else {
		enable(false);
	}
}
void MotorDriverStepperImpl::moveToAngle(float angle, uint32_t pDuration_ms) {
	movement.set(currentAngle, angle, millis(), pDuration_ms);
	enable(true);
}
float MotorDriverStepperImpl::getCurrentAngle() {
	return currentAngle;
}