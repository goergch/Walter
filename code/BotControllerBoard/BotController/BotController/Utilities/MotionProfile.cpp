#include "MotionProfile.h"

void MotionProfile::setup(float aVelocityMax, float aAccelerationMax, MotionProfiletype aMethod)
{
	maxVelocity = aVelocityMax;
	maxAcceleration = aAccelerationMax;
	method = aMethod;

	// Time variables
	lastTime = 0;
	delta = 0;
	
	// State variables
	reset();
}


float MotionProfile::update(float setpoint) {
	uint32_t now = millis();
	
	bool firstTime = true;
	// Get current time in microseconds;
	if (lastTime == 0) {
		lastTime = now;
		firstTime= true;
	}
		
	// Delta=dT [s]
	delta = float(now - lastTime) / 1000.0;
	lastTime = now;

	// Shift state variables
	oldPosition = position;
	oldVelocity = velocity;
			
	if (!firstTime) {
		// Check what type of motion profile to use
		switch(method) {
			case LINEAR_PROFILE:
				// Constant velocity profile
				calculateConstantVelocityProfile(setpoint);
				break;
			case TRAPEZOIDAL_PROFILE:
				// Trapezoidal velocity profile
				calculateTrapezoidalProfile(setpoint);
				break;
		}

		// Calculate velocity
		velocity = (position - oldPosition) / delta;
		if (velocity != velocity) {
			velocity = 0;
		}
	}
	return position;
}


void MotionProfile::calculateConstantVelocityProfile(float setpoint) {
	float suggestedVelocity = (setpoint - position) / delta;
	
	if (suggestedVelocity > maxVelocity) {
		position += maxVelocity * delta;
	}
	else if (suggestedVelocity < -maxVelocity) {
		position += -maxVelocity * delta;
	}
	else {
		position += suggestedVelocity * delta;
	}
}

void MotionProfile::calculateTrapezoidalProfile(float setpoint) {
	// Check if we need to de-accelerate
	if (((velocity * velocity) / maxAcceleration) / 2 >= abs(setpoint - position)) {
		if (velocity < 0) {
			position += (velocity + maxAcceleration * delta) * delta;
		}
		else if (velocity > 0) {
			position += (velocity - maxAcceleration * delta) * delta;
		}
	}
	else {
		// We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
		if (abs(velocity) < maxVelocity || 
			(setpoint < position && velocity > 0) || 
			(setpoint > position && velocity < 0)) {
			// We need to accelerate, do so but check the maximum acceleration.
			// Keep velocity constant at the maximum
			float suggestedVelocity = 0.0;
			if (setpoint > position) {
				suggestedVelocity = velocity + maxAcceleration * delta;
				if (suggestedVelocity > maxVelocity) {
					suggestedVelocity = maxVelocity;
				}
			}
			else {
				suggestedVelocity = velocity - maxAcceleration * delta;
				if (suggestedVelocity < -maxVelocity) {
					suggestedVelocity = -maxVelocity;
				}
			}
			position += suggestedVelocity * delta;
		}
		else {
			// Keep velocity constant at the maximum
			if (setpoint > position) {
				position += maxVelocity * delta;
			}
			else {
				position += -maxVelocity * delta;
			}
		}
	}
}

void MotionProfile::pause() {
	lastTime = 0;
}


void MotionProfile::reset() {
	// Reset all state variables
	position = 0;
	oldPosition = 0;
	velocity = 0;
	oldVelocity = 0;
}