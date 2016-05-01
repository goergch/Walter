/* 
* MotorDriverStepperImpl.h
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/


#ifndef __MOTORDRIVERSTEPPERIMPL_H__
#define __MOTORDRIVERSTEPPERIMPL_H__

#include "MotorDriver.h"
#include "Space.h"

class MotorDriverStepperImpl : public MotorDriver
{
public:
	MotorDriverStepperImpl(): MotorDriver() {
		currentAngle = 0;
		currentDirection = true;
	};
	
	void setup(int motorNumber);
	virtual void loop();
	virtual void moveToAngle(float angle, uint32_t pDuration_ms);
	virtual float getCurrentAngle();
	
	
private:
	uint16_t getPinDirection() {
		return StepperPort[myMotorNumber-1].directionPIN;
	}
	uint16_t getPinClock() {
		return StepperPort[myMotorNumber-1].clockPIN;
	}
	uint16_t getPinEnable() {
		return StepperPort[myMotorNumber-1].enablePIN;
	}

	float getDegreePerStep() {
		return StepperPort[myMotorNumber-1].degreePerStep;
	}

	bool getDirection() {
		return StepperPort[myMotorNumber-1].direction;
	}
	void direction(bool forward);
	void enable(bool on);
	void performStep();
	float currentAngle;
	bool currentDirection;
	bool enabled;

}; //MotorDriverStepperImpl

#endif //__MOTORDRIVERSTEPPERIMPL_H__
