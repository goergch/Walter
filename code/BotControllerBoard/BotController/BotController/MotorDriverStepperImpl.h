/* 
* MotorDriverStepperImpl.h
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/


#ifndef __MOTORDRIVERSTEPPERIMPL_H__
#define __MOTORDRIVERSTEPPERIMPL_H__

#include "Actuator.h"
#include "Space.h"

class MotorDriverStepperImpl : public Actuator
{
public:
	MotorDriverStepperImpl(): Actuator() {
		currentAngle = 0;
		currentDirection = true;
		tickCounter = 0;
		minTicksPerStep = 0;
		currentAngleAvailable = false;
	};
	
	void setup(int motorNumber);
	virtual void setAngle(float pAngle,uint32_t pAngleTargetDuration);
	virtual void loop(uint32_t now);
	virtual void moveToAngle(float angle, uint32_t pDuration_ms);
	virtual float getCurrentAngle();
	virtual void setMeasuredAngle(float pMeasuredAngle);
	void printConfiguration();	
private:
	uint16_t getPinDirection() {
		return StepperConfig[myActuatorNumber-1].directionPIN;
	}
	uint16_t getPinClock() {
		return StepperConfig[myActuatorNumber-1].clockPIN;
	}
	uint16_t getPinEnable() {
		return StepperConfig[myActuatorNumber-1].enablePIN;
	}

	float getDegreePerStep() {
		return StepperConfig[myActuatorNumber-1].degreePerStep;
	}
	float getActualDegreePerStep() {
		return degreePerActualSteps;
	}

	uint8_t getMicroSteps() {
		return StepperConfig[myActuatorNumber-1].microSteps;
	}

	uint16_t getMaxStepRatePerSecond() {
		return maxStepRatePerSecond;
	}
	
	float getGearReduction() {
		return StepperConfig[myActuatorNumber-1].gearReduction;
	}

	uint16_t getMaxRpm() {
		return StepperConfig[myActuatorNumber-1].rpm;
	}

	uint16_t getMinTicksPerStep() {
		return minTicksPerStep;
	}

	bool getDirection() {
		return StepperConfig[myActuatorNumber-1].direction;
	}
	void direction(bool forward);
	void enable(bool on);
	void performStep();
	float currentAngle;
	
	bool currentAngleAvailable;
	bool currentDirection;
	bool enabled;
	
	uint16_t minTicksPerStep;
	uint16_t tickCounter;
	float degreePerActualSteps;
	uint16_t maxStepRatePerSecond; 
}; //MotorDriverStepperImpl

#endif //__MOTORDRIVERSTEPPERIMPL_H__
