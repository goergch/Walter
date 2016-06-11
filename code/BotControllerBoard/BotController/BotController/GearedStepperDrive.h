/* 
* MotorDriverStepperImpl.h
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/


#ifndef __MOTORDRIVERSTEPPERIMPL_H__
#define __MOTORDRIVERSTEPPERIMPL_H__

#include "ActuatorConfig.h"
#include "Space.h"
#include "DriveBase.h"
#include "setup.h"

class GearedStepperDrive : public DriveBase
{
public:
	GearedStepperDrive(): DriveBase() {
		currentMotorAngle = 0;
		currentDirection = true;
		allowedToMoveTickCounter = 0;
		currentAngleAvailable = false;
		configData = NULL;
		setupData = NULL;
		targetTicksPerStep = 0;
		lastTicksPerStep = 0;
		currentTicksPerSteps  = 0;
	};
	
	void setup(StepperConfig* config, StepperSetupData* setupData);
	void setAngle(float pAngle,uint32_t pAngleTargetDuration);
	float getToBeAngle();
	void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration);
	void setCurrentAngle(float angle);

	void loop(uint32_t now);
	float getCurrentAngle();
	void setMeasuredAngle(float pMeasuredAngle);
	StepperConfig& getConfig() { return *configData;}
private:
	void computeTickLength(uint32_t now);

	void incTicksPerStep () {
		currentTicksPerSteps++;
	}

	void decTicksPerStep () {
		if (currentTicksPerSteps>0)
			currentTicksPerSteps--;
	}

	int16_t getCurrentTicksPerStep() {
		return currentTicksPerSteps;
	}
	void adaptTicksPerStep() {
		currentTicksPerSteps = (targetTicksPerStep+currentTicksPerSteps)/2;
	}
	void setDefaultTicksPerStep() {
		lastTicksPerStep = currentTicksPerSteps;
		targetTicksPerStep = 0;
	}

	int16_t targetTicksPerStep;
	int16_t lastTicksPerStep ;
	int16_t currentTicksPerSteps;
	
	uint16_t getPinDirection() {
		return setupData->directionPIN;
	}
	uint16_t getPinClock() {
		return setupData->clockPIN;
	}
	uint16_t getPinEnable() {
		return setupData->enablePIN;
	}

	float getDegreePerStep() {
		return setupData->degreePerStep;
	}

	uint8_t getMicroSteps() {
		return setupData->microSteps;
	}

	uint16_t getMaxStepRatePerSecond() {
		return configData->maxStepRatePerSecond;
	}
	
	float getGearReduction() {
		return setupData->gearReduction;
	}

	uint16_t getMaxRpm() {
		return setupData->rpm;
	}

	uint16_t getMaxAcc() {
		return setupData->accRpm;
	}
	
	uint16_t getMinTicksPerStep() {
		return configData->minTicksPerStep;
	}

	float getDegreePerActualSteps () {
		return configData->degreePerActualSteps;
	}
	
	bool getDirection() {
		return setupData->direction;
	}
	void direction(bool dontCache,bool forward);
	void setStepperDirection(bool forward);

	void enable(bool on);
	void performStep();
	
	bool currentAngleAvailable;
	bool currentDirection;
	
	uint8_t allowedToMoveTickCounter;


	float currentMotorAngle;
	
	StepperSetupData* setupData;
	StepperConfig* configData;
	
}; //MotorDriverStepperImpl

#endif //__MOTORDRIVERSTEPPERIMPL_H__
