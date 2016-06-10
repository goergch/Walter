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
#include "MotionProfile.h"
#include "DriveBase.h"
#include "setup.h"

class GearedStepperDrive : public DriveBase
{
public:
	GearedStepperDrive(): DriveBase() {
		currentMotorAngle = 0;
		currentDirection = true;
		allowedToMoveTickCounter = 0;
		minTicksPerStep = 0;
		currentAngleAvailable = false;
		configData = NULL;
		setupData = NULL;
	};
	
	void setup(StepperConfig& config, StepperSetupData& setupData);
	void setAngle(float pAngle,uint32_t pAngleTargetDuration);
	void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration);

	void loop(uint32_t now);
	float getCurrentAngle();
	void setMeasuredAngle(float pMeasuredAngle);
	void printConfiguration();	
private:

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
		return maxStepRatePerSecond;
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
		return minTicksPerStep;
	}

	bool getDirection() {
		return setupData->direction;
	}
	void direction(bool forward);
	void enable(bool on);
	void performStep();
	
	bool currentAngleAvailable;
	bool currentDirection;
	
	uint16_t minTicksPerStep;
	uint8_t allowedToMoveTickCounter;
	
	uint16_t maxStepRatePerSecond; 
	int16_t stepsSinceSpeedMeasurement; 

	float degreePerActualSteps;
	float currentMotorAngle;
	
	StepperSetupData* setupData;
	StepperConfig* configData;
	
	MotionProfile profile;
}; //MotorDriverStepperImpl

#endif //__MOTORDRIVERSTEPPERIMPL_H__
