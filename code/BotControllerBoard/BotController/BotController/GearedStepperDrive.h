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
#include "AccelStepper.h"


	
class GearedStepperDrive : public DriveBase
{
public:
	GearedStepperDrive(): DriveBase() {
		currentMotorAngle = 0;
		currentDirection = true;
		currentAngleAvailable = false;
		configData = NULL;
		setupData = NULL;
		enabled = false;
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
	void direction(bool dontCache,bool forward);
	void performStep();
	void enable();
	void disable();
	bool isEnabled();

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

	float getDegreePerFullStep() {
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

	float getDegreePerActualSteps () {
		return configData->degreePerMicroStep;
	}
	
	bool getDirection() {
		return setupData->direction;
	}

	void setStepperDirection(bool forward);
	void enableDriver(bool on);
	
	bool currentAngleAvailable;
	bool currentDirection;
	
	float currentMotorAngle;
	
	StepperSetupData* setupData;
	StepperConfig* configData;
	AccelStepper accel;
	bool enabled;
}; //MotorDriverStepperImpl

#endif //__MOTORDRIVERSTEPPERIMPL_H__
