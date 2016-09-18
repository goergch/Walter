/* 
* GearedStepperDrive.h
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/


#ifndef __MOTORDRIVERSTEPPERIMPL_H__
#define __MOTORDRIVERSTEPPERIMPL_H__

#include "Config.h"
#include "Space.h"
#include "DriveBase.h"
#include "AccelStepper.h"
#include "ActuatorProperty.h"

	
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
	
	void setup(StepperConfig* config, ActuatorConfigType* pActuatorConfig, StepperSetupData* setupData);
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

	float getGearReduction() {
		return actuatorConfig->gearRatio;
	}

	uint16_t getMaxRpm() {
		return actuatorConfig->maxSpeed;
	}

	uint16_t getMaxAcc() {
		return actuatorConfig->maxAcc;
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
	ActuatorConfigType* actuatorConfig;

	StepperConfig* configData;
	AccelStepper accel;
	bool enabled;
}; // GeardeStepperDriver

#endif //__MOTORDRIVERSTEPPERIMPL_H__
