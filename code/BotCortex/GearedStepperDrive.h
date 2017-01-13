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
#include "TimePassedBy.h"

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
	
	void setup(StepperConfig* config, ActuatorConfiguration* pActuatorConfig, StepperSetupData* setupData);
	void setAngle(float pAngle,uint32_t pAngleTargetDuration);
	void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration);
	void setCurrentAngle(float angle);

	void loop(uint32_t now);
	void loop();
	float getCurrentAngle();
	void setMeasuredAngle(float pMeasuredAngle, uint32_t now);
	StepperConfig& getConfig() { return *configData;}
	void direction(bool dontCache,bool forward);
	void performStep();
	void enable();
	void disable();
	bool isEnabled();
	bool isDue(uint32_t now) { return timer.isDue_ms(setupData->sampleRate, now); };
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
		return configData->maxSpeed;
	}



	uint16_t getMaxAcc() {
		return configData->maxAcc;
	}

	float getDegreePerActualSteps () {
		return configData->degreePerMicroStep;
	}
	
	bool getDirection() {
		return setupData->direction;
	}

	float getAnglePerStep() {
		return anglePerMicroStep;
	}
	void computeNewSpeed();

	void setStepperDirection(bool forward);
	void enableDriver(bool on);
	
	float anglePerMicroStep;
	bool currentAngleAvailable;
	bool currentDirection;
	float currentMotorAngle;
	float currentAngle;

	float maxStepsPerSample;
	
	StepperSetupData* setupData;
	ActuatorConfiguration* actuatorConfig;

	StepperConfig* configData = NULL;
	AccelStepper accel;
	bool enabled = false;

	long sampleRate = ENCODER_SAMPLE_RATE;
	float dT;
	float rezi_dT;

	float maxStepsPerSecond;
	float maxStepAccPerSecond;
	float maxAccPerSample;

	float currentSpeed = 0; 			// steps/s
	float currentFilteredSpeed = 0;		// steps/s
	float integral; 					// for PID controller
	float lastStepErrorPerSample = 0;	// for PID controller
	TimePassedBy timer;
}; // GeardeStepperDriver

#endif //__MOTORDRIVERSTEPPERIMPL_H__
