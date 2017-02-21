/* 
* GearedStepperDrive.h
*
* Created: 30.04.2016 22:19:39
* Author: JochenAlt
*/


#ifndef __MOTORDRIVERSTEPPERIMPL_H__
#define __MOTORDRIVERSTEPPERIMPL_H__

#include <MotorBase.h>
#include "Config.h"
#include "Space.h"
#include "AccelStepper.h"
#include "ActuatorProperty.h"
#include "TimePassedBy.h"
#include "RotaryEncoder.h"

class GearedStepperDrive : public MotorBase
{
public:
	GearedStepperDrive(): MotorBase() {
		currentDirection = true;
		currentAngleAvailable = false;
		configData = NULL;
		setupData = NULL;
		enabled = false;
	};
	
	void setup(StepperConfig* config, ActuatorConfiguration* pActuatorConfig, StepperSetupData* setupData, RotaryEncoder* encoder);
	void setAngle(float pAngle,uint32_t pAngleTargetDuration);
	void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration);
	void setCurrentAngle(float angle);

	void loop(uint32_t now);
	void loop();
	float getCurrentAngle();
	void setMeasuredAngle(float pMeasuredAngle, uint32_t now);
	StepperConfig& getConfig() { return *configData;}
	void direction(bool forward);
	void performStep();
	void enable();
	void disable();
	bool isEnabled();
	bool isDue(uint32_t now) { return timer.isDue_ms(configData->sampleRate, now); };
	void setDueTime(uint32_t due_ms) { timer.setDueTime(due_ms); };

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
		return configData->microSteps;
	}

	float getGearReduction() {
		return actuatorConfig->gearRatio;
	}

	uint16_t getMaxRpm() {
		return configData->maxSpeed;
	}

	float getMaxStepsPerSecond() {
		return configData->maxSpeed*(360/60)/getMotorDegreePerMicroStep();
	}

	uint16_t getMaxAcc() {
		return configData->maxAcc;
	}

	float getMicroStepsByAngle(float angle) {
		return angle * actuatorConfig->gearRatio * configData->microSteps*stepsPerDegree;
	}

	float getAnglePerMicroStep() {
		return getMotorDegreePerMicroStep()/getGearReduction();
	}

	float getMaxStepAccPerSecond() {
		return configData->maxAcc*(360/60)*getMotorMicroStepPerDegree();
	}

	float getMaxAccPerSample() {
		return  getMaxStepAccPerSecond()*1000.0/configData->sampleRate;
	}

	float getMotorDegreePerMicroStep() {
		return setupData->degreePerStep/configData->microSteps;
	}

	float getMotorMicroStepPerDegree() {
		return configData->microSteps*stepsPerDegree;
	}

	float getRPMByAnglePerSample(float anglePerSample) {
		return anglePerSample*sampleFrequency()*(60.0/360.0);
	}

	float sampleTime() {
		return float(configData->sampleRate)/1000.0;
	}

	float sampleFrequency() {
		return 1000.0/float(configData->sampleRate);
	}

	// set the Pibot Stepper Drivers enable PIN
	void enableDriver(bool on);


	StepperSetupData* setupData = NULL;
	ActuatorConfiguration* actuatorConfig = NULL;
	StepperConfig* configData = NULL;
	RotaryEncoder* encoder = NULL;
	AccelStepper accel;

	bool currentAngleAvailable = 0;		// true, if the encoder read an angle already
	bool currentDirection = false;		// set by setCurrentDirection
	float currentAngle;					// current actuator angle (not the motor angle!)
	bool enabled = false;				// set the setEnable
	float integral; 					// for PID controller
	float lastToBeAngle = 0;			// last to-be angle coming from to-be trajectory
	float anglePerMicroStep = 0;
	float frequency = 0;
	float stepsPerDegree = 0;
	TimePassedBy timer;
}; // GeardeStepperDriver

#endif //__MOTORDRIVERSTEPPERIMPL_H__
