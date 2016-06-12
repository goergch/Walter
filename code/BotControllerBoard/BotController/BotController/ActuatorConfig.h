
/*
 * ActuatorConfig.h
 *
 * Created: 07.06.2016 14:05:35
 *  Author: JochenAlt
 */ 




#ifndef __ACTUATOR_CONFIG_H_
#define __ACTUATOR_CONFIG_H_

#include "setup.h"
struct RotaryEncoderConfig {
	ActuatorId  id;
	uint8_t setupid;
	float  nullAngle;
	
	void print();
};

struct ServoConfig {
	ActuatorId  id;
	uint8_t setupid;

	float nullAngle;
	float  maxAngle;			// [°]
	float  minAngle;			// [°]
	
	void print();
};

struct StepperConfig {
	ActuatorId id;
	uint8_t setupid;

	float  maxAngle;			// [°]
	float  minAngle;			// [°]
	uint16_t minTicksPerStep;
	uint8_t allowedToMoveTickCounter;
	uint16_t maxStepRatePerSecond;
	float degreePerMicroStep;
	void print();
};


enum ActuatorType { SERVO_TYPE, STEPPER_ENCODER_TYPE, NO_ACTUATOR};
class ActuatorConfigurator {
	public:
	static void setDefaults();
	void print();

	ActuatorType actuatorType;
	ActuatorId id;
	union ActuatorConfig {
		struct {
			ServoConfig servo;
		} servoArm;
		struct {
			RotaryEncoderConfig  encoder;
			StepperConfig stepper;	
		} stepperArm;
	} config;
};

#endif