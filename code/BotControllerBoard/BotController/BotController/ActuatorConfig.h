
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
	uint8_t id;
	float  nullAngle;
	
	void print();
};

struct ServoConfig {
	uint8_t id;
	float nullAngle;
	float  maxAngle;			// [°]
	float  minAngle;			// [°]
	
	void print();
};

struct StepperConfig {
	uint8_t id;
	float  maxAngle;			// [°]
	float  minAngle;			// [°]
	
	void print();
};


enum ActuatorType { SERVO_TYPE, STEPPER_ENCODER_TYPE};
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