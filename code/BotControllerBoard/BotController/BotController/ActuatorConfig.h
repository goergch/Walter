
/*
 * ActuatorConfig.h
 *
 * Created: 07.06.2016 14:05:35
 *  Author: JochenAlt
 */ 




#ifndef __ACTUATOR_CONFIG_H_
#define __ACTUATOR_CONFIG_H_

struct RotaryEncoderConfig {
	uint8_t id;
	float  nullAngle;
};


struct StepperConfig {
	uint8_t id;
	float pivKp;
	float pivKi;
	float pivKd;
	
	float  maxAngle;			// [°]
	float  minAngle;			// [°]
};

struct ServoConfig {
	uint8_t id;
	float nullAngle;
	float  maxAngle;			// [°]
	float  minAngle;			// [°]
};

enum ActuatorType { SERVO_TYPE, STEPPER_ENCODER_TYPE};
class ActuatorConfigurator {
	public:
	static void setDefaults();
	void print();

	ActuatorType actuatorType;
	uint8_t id;
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