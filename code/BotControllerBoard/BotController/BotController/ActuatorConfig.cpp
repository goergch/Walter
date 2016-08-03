
/*
 * ActuatorConfig.cpp
 *
 * Created: 07.06.2016 14:06:10
 *  Author: JochenAlt
 */ 


#include "Arduino.h"
#include "ActuatorConfig.h"
#include "setup.h"
#include "BotMemory.h"
#include "setup.h"
void RotaryEncoderConfig::print() {
	logger->print(F("EncoderConf("));
	logActuator(id);
	logger->print(F("){"));

	logger->print(F("null="));
	logger->print(nullAngle,1);
	logger->println(F("}"));
}

void ServoConfig::print() {
	logger->print(F("ServoConf("));
	logActuator(id);
	logger->print(F(") {"));

	logger->print(F(") {"));
	logger->print(F("null="));
	logger->print(nullAngle,1);
	logger->print(F(" maxAngle="));
	logger->print(maxAngle,1);
	logger->print(F(" minAngle="));
	logger->print(minAngle,1);
	logger->println(F("}"));
}


void StepperConfig::print() {
	logger->print(F("StepperConf("));
	logActuator(id);
	logger->print(F(") {"));

	logger->print(F(" maxAngle="));
	logger->print(maxAngle,1);
	logger->print(F(" minAngle="));
	logger->print(minAngle,1);

	logger->print(F(" degreePerSteps="));
	logger->print(degreePerMicroStep);
	logger->print(F(" maxStepRate="));
	logger->print(maxStepRatePerSecond);

	logger->println(F("}"));
}


void ActuatorConfig::print() {
	logger->print(F("ActuatorConf("));
	logActuator(id);
	logger->print(F(")"));
	logger->println();	
	switch (actuatorType) {
		case SERVO_TYPE:
			logger->print(F("   "));
			config.servoArm.servo.print();
			break;
		case STEPPER_ENCODER_TYPE:
			logger->print(F("   "));
			config.stepperArm.stepper.print();
			logger->print(F("   "));
			config.stepperArm.encoder.print();
			break;
		case NO_ACTUATOR:
			logger->print(F("none."));
			break;
	}
}

void ActuatorConfig::setDefaults() {
	// Gripper (herkulex Servo)
	uint8_t actuatorNo = 0;
	memory.persMem.armConfig[actuatorNo].actuatorType = SERVO_TYPE;
	memory.persMem.armConfig[actuatorNo].id = GRIPPER;
	memory.persMem.armConfig[actuatorNo].config.servoArm.servo.id = GRIPPER;
	memory.persMem.armConfig[actuatorNo].config.servoArm.servo.nullAngle = 0.0;
	memory.persMem.armConfig[actuatorNo].config.servoArm.servo.minAngle= 0.0;
	memory.persMem.armConfig[actuatorNo].config.servoArm.servo.maxAngle= 75.0;
	actuatorNo++;
	
	// Hand (herkulex Servo)
	memory.persMem.armConfig[actuatorNo].actuatorType = SERVO_TYPE;
	memory.persMem.armConfig[actuatorNo].id = HAND;
	memory.persMem.armConfig[actuatorNo].config.servoArm.servo.id = HAND;
	memory.persMem.armConfig[actuatorNo].config.servoArm.servo.nullAngle = 0.0;
	memory.persMem.armConfig[actuatorNo].config.servoArm.servo.minAngle= -120.0;
	memory.persMem.armConfig[actuatorNo].config.servoArm.servo.maxAngle= +120.0;
	actuatorNo++;
	
	// Wrist (stepper/Encoder)
	memory.persMem.armConfig[actuatorNo].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[actuatorNo].id = WRIST;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.id = WRIST;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.id = WRIST;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.minAngle= -100.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.maxAngle= +100.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.nullAngle= 73.7;
	actuatorNo++;
	
	// Forearm (stepper/Encoder)
	memory.persMem.armConfig[actuatorNo].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[actuatorNo].id = ELLBOW;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.id = ELLBOW;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.id = ELLBOW;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.minAngle= -91.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.maxAngle= +92.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.nullAngle= 1.5;
	actuatorNo++;
	
	// Upperarm (stepper/Encoder)
	memory.persMem.armConfig[actuatorNo].actuatorType = STEPPER_ENCODER_TYPE;   
	memory.persMem.armConfig[actuatorNo].id = FOREARM;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.id = FOREARM;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.id = FOREARM;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.minAngle= -90.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.nullAngle= 0.0;
	actuatorNo++;

	// shoulder (stepper/Encoder)
	memory.persMem.armConfig[actuatorNo].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[actuatorNo].id = UPPERARM;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.id = UPPERARM;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.id = UPPERARM;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.minAngle= -90.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.nullAngle= 0.0;
	actuatorNo++;

	// Hip (stepper/Encoder)	
	memory.persMem.armConfig[actuatorNo].actuatorType = STEPPER_ENCODER_TYPE;   
	memory.persMem.armConfig[actuatorNo].id = HIP;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.id = HIP;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.id = HIP;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.minAngle= -90.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.nullAngle= 0;
	actuatorNo++;
}