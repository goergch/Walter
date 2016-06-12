
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
	Serial.print(F("EncoderConf("));
	printActuator(id);
	Serial.print(F("){"));

	Serial.print(F("null="));
	Serial.print(nullAngle,1);
	Serial.println(F("}"));
}
void ServoConfig::print() {
	Serial.print(F("ServoConf("));
	printActuator(id);
	Serial.print(F(") {"));
	Serial.print(F("null="));
	Serial.print(nullAngle,1);
	Serial.print(F(" maxAngle="));
	Serial.print(maxAngle,1);
	Serial.print(F(" minAngle="));
	Serial.print(minAngle,1);
	Serial.println(F("}"));
}


void StepperConfig::print() {
	Serial.print(F("StepperConf("));
	printActuator(id);
	Serial.print(F(") {"));

	Serial.print(F(" maxAngle="));
	Serial.print(maxAngle,1);
	Serial.print(F(" minAngle="));
	Serial.print(minAngle,1);

	Serial.print(F(" degreePerSteps="));
	Serial.print(degreePerMicroStep);
	Serial.print(F(" minTicksPerStep="));
	Serial.print(minTicksPerStep);
	Serial.print(F(" maxStepRate="));
	Serial.print(maxStepRatePerSecond);

	Serial.println(F("}"));
}


void ActuatorConfigurator::print() {
	Serial.print(F("ActuatorConf("));
	printActuator(id);
	Serial.print(F(")"));
	Serial.println();	
	switch (actuatorType) {
		case SERVO_TYPE:
			Serial.print(F("   "));
			config.servoArm.servo.print();
			break;
		case STEPPER_ENCODER_TYPE:
			Serial.print(F("   "));
			config.stepperArm.stepper.print();
			Serial.print(F("   "));
			config.stepperArm.encoder.print();
			break;
		case NO_ACTUATOR:
			Serial.print(F("none."));
			break;
	}
}

void ActuatorConfigurator::setDefaults() {
	// Wrist Turn (herkulex Servo)
	memory.persMem.armConfig[0].actuatorType = SERVO_TYPE;
	memory.persMem.armConfig[0].id = HAND;
	memory.persMem.armConfig[0].config.servoArm.servo.id = HAND;
	memory.persMem.armConfig[0].config.servoArm.servo.nullAngle = 0.0;
	memory.persMem.armConfig[0].config.servoArm.servo.minAngle= -120.0;
	memory.persMem.armConfig[0].config.servoArm.servo.maxAngle= +120.0;
	memory.persMem.armConfig[0].config.servoArm.servo.setupid = 0;

	// Wrist Nick (stepper/Encoder)
	memory.persMem.armConfig[1].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[1].id = WRIST;
	memory.persMem.armConfig[1].config.stepperArm.encoder.id = WRIST;
	memory.persMem.armConfig[1].config.stepperArm.stepper.id = WRIST;
	memory.persMem.armConfig[0].config.stepperArm.encoder.setupid = 0;
	memory.persMem.armConfig[0].config.stepperArm.stepper.setupid= 0;

	memory.persMem.armConfig[1].config.stepperArm.stepper.minAngle= -110.0;
	memory.persMem.armConfig[1].config.stepperArm.stepper.maxAngle= +110.0;
	memory.persMem.armConfig[1].config.stepperArm.encoder.nullAngle= 73.7;
	memory.persMem.armConfig[0].config.stepperArm.encoder.setupid = 1;
	memory.persMem.armConfig[0].config.stepperArm.stepper.setupid= 1;
	
	// Wrist Turn (stepper/Encoder)
	memory.persMem.armConfig[2].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[2].id = ELLBOW;
	memory.persMem.armConfig[2].config.stepperArm.encoder.id = ELLBOW;
	memory.persMem.armConfig[2].config.stepperArm.stepper.id = ELLBOW;
	memory.persMem.armConfig[2].config.stepperArm.stepper.minAngle= -100.0;
	memory.persMem.armConfig[2].config.stepperArm.stepper.maxAngle= +100.0;
	memory.persMem.armConfig[2].config.stepperArm.encoder.nullAngle= 0.0;
	
	memory.persMem.armConfig[3].actuatorType = NO_ACTUATOR;
	memory.persMem.armConfig[4].actuatorType = NO_ACTUATOR;
	memory.persMem.armConfig[5].actuatorType = NO_ACTUATOR;
}