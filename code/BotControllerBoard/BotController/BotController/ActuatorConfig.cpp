
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

void RotaryEncoderConfig::print() {
	Serial.print(F("EncoderConf("));
	Serial.print(id);
	Serial.print(F("){"));

	Serial.print(F("null="));
	Serial.print(nullAngle,1);
	Serial.println(F("}"));
}
void ServoConfig::print() {
	Serial.print(F("ServoConf("));
	Serial.print(id);
	Serial.print(F(") {"));
	Serial.print(F("null="));
	Serial.print(nullAngle,1);
	Serial.print(F("maxAngle="));
	Serial.print(maxAngle,1);
	Serial.print(F("minAngle="));
	Serial.print(minAngle,1);
	Serial.println(F("}"));
}


void StepperConfig::print() {
	Serial.print(F("StepperConf("));
	Serial.print(id);
	Serial.print(F(") {"));

	Serial.print(F("maxAngle="));
	Serial.print(maxAngle,1);
	Serial.print(F("minAngle="));
	Serial.print(minAngle,1);
	Serial.println(F("}"));
}


void ActuatorConfigurator::print() {
	Serial.print(F("ActuatorConf("));
	Serial.print(id);
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
	}
}

void ActuatorConfigurator::setDefaults() {

	Serial.println(F("reset configuration"));
	// Wrist Turn (herkulex Servo)
	memory.persMem.armConfig[0].actuatorType = SERVO_TYPE;
	memory.persMem.armConfig[0].id = HAND;
	memory.persMem.armConfig[0].config.servoArm.servo.id = 0;
	memory.persMem.armConfig[0].config.servoArm.servo.nullAngle = 0.0;
	memory.persMem.armConfig[0].config.servoArm.servo.minAngle= -120.0;
	memory.persMem.armConfig[0].config.servoArm.servo.maxAngle= +120.0;

	// Wrist Nick (stepper/Encoder)
	memory.persMem.armConfig[1].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[1].id = WRIST;
	memory.persMem.armConfig[1].config.stepperArm.encoder.id = 0;
	memory.persMem.armConfig[1].config.stepperArm.stepper.id = 0;
	memory.persMem.armConfig[1].config.stepperArm.stepper.minAngle= -100.0;
	memory.persMem.armConfig[1].config.stepperArm.stepper.maxAngle= +100.0;
	memory.persMem.armConfig[1].config.stepperArm.encoder.nullAngle= -286.0;

}