
/*
 * ActuatorConfig.cpp
 *
 * Created: 07.06.2016 14:06:10
 *  Author: SuperJochenAlt
 */ 


#include "Arduino.h"
#include "ActuatorConfig.h"
#include "setup.h"
#include "BotMemory.h"

void ActuatorConfigurator::print() {
	Serial.print(F("ActuatorConfig("));
	Serial.print(getName_P(id));

	Serial.print(config.servoArm.servo.nullAngle,1);

	Serial.print(F("servo.null="));
	Serial.print(config.servoArm.servo.nullAngle,1);

	Serial.print(F("step.min="));
	Serial.print(config.stepperArm.stepper.minAngle,1);
	Serial.print(F("step.max="));
	Serial.print(config.stepperArm.stepper.maxAngle,1);

	Serial.print(F("enc.null="));
	Serial.print(config.stepperArm.encoder.nullAngle,1);

}

void ActuatorConfigurator::setDefaults() {
	// set default for all
	for (int i = 0;i<MAX_ACTUATORS;i++) {
		memory.persMem.armConfig[i].config.stepperArm.stepper.pivKp = 0.8;
		memory.persMem.armConfig[i].config.stepperArm.stepper.pivKi = 0.5;
		memory.persMem.armConfig[i].config.stepperArm.stepper.pivKd = 0.0;
	}
	
	// overwrite defauls where necessary
	// Wrist Turn (herkulex Servo)
	memory.persMem.armConfig[0].actuatorType = SERVO_TYPE;
	memory.persMem.armConfig[0].id = HAND;
	memory.persMem.armConfig[0].config.servoArm.servo.id = HAND;
	memory.persMem.armConfig[0].config.servoArm.servo.nullAngle = 0.0;
	memory.persMem.armConfig[0].config.servoArm.servo.minAngle= -120.0;
	memory.persMem.armConfig[0].config.servoArm.servo.maxAngle= +120.0;

	// Wrist Nick (stepper/Encoder)
	memory.persMem.armConfig[1].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[0].id = WRIST;
	memory.persMem.armConfig[0].config.stepperArm.encoder.id = WRIST;
	memory.persMem.armConfig[0].config.stepperArm.stepper.id = WRIST;
	memory.persMem.armConfig[1].config.stepperArm.stepper.minAngle= -100.0;
	memory.persMem.armConfig[1].config.stepperArm.stepper.maxAngle= +100.0;
	memory.persMem.armConfig[1].config.stepperArm.encoder.nullAngle= -286.0;

}