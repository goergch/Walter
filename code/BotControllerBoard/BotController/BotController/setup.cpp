
/*
 * setup.cpp
 *
 * Created: 10.06.2016 15:29:54
 *  Author: JochenAlt
 */ 

#include "setup.h"

ActuatorSetupData actuatorSetup[MAX_ACTUATORS] {
	{ HAND },
	{ WRIST},
	{ ELLBOW},
	{ FOREARM},
	{ UPPERARM},
	{ HIP} };

StepperSetupData stepperSetup[MAX_STEPPERS] {
	{ WRIST,    false, 4, PIN_A2, PIN_A3, PIN_A4, 1.8, 56.0/16.0,			   160, 250},
	{ ELLBOW,   true,  1, PIN_A6, PIN_A7, PIN_C7, 1.8, (56.0/16.0)*(24.0/15.0), 60, 160},
	{ FOREARM,  true,  1, PIN_C6, PIN_C5, PIN_C4, 1.8, 1.0,       60, 160},
	{ UPPERARM, true,  1, PIN_C3, PIN_C2, PIN_D7, 1.8, 1.0,       60, 160},
	{ HIP,      true,  1, PIN_D6, PIN_D5, PIN_D4, 1.8, 1.0,       60, 160} };


RotaryEncoderSetupData encoderSetup[MAX_ENCODERS] {
	// 	ActuatorId/ programmI2CAddress / I2CAddreess / clockwise 
	{ WRIST,    true,  AS5048_ADDRESS+0, false},
	{ ELLBOW,   false, AS5048_ADDRESS+0, true},
	{ FOREARM,  false, AS5048_ADDRESS+1, true},
	{ UPPERARM, false, AS5048_ADDRESS+2, true},
	{ HIP,      false, AS5048_ADDRESS+3, true}};

ServoSetupData servoSetup[MAX_SERVOS] {
	{ HAND, HERKULEX_MOTOR_ID, }
};

void ActuatorSetupData::print() {
	Serial.print(F("ActuatorSetup("));
	printActuator(id);
	Serial.print(F(")"));
	Serial.println(F("}"));
}

void ServoSetupData::print() {
	Serial.print(F("ServoSetup("));
	printActuator(id);
	Serial.print(F("){"));
	
	Serial.print(F(" herkulexMotorId="));
	Serial.print(herkulexMotorId,HEX);
	Serial.println(F("}"));
}
	
void StepperSetupData::print() {
	Serial.print(F("StepperSetup("));
	printActuator(id);
	Serial.print(F("){"));
	
	Serial.print(F(" direction="));
	Serial.print(direction,1);

	Serial.print(F(" microSteps="));
	Serial.print(microSteps,1);
	Serial.print(F(" microSteps="));
	Serial.print(microSteps,1);
	Serial.print(F(" degreePerStep="));
	Serial.print(degreePerStep,1);
	Serial.print(F(" gearReduction="));
	Serial.print(gearReduction,1);
	Serial.print(F(" rpm="));
	Serial.print(rpm,1);
	Serial.print(F(" accRpm="));
	Serial.print(accRpm,1);
	Serial.println(F("}"));
};

void RotaryEncoderSetupData::print() {
	Serial.print(F("EncoderSetup("));
	printActuator(id);
	Serial.print(F("){"));
	
	Serial.print(F(" programmI2CAddress="));
	Serial.print(programmI2CAddress,HEX);

	Serial.print(F(" I2CAddress="));
	Serial.print(I2CAddress,HEX);
	Serial.print(F("clockwise="));
	Serial.print(clockwise);
	Serial.println(F("}"));
};

void printActuator(ActuatorId id) {
	switch(id) {
		case HAND: Serial.print(F("hand"));return;
		case WRIST: Serial.print(F("wrist"));return;
		case ELLBOW: Serial.print(F("ellbow"));return;
		case FOREARM: Serial.print(F("forearm"));return;
		case UPPERARM: Serial.print(F("upperarm"));return;
		case HIP: Serial.print(F("hip"));return;
		break;
	}
}
