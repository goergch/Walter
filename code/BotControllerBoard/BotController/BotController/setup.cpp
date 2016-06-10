
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
	{ WRIST,    false, 4, PIN_A2, PIN_A3, PIN_A4, 1.8, 56.0/16.0,160, 160},
	{ ELLBOW,   true,  1, PIN_A6, PIN_A7, PIN_C7, 1.8, 1.0,       60, 160},
	{ FOREARM,  true,  1, PIN_C6, PIN_C5, PIN_C4, 1.8, 1.0,       60, 160},
	{ UPPERARM, true,  1, PIN_C3, PIN_C2, PIN_D7, 1.8, 1.0,       60, 160},
	{ HIP,      true,  1, PIN_D6, PIN_D5, PIN_D4, 1.8, 1.0,       60, 160} };


RotaryEncoderSetupData encoderSetup[MAX_ENCODERS] {
	{ WRIST,    true,  AS5048_ADDRESS+0, true, true },
	{ ELLBOW,   false, AS5048_ADDRESS+0, true, false },
	{ FOREARM,  false, AS5048_ADDRESS+1, true, false },
	{ UPPERARM, false, AS5048_ADDRESS+2, true, false },
	{ HIP,      false, AS5048_ADDRESS+3, true, false }};


void ActuatorSetupData::print() {
	Serial.print(F("ActuatorSetup("));
	Serial.print(id);
	Serial.print(F("){"));
	Serial.println(F("}"));
}

void ServoSetupData::print() {
	Serial.print(F("ServoSetup("));
	Serial.print(id);
	Serial.print(F("){"));
	
	Serial.print(F("herkulexMotorId="));
	Serial.print(herkulexMotorId,1);
	Serial.println(F("}"));
}
	
void StepperSetupData::print() {
	Serial.print(F("StepperSetup("));
	Serial.print(id);
	Serial.print(F("){"));
	
	Serial.print(F("direction="));
	Serial.print(direction,1);

	Serial.print(F("microSteps="));
	Serial.print(microSteps,1);
	Serial.print(F("microSteps="));
	Serial.print(microSteps,1);
	Serial.print(F("degreePerStep="));
	Serial.print(degreePerStep,1);
	Serial.print(F("gearReduction="));
	Serial.print(gearReduction,1);
	Serial.print(F("rpm="));
	Serial.print(rpm,1);
	Serial.print(F("accRpm="));
	Serial.print(accRpm,1);
	Serial.println(F("}"));
};

void RotaryEncoderSetupData::print() {
	Serial.print(F("EncoderSetup("));
	Serial.print(id);
	Serial.print(F("){"));
	
	Serial.print(F("programmI2CAddress="));
	Serial.print(programmI2CAddress,1);

	Serial.print(F("I2CAddress="));
	Serial.print(I2CAddress,1);
	Serial.print(F("clockwise="));
	Serial.print(clockwise,1);
	Serial.print(F("reverse="));
	Serial.print(reverse,1);
	Serial.println(F("}"));
};

const __FlashStringHelper* getName_P(ActuatorId id) {
	switch(id) {
		case HAND: return F("hand");
		case WRIST: return F("wrist");
		case ELLBOW: return F("ellbow");
		case FOREARM: return F("forearm");
		case UPPERARM: return F("upperarm");
		case HIP: return F("hip");
		break;
	}
	return F("");
}
