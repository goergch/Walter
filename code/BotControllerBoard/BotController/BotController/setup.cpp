
/*
 * setup.cpp
 *
 * Created: 10.06.2016 15:29:54
 *  Author: JochenAlt
 */ 

#include "setup.h"

// SoftwareSerial logger(PIN_D5,LOGGER_TX_PIN); // TX
// Stream* logger = &Serial;
Stream* logger = new SoftwareSerial(PIN_D5,LOGGER_TX_PIN); // TX


ActuatorSetupData actuatorSetup[MAX_ACTUATORS] {
	{ GRIPPER},
	{ HAND},
	{ WRIST},
	{ FOREARM},
	{ UPPERARM},
	{ SHOULDER},
	{ HIP} };

StepperSetupData stepperSetup[MAX_STEPPERS] {
	// Arm      direction Microsteps enable  dir     clock   angle gear                    maxspeed maxacc current[A]
	{ WRIST,    true,     8,         PIN_A1, PIN_A2, PIN_A3, 1.8,  (56.0/16.0),			   160,     800,   0.4},
	{ FOREARM,  true,     4,         PIN_A4, PIN_A5, PIN_A6, 1.8,  (56.0/16.0)*(22.0/16.0),160,     600,   0.4},
	{ UPPERARM, true,     4,         PIN_A7, PIN_C7, PIN_C6, 1.8,  (60.0/14.0)*(48.0/18.0),200,     600,   1.4},
	{ SHOULDER, true,     4,         PIN_C5, PIN_C4, PIN_C3, 1.8,  (80.0/14.0)*(48.0/18.0),160,     600,   4.2},
	{ HIP,      true,     4,         PIN_C2, PIN_D7, PIN_D6, 1.8,  (90.0/12.0),            160,     600,   2.8} };


RotaryEncoderSetupData encoderSetup[MAX_ENCODERS] {
	// 	ActuatorId/ programmI2CAddress / I2CAddreess / clockwise 
	{ WRIST,    true,  AS5048_ADDRESS+0, false},
	{ FOREARM,   false, AS5048_ADDRESS+0, true},
	{ UPPERARM,  false, AS5048_ADDRESS+1, true},
	{ SHOULDER, false, AS5048_ADDRESS+2, true},
	{ HIP, false, AS5048_ADDRESS+2, true}};

ServoSetupData servoSetup[MAX_SERVOS] {
//    actuator  ID	                 reverse   minTorque maxTorque, setupSpeed (° /s )
	{ GRIPPER,	HERKULEX_MOTOR_ID-1, true,     65,       512,       30},
	{ HAND,		HERKULEX_MOTOR_ID,   false,    65,       512,       30}
};


void ActuatorSetupData::print() {
	logger->print(F("ActuatorSetup("));
	printActuator(id);
	logger->print(F(")"));
	logger->println(F("}"));
}

void ServoSetupData::print() {
	logger->print(F("ServoSetup("));
	printActuator(id);
	logger->print(F("){"));
	
	logger->print(F(" herkulexMotorId="));
	logger->print(herkulexMotorId,HEX);
	logger->println(F("}"));
}
	
void StepperSetupData::print() {
	logger->print(F("StepperSetup("));
	printActuator(id);
	logger->print(F("){"));
	
	logger->print(F(" direction="));
	logger->print(direction,1);

	logger->print(F(" microSteps="));
	logger->print(microSteps,1);
	logger->print(F(" degreePerStep="));
	logger->print(degreePerStep,1);
	logger->print(F(" gearReduction="));
	logger->print(gearReduction,1);
	logger->print(F(" rpm="));
	logger->print(rpm,1);
	logger->print(F(" accRpm="));
	logger->print(accRpm,1);
	logger->print(F(" amps="));
	logger->print(amps,1);

	logger->println(F("}"));
};

void RotaryEncoderSetupData::print() {
	logger->print(F("EncoderSetup("));
	printActuator(id);
	logger->print(F("){"));
	
	logger->print(F(" programmI2CAddress="));
	logger->print(programmI2CAddress,HEX);

	logger->print(F(" I2CAddress="));
	logger->print(I2CAddress,HEX);
	logger->print(F(" clockwise="));
	logger->print(clockwise);
	logger->println(F("}"));
};

void printActuator(ActuatorId id) {
	switch(id) {
		case GRIPPER:	logger->print(F("gripper"));break;
		case HAND:		logger->print(F("hand"));break;
		case WRIST:		logger->print(F("wrist"));break;
		case FOREARM:	logger->print(F("forearm"));break;
		case UPPERARM:	logger->print(F("upperarm"));break;
		case SHOULDER:	logger->print(F("shoulder"));break;
		case HIP:		logger->print(F("hip"));break;
		default:
			logger->print(id);
			fatalError(F("invalid actuator"));
		break;
	}
	logger->print(F("("));
	logger->print(id);
	logger->print(F(")"));
}


void fatalError(const __FlashStringHelper *ifsh) {
	logger->print(F("ERROR:"));
	logger->println(ifsh);
}

bool scanI2CAddress(uint8_t address, byte &error)
{
	// The i2c_scanner uses the return value of
	// the Write.endTransmisstion to see if
	// a device did acknowledge to the address.

	Wire.beginTransmission(address);
	error = Wire.endTransmission();
	return error == 0;
}

bool logSetup = true;
bool logServo = false;
bool logStepper = false;
bool logEncoder = false;
