
/*
 * ActuatorConfig.cpp
 *
 * Created: 07.06.2016 14:06:10
 *  Author: JochenAlt
 */ 


#include "Arduino.h"
#include "Config.h"
#include "BotMemory.h"
#include "utilities.h"

Stream* logger = new SoftwareSerial(0,LOGGER_TX_PIN); // TX only, no receive D5 is not active

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
	memory.persMem.armConfig[GRIPPER].actuatorType = SERVO_TYPE;
	memory.persMem.armConfig[GRIPPER].id = GRIPPER;
	memory.persMem.armConfig[GRIPPER].config.servoArm.servo.id = GRIPPER;
	memory.persMem.armConfig[GRIPPER].config.servoArm.servo.nullAngle = 0;
	memory.persMem.armConfig[GRIPPER].config.servoArm.servo.minAngle= 0.0;
	memory.persMem.armConfig[GRIPPER].config.servoArm.servo.maxAngle= 75.0;
	
	// Hand (herkulex Servo)
	memory.persMem.armConfig[HAND].actuatorType = SERVO_TYPE;
	memory.persMem.armConfig[HAND].id = HAND;
	memory.persMem.armConfig[HAND].config.servoArm.servo.id = HAND;
	memory.persMem.armConfig[HAND].config.servoArm.servo.nullAngle = 0.0;
	memory.persMem.armConfig[HAND].config.servoArm.servo.minAngle= -120.0;
	memory.persMem.armConfig[HAND].config.servoArm.servo.maxAngle= +120.0;
	
	// Wrist (stepper/Encoder)
	memory.persMem.armConfig[WRIST].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[WRIST].id = WRIST;
	memory.persMem.armConfig[WRIST].config.stepperArm.encoder.id = WRIST;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.id = WRIST;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.minAngle= -100.0;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.maxAngle= +100.0;
	memory.persMem.armConfig[WRIST].config.stepperArm.encoder.nullAngle= -58;
	
	// ellbow (stepper/Encoder)
	memory.persMem.armConfig[ELLBOW].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[ELLBOW].id = ELLBOW;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.encoder.id = ELLBOW;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.id = ELLBOW;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.minAngle= -91.0;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.maxAngle= +91.0;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.encoder.nullAngle= -43;
	
	// forearm (stepper/Encoder)
	memory.persMem.armConfig[FOREARM].actuatorType = STEPPER_ENCODER_TYPE;   
	memory.persMem.armConfig[FOREARM].id = FOREARM;
	memory.persMem.armConfig[FOREARM].config.stepperArm.encoder.id = FOREARM;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.id = FOREARM;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.minAngle= -90.0;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[FOREARM].config.stepperArm.encoder.nullAngle= 103.5;

	// upperarm (stepper/Encoder)
	memory.persMem.armConfig[UPPERARM].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[UPPERARM].id = UPPERARM;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.encoder.id = UPPERARM;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.id = UPPERARM;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.minAngle= -90.0;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.encoder.nullAngle= -81;
	
	// Hip (stepper/Encoder)	
	memory.persMem.armConfig[HIP].actuatorType = STEPPER_ENCODER_TYPE;   
	memory.persMem.armConfig[HIP].id = HIP;
	memory.persMem.armConfig[HIP].config.stepperArm.encoder.id = HIP;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.id = HIP;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.minAngle= -90.0;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[HIP].config.stepperArm.encoder.nullAngle= 142;
}


ActuatorSetupData actuatorSetup[MAX_ACTUATORS] {
	{ GRIPPER},
	{ HAND},
	{ WRIST},
	{ ELLBOW},
	{ FOREARM},
	{ UPPERARM},
	{ HIP} };

StepperSetupData stepperSetup[MAX_STEPPERS] {
	// Arm      clockwise Microsteps enable  dir     clock   angle current[A]
	{ HIP,      true,     16,        PIN_C2, PIN_D7, PIN_D6, 1.8, 2.8, BLACK, GREEN, RED, BLUE},
	{ UPPERARM, false,    16,        PIN_C5, PIN_C4, PIN_C3, 1.8, 3.5, BLACK, GREEN, RED, BLUE},
	{ FOREARM,  true,     16,        PIN_A7, PIN_C7, PIN_C6, 1.8, 1.4, NON_COLOR, NON_COLOR, NON_COLOR, NON_COLOR},
	{ ELLBOW,   true,     16,        PIN_A4, PIN_A5, PIN_A6, 1.8, 0.7, BLACK, GREEN, RED, BLUE},
	{ WRIST,    true,     8,         PIN_A1, PIN_A2, PIN_A3, 1.8, 0.4, BLACK, GREEN, RED, BLUE}
};

RotaryEncoderSetupData encoderSetup[MAX_ENCODERS] {
	// 	ActuatorId	programmI2CAddress	I2CAddreess			clockwise
	{ HIP,			false,				AS5048_ADDRESS+0,	false},
	{ UPPERARM,		false,				AS5048_ADDRESS+3,	true},
	{ FOREARM,		false,				AS5048_ADDRESS+2,	true},
	{ ELLBOW,		false,				AS5048_ADDRESS+1,	false},		
	{ WRIST,		true,				AS5048_ADDRESS+0,	true}
};

ServoSetupData servoSetup[MAX_SERVOS] {
	// actuator Herkulex ID					reverse   minTorque maxTorque, setupSpeed (° /s )
	{ HAND,		HAND_HERKULEX_MOTOR_ID,		false,    65,       256,       30},
	{ GRIPPER,	GRIPPER_HERKULEX_MOTOR_ID,	true,     65,       256,       30}
};


void ActuatorSetupData::print() {
	logger->print(F("ActuatorSetup("));
	logActuator(id);
	logger->print(F(")"));
	logger->println(F("}"));
}

void ServoSetupData::print() {
	logger->print(F("ServoSetup("));
	logActuator(id);
	logger->print(F("){"));
				
	logger->print(F(" herkulexMotorId="));
	logger->print(herkulexMotorId,HEX);
	logger->println(F("}"));
}
		
void StepperSetupData::print() {
	logger->print(F("StepperSetup("));
	logActuator(id);
	logger->print(F("){"));
	logger->print(F(" pin(EN,DIR,CLK)=("));
	logPin(enablePIN);
	logger->print(",");
	logPin(directionPIN);
	logger->print(",");
	logPin(clockPIN);
	logger->print(")");
		
	logger->print(F(" direction="));
	logger->print(direction,1);
	logger->print(F(" microSteps="));
	logger->print(microSteps,1);
	logger->print(F(" degreePerStep="));
	logger->print(degreePerStep,1);
	logger->print(F(" amps="));
	logger->print(amps,1);
	logger->println(F("}"));
};

void RotaryEncoderSetupData::print() {
	logger->print(F("EncoderSetup("));
	logActuator(id);
	logger->print(F("){"));
				
	logger->print(F(" progI2CAddr="));
	logger->print(programmI2CAddress);

	logger->print(F(" I2CAddr=0x"));
	logger->print(I2CAddress,HEX);
	logger->print(F(" clockwise="));
	logger->print(clockwise);
	logger->println(F("}"));
};

void logActuator(ActuatorIdentifier id) {
	switch(id) {
		case GRIPPER:	logger->print(F("gripper"));break;
		case HAND:		logger->print(F("hand"));break;
		case WRIST:		logger->print(F("wrist"));break;
		case ELLBOW:	logger->print(F("ellbow"));break;
		case FOREARM:	logger->print(F("forearm"));break;
		case UPPERARM:	logger->print(F("upperarm"));break;
		case HIP:		logger->print(F("hip"));break;
	default:
		logger->print(id);
		logFatal(F("invalid actuator"));
		break;
	}
	logger->print(F("("));
	logger->print(id);
	logger->print(F(")"));
}


void logFatal(const __FlashStringHelper *ifsh) {
	logger->print(F("FATAL:"));
	logger->println(ifsh);
}

void logError(const __FlashStringHelper *ifsh) {
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

void logPin(uint8_t pin) {
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	if (port > 3) {
			logger->print(F("pin("));
			logger->print(port);
			logger->print(",");
			logger->print(bit);
			logger->print(F(")"));
	}
	else {
		logger->print((char)('A'+port-1));
		for (int i = 0;i<8;i++) {
			if (_BV(i) == bit)
				logger->print(i);
		}
	}
}
