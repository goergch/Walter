
/*
 * ActuatorConfig.cpp
 *
 * Created: 07.06.2016 14:06:10
 *  Author: JochenAlt
 */ 


#include "Arduino.h"
#include "Config.h"
#include "BotMemory.h"

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
	
	// forearm (stepper/Encoder)
	memory.persMem.armConfig[actuatorNo].actuatorType = STEPPER_ENCODER_TYPE;   
	memory.persMem.armConfig[actuatorNo].id = FOREARM;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.id = FOREARM;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.id = FOREARM;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.minAngle= -90.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.nullAngle= 0.0;
	actuatorNo++;

	// upperarm (stepper/Encoder)
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

Stream* logger = new SoftwareSerial(PIN_D5,LOGGER_TX_PIN); // TX only, no receive D5 is not active

ActuatorSetupData actuatorSetup[MAX_ACTUATORS] {
	{ GRIPPER},
	{ HAND},
	{ WRIST},
	{ ELLBOW},
	{ FOREARM},
	{ UPPERARM},
	{ HIP} };

StepperSetupData stepperSetup[MAX_STEPPERS] {
	// Arm      direction Microsteps enable  dir     clock   angle gear                     maxspeed maxacc current[A]
	{ WRIST,    true,     8,         PIN_A1, PIN_A2, PIN_A3, 1.8,  /*(56.0/16.0),			  160 ,     800,*/   0.4, BLACK, RED, BLUE, GREEN},
	{ ELLBOW,   true,     4,         PIN_A4, PIN_A5, PIN_A6, 1.8,  /*(56.0/16.0)*(22.0/16.0), 160 ,     600,*/   0.4, BLACK, GREEN, RED, BLUE},
	{ FOREARM,  true,     16,        PIN_A7, PIN_C7, PIN_C6, 1.8,  /*(60.0/14.0)*(48.0/18.0), 200 ,     600,*/   1.4, NON_COLOR, NON_COLOR, NON_COLOR, NON_COLOR},
	{ UPPERARM, true,     16,        PIN_C5, PIN_C4, PIN_C3, 1.8,  /*(80.0/14.0)*(48.0/18.0), 160 ,     600,*/   3.5, BLACK, GREEN, RED, BLUE},
	{ HIP,      false,    16,        PIN_C2, PIN_D7, PIN_D6, 1.8,  /*(90.0/12.0),             160 ,     600,*/   2.8, BLACK, GREEN, RED, BLUE} };

RotaryEncoderSetupData encoderSetup[MAX_ENCODERS] {
	// 	ActuatorId/ programmI2CAddress / I2CAddreess / clockwise
	{ WRIST,    true,  AS5048_ADDRESS+0, false},
	{ ELLBOW,   false, AS5048_ADDRESS+0, true},
	{ FOREARM,  false, AS5048_ADDRESS+1, true},
	{ UPPERARM, false, AS5048_ADDRESS+2, true},
	{ HIP,		false, AS5048_ADDRESS+3, true}};

ServoSetupData servoSetup[MAX_SERVOS] {
	//    actuator  ID	                 reverse   minTorque maxTorque, setupSpeed (° /s )
	{ GRIPPER,	HERKULEX_MOTOR_ID-1, true,     65,       512,       30},
	{ HAND,		HERKULEX_MOTOR_ID,   false,    65,       512,       30}
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
	// logger->print(F(" gearReduction="));
	// logger->print(gearReduction,1);
	// logger->print(F(" rpm="));
	// logger->print(rpm,1);
	// logger->print(F(" accRpm="));
	//logger->print(accRpm,1);
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

void logActuator(ActuatorId id) {
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
	logger->print((char)('A'+port-1));

	for (int i = 0;i<8;i++) {
		if (_BV(i) == bit)
			logger->print(i);
	}
}

