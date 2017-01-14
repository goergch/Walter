
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
#include "pins.h"


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
	memory.persMem.armConfig[HAND].config.servoArm.servo.nullAngle = 41;
	memory.persMem.armConfig[HAND].config.servoArm.servo.minAngle= -120.0;
	memory.persMem.armConfig[HAND].config.servoArm.servo.maxAngle= +120.0;
	
	// Wrist (stepper/Encoder)
	memory.persMem.armConfig[WRIST].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[WRIST].id = WRIST;
	memory.persMem.armConfig[WRIST].config.stepperArm.encoder.id = WRIST;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.id = WRIST;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.minAngle= -100.0;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.maxAcc= 3000;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.maxSpeed= 140;
	memory.persMem.armConfig[WRIST].config.stepperArm.encoder.nullAngle = -58.1;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.kP= 0.55;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.kD= 0.0;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.kI= 0.0;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.sampleRate= 20;
	memory.persMem.armConfig[WRIST].config.stepperArm.stepper.initialMicroSteps = 8;

	
	// ellbow (stepper/Encoder)
	memory.persMem.armConfig[ELLBOW].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[ELLBOW].id = ELLBOW;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.encoder.id = ELLBOW;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.id = ELLBOW;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.minAngle= -91.0;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.maxAngle= +91.0;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.encoder.nullAngle= 27.0-5.5;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.maxAcc= 10000;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.maxSpeed= 270;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.kP= 0.50;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.kD= 0.0;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.kI= 0.0;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.sampleRate= 20;
	memory.persMem.armConfig[ELLBOW].config.stepperArm.stepper.initialMicroSteps = 4;
	
	// forearm (stepper/Encoder)
	memory.persMem.armConfig[FOREARM].actuatorType = STEPPER_ENCODER_TYPE;   
	memory.persMem.armConfig[FOREARM].id = FOREARM;
	memory.persMem.armConfig[FOREARM].config.stepperArm.encoder.id = FOREARM;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.id = FOREARM;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.minAngle= -180.0;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.maxAngle= +30.0;
	memory.persMem.armConfig[FOREARM].config.stepperArm.encoder.nullAngle= 209.7-1.6;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.maxAcc= 1000;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.maxSpeed= 150;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.kP= 0.3;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.kD= 0.000;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.kI= 0.0;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.sampleRate= 30;
	memory.persMem.armConfig[FOREARM].config.stepperArm.stepper.initialMicroSteps = 8;


	// upperarm (stepper/Encoder)
	memory.persMem.armConfig[UPPERARM].actuatorType = STEPPER_ENCODER_TYPE;
	memory.persMem.armConfig[UPPERARM].id = UPPERARM;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.encoder.id = UPPERARM;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.id = UPPERARM;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.minAngle= -90.0;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.encoder.nullAngle= 106-2.3;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.maxAcc= 500;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.maxSpeed= 140;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.kP= 0.3;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.kD= 0.000;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.kI= 0.0;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.sampleRate= 20;
	memory.persMem.armConfig[UPPERARM].config.stepperArm.stepper.initialMicroSteps = 8;

	
	// Hip (stepper/Encoder)	
	memory.persMem.armConfig[HIP].actuatorType = STEPPER_ENCODER_TYPE;   
	memory.persMem.armConfig[HIP].id = HIP;
	memory.persMem.armConfig[HIP].config.stepperArm.encoder.id = HIP;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.id = HIP;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.minAngle= -90.0;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.maxAngle= +90.0;
	memory.persMem.armConfig[HIP].config.stepperArm.encoder.nullAngle= -35.4;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.maxAcc= 1000;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.maxSpeed= 140;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.kP= 0.3;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.kD= 0.0;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.kI= 0.0;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.sampleRate= 20;
	memory.persMem.armConfig[HIP].config.stepperArm.stepper.initialMicroSteps = 16;
}

StepperSetupData stepperSetup[MAX_STEPPERS] {
	// Arm      clockwise 	enable  		dir     		 clock   			angle	current[A]
	{ HIP,      true,		HIP_EN_PIN, 	HIP_DIR_PIN, 	 HIP_CLK_PIN, 		1.8,	2.8, BLACK, GREEN, RED, BLUE},
	{ UPPERARM, true,		UPPERARM_EN_PIN,UPPERARM_DIR_PIN,UPPERARM_CLK_PIN, 	1.8,	3.5, BLACK, GREEN, RED, BLUE},
	{ FOREARM,  true,		FOREARM_EN_PIN,	FOREARM_DIR_PIN, FOREARM_CLK_PIN, 	1.8,	1.4, NON_COLOR, NON_COLOR, NON_COLOR, NON_COLOR},
	{ ELLBOW,   false,		ELBOW_EN_PIN, 	ELBOW_DIR_PIN,	 ELBOW_CLK_PIN, 	1.8,	0.7, BLACK, GREEN, RED, BLUE},
	{ WRIST,    false,		WRIST_EN_PIN,	WRIST_DIR_PIN,	 WRIST_CLK_PIN, 	1.8,	0.4, BLACK, GREEN, RED, BLUE}
};

RotaryEncoderSetupData encoderSetup[MAX_ENCODERS] {
	// 	ActuatorId	I2CAddreess			I2Bus clockwise
	{ HIP,			AS5048_ADDRESS+0,	I2C1, false},
	{ UPPERARM,		AS5048_ADDRESS+3,	I2C0, true},
	{ FOREARM,		AS5048_ADDRESS+2,	I2C0, true},
	{ ELLBOW,		AS5048_ADDRESS+1,	I2C0, false},
	{ WRIST,		AS5048_ADDRESS+0,	I2C0, true}
};

ServoSetupData servoSetup[MAX_SERVOS] {
	// actuator Herkulex ID					reverse   minTorque maxTorque, setupSpeed (° /s )
	{ HAND,		HAND_HERKULEX_MOTOR_ID,		false,    65,       256,       30},
	{ GRIPPER,	GRIPPER_HERKULEX_MOTOR_ID,	true,     65,       256,       30}
};

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

	logger->print(F(" microSteps="));
	logger->print(initialMicroSteps);


	logger->print(F(" PID("));
	logger->print(kP,2);
	logger->print(",");
	logger->print(kI,2);
	logger->print(",");
	logger->print(kD,2);
	logger->print(")");

	logger->print(F(" maxSpeed="));
	logger->print(maxSpeed,2);
	logger->print(F(" maxAcc="));
	logger->print(maxAcc,2);

	logger->print(F(" maxAcc="));
	logger->print(maxAcc,2);
	logger->println(F(")"));
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
	logger->print(F(" I2CBus="));
	logger->print(I2CBusNo);
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


void logPin(uint8_t pin) {
	logger->print(F("pin("));
	logger->print(pin);
	logger->print(F(")"));
}


