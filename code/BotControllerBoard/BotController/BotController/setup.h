
#include "Arduino.h"

#ifndef __SETUP_H__
#define __SETUP_H__

#include "ams_as5048b.h"
#define MAX_ACTUATORS 6					// total number of arms, some servos, some stepper
#define MAX_ENCODERS 5					// total number of encoders
#define MAX_STEPPERS 5					// total number of steppers
#define MAX_SERVOS 1					// total number of servos

#define CONNECTION_BAUD_RATE 115200		// baud rate for connection to main board
#define MAX_INT_16 ((2<<15)-1)

#define MOTOR_KNOB_PIN PIN_A1
#define MOTOR_KNOB_SAMPLE_RATE 200		// every [ms] the potentiometer is sampled 
#define SERVO_SAMPLE_RATE 100			// every [ms] the motors get a new position
#define ENCODER_SAMPLE_RATE 50			// every [ms] the motors get a new position
#define ANGLE_SAMPLE_RATE 100			// every [ms] the uC gets a new angle
#define STEPPER_SPEED_SAMPLE_RATE 100L  // in [ms]

#define LED PIN_A0

#define HERKULEX_BROADCAST_ID 0xfe		// Herkulex Broadcast ID
#define HERKULEX_MOTOR_ID 0xFD			// HERKULEX_BROADCAST_ID				// ID of wrist motor

enum ActuatorId {HAND=0, WRIST=1, ELLBOW=2, FOREARM=3, UPPERARM=4, HIP=5};
extern void printActuator(ActuatorId actuatorNumber);

struct ActuatorSetupData {
	ActuatorId id;
	void print();
};

extern ActuatorSetupData actuatorSetup[MAX_ACTUATORS];


struct ServoSetupData {
	ActuatorId id;
	uint8_t herkulexMotorId;
	void print();
};

extern ServoSetupData servoSetup[MAX_SERVOS];


struct StepperSetupData {
	ActuatorId id;
	bool direction;		  // forward or reverse direction?
	uint8_t microSteps;	  // configured micro steps, typically 1, 2,4,16

	uint8_t directionPIN; // PIN for direction indication
	uint8_t clockPIN;	  // PIN for step ticks
	uint8_t enablePIN;	  // PIN for enabling this stepper
	
	float degreePerStep;  // typically 1.8 or 0.9° per step
	float gearReduction;  // ratio given by gearbox, not yet used
	uint16_t rpm;		  // maximum full steps per second
	uint16_t accRpm;	  // maximum acceleration in rpm / s, used to produce a trapezoid profile
	void print();
};

extern StepperSetupData stepperSetup[MAX_STEPPERS];

#define I2C_ADDRESS_ADDON 1					// add one to I2C address of conflicting sensor
#define I2C_ADDRESS_ADDON_VDD_PIN PIN_B1	// power pins for sensor with conflicting I2C address
#define I2C_ADDRESS_ADDON_GND_PIN PIN_B0	// GND pin for sensor with conflicting I2C address

struct RotaryEncoderSetupData {
	ActuatorId id;
	bool programmI2CAddress;
	uint8_t I2CAddress;
	bool clockwise;
	void print();
};
extern RotaryEncoderSetupData encoderSetup[MAX_ENCODERS];
		
#define ENCODER_CHECK_MAX_VARIANCE 1.0 // variance [°] in encoder check which is ok 
#define ENCODER_CHECK_NO_OF_SAMPLES 4

#define DEBUG_HERKULEX // logging output of Herkulex Servo
// #define DEBUG_ENCODERS // logging output of encoder angles
// #define DEBUG_STEPPER // logging output of stepper
#define USE_FAST_DIGITAL_WRITE // use macro based digitalWrite
// #define DEBUG_SETUP // logging output of stepper

#endif