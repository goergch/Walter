
#include "Arduino.h"

#ifndef __SETUP_H__
#define __SETUP_H__

#include "ams_as5048b.h"
#define MAX_MOTORS 6					// total number of motors

#define CONNECTION_BAUD_RATE 115200		// baud rate for connection to main board
#define MAX_INT_16 ((2<<15)-1)

#define MOTOR_KNOB_PIN PIN_A1
#define MOTOR_KNOB_SAMPLE_RATE 50		// every [ms] the potentiometer is sampled 
#define SERVO_SAMPLE_RATE 100			// every [ms] the motors get a new position
#define ENCODER_SAMPLE_RATE 100		// every [ms] the motors get a new position
#define STEPPER_SAMPLE_RATE_US 100L		// every [us] the motors get a new position (timer based) 

#define ANGLE_SAMPLE_RATE 100			// every [ms] the uC gets a new angle

#define LED PIN_A0

#define HERKULEX_BROADCAST_ID 0xfe		// Herkulex Broadcast ID
#define HERKULEX_MOTOR_ID 0xFD			// HERKULEX_BROADCAST_ID				// ID of wrist motor

struct StepperData {
	uint8_t directionPIN; // PIN for direction indication
	uint8_t clockPIN;	  // PIN for step ticks
	uint8_t enablePIN;	  // PIN for enabling this stepper
	float degreePerStep;  // typically 1.8 or 0.9° per step
	uint8_t microSteps;	  // configured micro steps, typically 1, 2,4,16
	bool direction;		  // forward or reverse direction?
	float gearReduction;  // ratio given by gearbox, not yet used
	uint16_t maxFullStepRate; // maximum full steps per second
};

static StepperData StepperConfig[MAX_MOTORS-1] { { PIN_A2,PIN_A3,PIN_A4,1.8, 2, false,1.0,4000}, 
								                { PIN_A6,PIN_A7,PIN_C7,1.8,1, true,1.0,5000},
												{ PIN_C6,PIN_C5,PIN_C4,1.8,1, true,1.0,5000},
												{ PIN_C3,PIN_C2,PIN_D7,1.8,1, true,1.0,5000},
												{ PIN_D6,PIN_D5,PIN_D4,1.8,1, true,1.0,5000} };

	
struct RotaryEncoderData {
	bool programmI2CAddress;
	uint8_t I2CAddress;
	bool clockwise;
};

#define I2C_ADDRESS_ADDON 1					// add one to I2C address of conflicting sensor
#define I2C_ADDRESS_ADDON_VDD_PIN PIN_B1	// power pins for sensor with conflicting I2C address
#define I2C_ADDRESS_ADDON_GND_PIN PIN_B0	// GND pin for sensor with conflicting I2C address

static RotaryEncoderData EncoderConfig[MAX_MOTORS-1] { { true,  AS5048_ADDRESS+0, true }, 
													   { false, AS5048_ADDRESS+0, true },
													   { false, AS5048_ADDRESS+1, true },
													   { false, AS5048_ADDRESS+2, true },
													   { false, AS5048_ADDRESS+3, true }};
		

// #define DEBUG_HERKULEX // logging output of Herkulex Servo
// #define DEBUG_ENCODERS // logging output of encoder angles

#define USE_FAST_DIGITAL_WRITE // use macro based digitalWrite

#endif