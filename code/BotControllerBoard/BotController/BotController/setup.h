
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
#define ENCODER_SAMPLE_RATE 1000			// every [ms] the motors get a new position
#define STEPPER_SAMPLE_RATE_US 180		// every [us] the motors get a new position (timer based) 

#define ANGLE_SAMPLE_RATE 100			// every [ms] the uC gets a new angle

#define LED PIN_A0

#define HERKULEX_BROADCAST_ID 0xfe		// Herkulex Broadcast ID
#define HERKULEX_MOTOR_ID 0xFD			// HERKULEX_BROADCAST_ID				// ID of wrist motor

struct StepperData {
	uint8_t directionPIN;
	uint8_t clockPIN;
	uint8_t enablePIN;
	float degreePerStep;
	bool direction;
	float gearReduction;
	
};

static StepperData StepperConfig[MAX_MOTORS-1] { { PIN_A3,PIN_A4,PIN_A5,1.8/4,true,1}, 
								                { PIN_A6,PIN_A7,PIN_C7,1.8/4,true,1},
												{ PIN_C6,PIN_C5,PIN_C4,1.8/4,true,1},
												{ PIN_C3,PIN_C2,PIN_D7,1.8/4,true,1},
												{ PIN_D6,PIN_D5,PIN_D4,1.8/4,true,1} };

	
struct RotaryEncoderData {
	bool programmI2CAddress;
	uint8_t I2CAddress;
	bool clockwise;
};

#define I2C_ADDRESS_ADDON 1
#define I2C_ADDRESS_ADDON_VDD_PIN PIN_B1
#define I2C_ADDRESS_ADDON_VDD_GND PIN_B0

static RotaryEncoderData EncoderConfig[MAX_MOTORS-1] { { true,  AS5048_ADDRESS+0, true }, 
													   { false, AS5048_ADDRESS+0, true },
													   { false, AS5048_ADDRESS+1, true },
													   { false, AS5048_ADDRESS+2, true },
													   { false, AS5048_ADDRESS+3, true }};
		
#endif