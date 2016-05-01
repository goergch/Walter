
#include "Arduino.h"

#ifndef __SETUP_H__
#define __SETUP_H__

#define MAX_MOTORS 6					// total number of motors

#define CONNECTION_BAUD_RATE 115200		// baud rate for connection to main board
#define MAX_INT_16 ((2<<15)-1)

#define MOTOR_KNOB_PIN PIN_A1
#define MOTOR_KNOB_SAMPLE_RATE 50		// every [ms] the potentiometer is sampled 
#define SERVO_SAMPLE_RATE 50			// every [ms] the motors get a new position
#define STEPPER_SAMPLE_RATE 1			// every [ms] the motors get a new position
#define STEPPER_SAMPLE_RATE_US 180			// every [us] the motors get a new position

#define LED PIN_A0

#define HERKULEX_BROADCAST_ID 0xfe		// Herkulex Broadcast ID
#define HERKULEX_MOTOR_ID 0xFD			// HERKULEX_BROADCAST_ID				// ID of wrist motor

struct StepperData {
	uint8_t directionPIN;
	uint8_t clockPIN;
	uint8_t enablePIN;
	float degreePerStep;
	bool direction;
	
};

static StepperData StepperPort[MAX_MOTORS-1] { { PIN_A3,PIN_A4,PIN_A5,1.8/4,true}, 
								                { PIN_A6,PIN_A7,PIN_C7,1.8/4,true},
												{ PIN_C6,PIN_C5,PIN_C4,1.8/4,true},
												{ PIN_C3,PIN_C2,PIN_D7,1.8/4,true},
												{ PIN_D6,PIN_D5,PIN_D4,1.8/4,true} };

	
		
#endif