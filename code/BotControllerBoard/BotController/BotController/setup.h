
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

static uint16_t StepperPinDirection[MAX_MOTORS-1] { PIN_A3,PIN_A5,PIN_A7,PIN_B1,PIN_B3 };
static uint16_t StepperPinClock[MAX_MOTORS-1] { PIN_A2,PIN_A4,PIN_A6,PIN_B0,PIN_B2 };
static uint16_t StepperPinEnable[MAX_MOTORS-1] { PIN_A4,PIN_A4,PIN_A6,PIN_B0,PIN_B2 };

static float StepperDegreePerStep[MAX_MOTORS-1] { 1.8/4,1.8,1.8,1.8,1.8 };
static bool StepperDirection[MAX_MOTORS-1] { true, true, true, true, true };
	
		
#endif