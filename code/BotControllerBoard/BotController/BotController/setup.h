
#include "Arduino.h"

#define MAX_MOTORS 6					// total number of motors

#define CONNECTION_BAUD_RATE 115200		// baud rate for connection to main board
#define MAX_INT_16 ((2<<15)-1)

#define MOTOR_KNOB_PIN PIN_A1
#define MOTOR_KNOB_SAMPLE_RATE 50		// every [ms] the potentiometer is sampled 
#define MOTOR_SAMPLE_RATE 50			// every [ms] the motors get a new position

#define LED PIN_A0

#define HERKULEX_BROADCAST_ID 0xfe		// Herkulex Broadcast ID
#define HERKULEX_MOTOR_ID 0xFD				// HERKULEX_BROADCAST_ID				// out motor is 1(first)
