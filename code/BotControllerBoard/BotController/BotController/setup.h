
#include "Arduino.h"

#define MAX_MOTORS 6					// total number of motors

#define CONNECTION_BAUD_RATE 115000		// baud rate for connection to main board
#define HERKULEX_SERVO_BAUD_RATE 57600  // baud rate for connection to herkulex servo
#define MAX_INT_16 ((2<<15)-1)

#define MOTOR_KNOB_PIN PIN_A0
#define MOTOR_KNOB_SAMPLE_RATE 100		// every [ms] the potentiometer is sampled 