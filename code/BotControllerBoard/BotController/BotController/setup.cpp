
/*
 * setup.cpp
 *
 * Created: 10.06.2016 15:29:54
 *  Author: JochenAlt
 */ 

#include "setup.h"

ActuatorSetupData actuatorSetup[MAX_ACTUATORS] {
	{ HAND },
	{ WRIST},
	{ ELLBOW},
	{ FOREARM},
	{ UPPERARM},
	{ HIP} };

StepperSetupData stepperSetup[MAX_STEPPERS] {
	{ WRIST,    false, 4, PIN_A2, PIN_A3, PIN_A4, 1.8, 56.0/16.0,160, 160},
	{ ELLBOW,   true,  1, PIN_A6, PIN_A7, PIN_C7, 1.8, 1.0,       60, 160},
	{ FOREARM,  true,  1, PIN_C6, PIN_C5, PIN_C4, 1.8, 1.0,       60, 160},
	{ UPPERARM, true,  1, PIN_C3, PIN_C2, PIN_D7, 1.8, 1.0,       60, 160},
	{ HIP,      true,  1, PIN_D6, PIN_D5, PIN_D4, 1.8, 1.0,       60, 160} };


RotaryEncoderSetupData encoderSetup[MAX_ENCODERS] {
	{ WRIST,true,  AS5048_ADDRESS+0, true, true },
	{ ELLBOW,false, AS5048_ADDRESS+0, true, false },
	{ FOREARM, AS5048_ADDRESS+1, true, false },
	{ UPPERARM, false, AS5048_ADDRESS+2, true, false },
	{ HIP, false, AS5048_ADDRESS+3, true, false }};

