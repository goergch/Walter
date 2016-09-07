/*
 * setup.h
 *
 *  Created on: 27.06.2016
 *      Author: SuperJochenAlt
 */

#ifndef SETUP_H_
#define SETUP_H_

#include <valarray>

// communication to uC board
#define ACTUATOR_CTRL_SERIAL_PORT "COM3"
#define ACTUATOR_CTRL_BAUD_RATE 115200
#define ACTUATOR_CTRL_LOGGER_PORT "COM4"
#define ACTUATOR_CTRL_LOGGER_BAUD_RATE 115200

// logging switches
#define KINEMATICS_LOGGING

// bot consists of 6 actuators plus gripper
const int Actuators = 6; 			// excluding gripper
const int NumberOfActuators = 7; 	// including gripper
enum ActuatorType { HIP=0, UPPERARM = 1, FOREARM=2, ELLBOW = 3, WRIST=4, HAND=5,GRIPPER=6}; // used as array indexes
enum CoordDimType { X=0, Y=1, Z=2 };

typedef double rational;
typedef uint32_t milliseconds;		// time
typedef rational mmPerMillisecond;	// speed
typedef rational millimeter;		// speed

// allowed difference when checking floats for equality
const rational floatPrecision=0.000000001f;

// Kinematics constants of bot, taken from CAD models. Al in [mm]
const rational HipHeight 			= 263;
const rational UpperArmLength 		= 225;
const rational EllbowLength 		= 80;
const rational ForearmLength 		= 140;
const rational TotalForearmLength 	= EllbowLength+ForearmLength;
const rational HandLength			= 30;
const rational ForehandLength 		= 27;
const rational GripperLeverLength  	= 43;
const rational GripperLength  		= 60;
const rational totalHandLength  	= HandLength+ForehandLength+GripperLeverLength+GripperLength/2;



struct ActuatorStateType {
	float currentAngle;
	float minAngle;
	float maxAngle;
	float nullAngle;
};

// trajectories points are computed every 50ms
const milliseconds TrajectorySampleRate = 50;

#endif /* SETUP_H_ */
