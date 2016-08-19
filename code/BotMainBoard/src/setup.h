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

// in some cases, we need to be tolerant to inprecision of floating point arithmetics, e.g. when acos/asin is used.
// So, if checks like <=1 or = PI/2, we use this number as tolerance
const rational floatPrecision=0.000000001f;

// Kinematics constants
const rational HipHeight 			= 263;
const rational UpperArmLength 		= 225;
const rational EllbowLength 		= 80;
const rational ForarmWithoutEllbowLength 	= 140;
const rational ForearmLength 		= EllbowLength+ForarmWithoutEllbowLength;
const rational HandLength			= 30;
const rational ForehandLength 		= 27;
const rational GripperLeverLength  	= 43; // part of Handlength
const rational GripperLength  		= 60; // part of Handlength
const rational totalHandLength  	= HandLength+ForehandLength+GripperLeverLength+GripperLength/2;

const int pearlChainDistance_ms		= 50;

struct ActuatorLimitType {
	rational minAngle;
	rational maxAngle;
};
typedef std::vector<ActuatorLimitType> ActuatorLimitsType;
extern ActuatorLimitsType actuatorLimits;

const float ViewEyeDistance = 1500.0f;	// distance of the eye to the bot
const float ViewBotHeight = 800.0f;		// height of the bot to be viewed


#endif /* SETUP_H_ */
