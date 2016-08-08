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

// Kinematics constants
const rational HipHeight 		= 300;
const rational UpperArmLength 	= 300;
const rational ForearmLength 	= 200;
const rational HandLength  		= 50;

struct ActuatorLimitType {
	rational minAngle;
	rational maxAngle;
};
typedef std::vector<ActuatorLimitType> ActuatorLimitsType;
extern ActuatorLimitsType actuatorLimits;

const float ViewEyeDistance = 1500.0f;	// distance of the eye to the bot
const float ViewBotHeight = 800.0f;		// height of the bot to be viewed


#endif /* SETUP_H_ */
