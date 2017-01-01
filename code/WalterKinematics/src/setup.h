/*
 * setup.h
 *
 *  Created on: 27.06.2016
 *      Author: JochenAlt
 */

#ifndef SETUP_H_
#define SETUP_H_

#include <valarray>

#define LOGVIEW_MAXSIZE 10000


// logging switches
// #define KINEMATICS_LOGGING

// bot consists of 6 actuators plus gripper
const int Actuators = 6; 			// excluding gripper
const int NumberOfActuators = 7; 	// including gripper
enum ActuatorType { HIP=0, UPPERARM = 1, FOREARM=2, ELLBOW = 3, WRIST=4, HAND=5,GRIPPER=6}; // used as array indexes
enum CoordDimType { X=0, Y=1, Z=2 };

typedef double rational;
typedef int milliseconds;							// time
typedef rational mmPerMillisecond;					// speed
typedef rational mmPerSecond;						// speed
typedef rational millimeter;						// distance
typedef rational mmPerMillisecondPerMillisecond;	// acceleration

// allowed difference when checking floats for equality
const rational floatPrecision=0.000000001f;

// Kinematics constants of bot, taken from CAD models. Al in [mm]
const rational HipHeight 			= 263;
const rational UpperArmLength 		= 225;
const rational EllbowLength 		= 82;
const rational ForearmLength 		= 136;
const rational TotalForearmLength 	= EllbowLength+ForearmLength;
const rational HandLength			= 30;
const rational ForehandLength 		= 27;
const rational GripperLeverLength  	= 43;
const rational GripperLength  		= 60;
const rational totalHandLength  	= HandLength+ForehandLength+GripperLeverLength+GripperLength/2;

// struct used to fetch data from uC
struct ActuatorStateType {
	float currentAngle;
	float minAngle;
	float maxAngle;
	float nullAngle;
};

// UI trajectories are samples with 1000/25 = 40 fps
const milliseconds UITrajectorySampleRate = 50;
const milliseconds BotTrajectorySampleRate = 50;
const milliseconds CortexSampleRate  = 50;



#endif /* SETUP_H_ */
