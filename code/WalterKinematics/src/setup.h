/*
 * setup.h
 *
 * Author: JochenAlt
 */

#ifndef SETUP_H_
#define SETUP_H_

#include <valarray>

#define LOGVIEW_MAXSIZE 10000 // number of displayed log lines in server view

// logging switches
// #define KINEMATICS_LOGGING

// bot consists of 7 actuators including gripper
const int NumberOfActuators = 7;
enum ActuatorType { HIP=0, UPPERARM = 1, FOREARM=2, ELLBOW = 3, WRIST=4, HAND=5,GRIPPER=6}; // used as array indexes
enum CoordDimType { X=0, Y=1, Z=2 };

typedef double rational;
typedef int milliseconds;							// time
typedef rational mmPerMillisecond;					// speed
typedef rational mmPerSecond;						// speed
typedef rational millimeter;						// distance
typedef rational mmPerMillisecondPerMillisecond;	// acceleration

// allowed difference when checking floats for equality
const int floatPrecisionDigits=8;
const rational floatPrecision=pow(10.0,-floatPrecisionDigits);

const mmPerMillisecondPerMillisecond maxAcceleration_mm_msms = 0.0005; // used in speedprofile


// Kinematics constants of bot, taken from CAD models. Al in [mm]
const rational HipHeight 			= 263;
const rational UpperArmLength 		= 225;
const rational EllbowLength 		= 82;
const rational ForearmLength 		= 133;
const rational TotalForearmLength 	= EllbowLength+ForearmLength;
const rational HandLength			= 30; // distance between wrist centre and start of hand
const rational ForehandLength 		= 27; // distance between hand start of hand and centre of gripper servo
const rational GripperLeverLength  	= 43; // length of ine lever
const rational GripperLength  		= 60; // vertical distance between centre of outer lever and end of gripper
const rational GripperOffset        = 6;  // horizontal offset of gripper's distance to the centre

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
const milliseconds BotTrajectorySampleRate = 100;
const milliseconds CortexSampleRate  = 100;



#endif /* SETUP_H_ */
