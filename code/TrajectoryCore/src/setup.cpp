/*
 * setup.cpp
 *
 *  Created on: 07.08.2016
 *      Author: JochenAlt
 */


#include "setup.h"
#include "Util.h"

// limits of joints
// (min, max, maxSpeed[rad/s], maxAcc [rad/s*s]
ActuatorLimitsType actuatorLimits =  {
	{ radians(-179.0f)	,radians(179.0f), 	radians(120), 	radians(1000)}, // Hip
	{ radians(-89.0f)	,radians(89.0f), 	radians(180), 	radians(1000)},	// Upperarm
	{ radians(-230.0f)	,radians(45.0f), 	radians(180), 	radians(1000)},	// Forearm
	{ radians(-180.0f)	,radians(180.0f),	radians(360),	radians(1000)}, // Ellbow
	{ radians(-100.0f)	,radians(100.0f),	radians(360),	radians(1000)}, // Wrist
	{ radians(-180.0f)	,radians(180.0f),	radians(360),	radians(1000)}, // Hand
	{ radians(11.0f)	,radians(60.0f),	radians(360),	radians(1000)} 	// Gripper
};
