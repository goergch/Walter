/*
 * setup.cpp
 *
 *  Created on: 07.08.2016
 *      Author: JochenAlt
 */


#include "setup.h"
#include "Util.h"

ActuatorLimitsType actuatorLimits =  {
	{ radians(-180.0f)	,radians(180.0f)},  // Hip
	{ radians(-90.0f)	,radians(90.0f)},	// Upperarm
	{ radians(-270.0f)	,radians(90.0f)},	// Forearm
	{ radians(-180.0f)	,radians(180.0f)}, // Ellbow
	{ radians(-100.0f)	,radians(100.0f)}, // Wrist
	{ radians(-120.0f)	,radians(120.0f)}, // Handturn
	{ radians(0.0f)		,radians(60.0f) } 	// Gripper
};
