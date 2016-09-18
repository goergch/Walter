/*
 * ActuatorProperty.cpp
 *
 *  Created on: 05.09.2016
 *      Author: JochenAlt
 */

#include "ActuatorProperty.h"

// don't include anything else, since this declaration is used in all three tiers
#define radians(deg) (deg*(3.1415927/180.0))

// configuration data of joints
// (gear ratio, min[rad], max[rad], maxSpeed[RPM], maxAcc [RPM/s*s]
AllActuatorsConfigType actuatorConfigType =  {
	{ (90.0/12.0), 				radians(-179.0f)	,radians(179.0f), 	160, 400},  // Hip
	{ (80.0/12.0)*(48.0/14.0), 	radians(-89.0f)		,radians(89.0f), 	160, 600}, // Upperarm
	{ (60.0/14.0)*(48.0/18.0), 	radians(-230.0f)	,radians(45.0f), 	200, 1200}, // Forearm
	{ (56.0/16.0)*(22.0/16.0), 	radians(-180.0f)	,radians(180.0f),	160, 1200}, // Ellbow
	{ 1.0, 						radians(-100.0f)	,radians(100.0f),	160, 1200}, // Wrist
	{ 1.0, 						radians(-180.0f)	,radians(180.0f),	160, 1200}, // Hand
	{ 1.0, 						radians(11.0f)		,radians(60.0f),	160, 1200}  // Gripper
};
