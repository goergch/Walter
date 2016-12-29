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
// (gear ratio, min[rad], max[rad]
// minAngle/maxAngle/acceleration/maxSpeed is used by Visualizer only
AllActuatorsConfigType actuatorConfigType =  {
	// actuator							gear ratio					angle-offset	min angle			max angle			accel maxspeed (RPM)
	{ ActuatorConfiguration::HIP,		(90.0/10.0), 				0.0,			radians(-179.0f)	,radians(179.0f), 	161,	400},
	{ ActuatorConfiguration::UPPERARM,  (72.0/12.0)*(48.0/12.0), 	0.0,			radians(-89.0f)		,radians(89.0f), 	161,	600},
	{ ActuatorConfiguration::FOREARM,   (60.0/14.0)*(48.0/14.0), 	90.0,			radians(-135.0f)	,radians(135.0f), 	161,	600},
	{ ActuatorConfiguration::ELLBOW,	(49.0/16.0)*(34.0/16.0), 	0.0,			radians(-180.0f)	,radians(180.0f),	161,	600},
	{ ActuatorConfiguration::WRIST,		(65.0/15.0),				0.0,			radians(-100.0f)	,radians(100.0f),	161,	600}, 
	{ ActuatorConfiguration::HAND,		1.0, 						0.0,			radians(-180.0f)	,radians(180.0f),	161,	600}, 
	{ ActuatorConfiguration::GRIPPER,	1.0, 						0.0,			radians(11.0f)		,radians(60.0f),	161,	600}  
};
