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
	// actuator							gearratio					angle-offset	
	{ ActuatorConfiguration::HIP,		(90.0/10.0), 				0.0,	},
	{ ActuatorConfiguration::UPPERARM,  (72.0/12.0)*(48.0/12.0), 	0.0,	},
	{ ActuatorConfiguration::FOREARM,   (60.0/14.0)*(48.0/14.0), 	90.0,	},
	{ ActuatorConfiguration::ELLBOW,	(49.0/16.0)*(34.0/16.0), 	0.0,	},
	{ ActuatorConfiguration::WRIST,		(65.0/15.0),				0.0,	}, 
	{ ActuatorConfiguration::HAND,		1.0, 						0.0,	}, 
	{ ActuatorConfiguration::GRIPPER,	1.0, 						0.0,	}  
};
