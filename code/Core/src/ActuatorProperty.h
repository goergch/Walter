/*
 * ActuatorProperty.h
 *
 * Defines settings of all actuators in terms of allowed range and gearratio, used in uC and in TrajectoryCode
 *
 *  Created on: 05.09.2016
 *      Author: JochenAlt
 */

#ifndef ACTUATORPROPERTY_H_
#define ACTUATORPROPERTY_H_

// defining the absolute limits, gear ratio and limits of joints
struct ActuatorConfiguration {
	enum ActuatorId {HIP=0, UPPERARM=1, ELLBOW=2, FOREARM=3, WRIST=4, HAND=5, GRIPPER=6 };
	ActuatorId id;		
	float gearRatio; 	// gear reduction in 1:n
	float angleOffset;	// move the null angle to a certain offsen (used for upperarm)
};

typedef ActuatorConfiguration AllActuatorsConfigType[7];	// all actuators
extern AllActuatorsConfigType actuatorConfigType; 						// defined in ActuatorProperty.cpp

#endif /* ACTUATORPROPERTY_H_ */
