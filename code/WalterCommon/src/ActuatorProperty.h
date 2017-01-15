/*
 * ActuatorProperty.h
 *
 *  Actuator configurations that are used by Walter's Cortex as well as the
 *  Walter's Webserver, like gear ratios, min max angles etc.
 *
 *  Author: JochenAlt
 */

#ifndef ACTUATORPROPERTY_H_
#define ACTUATORPROPERTY_H_

// define the absolute limits, gear ratio and limits of an actuator
struct ActuatorConfiguration {
	enum ActuatorId {HIP=0, UPPERARM=1, ELLBOW=2, FOREARM=3, WRIST=4, HAND=5, GRIPPER=6 };
	ActuatorId 	id;
	float 		gearRatio; 		// gear reduction in 1:n
	float 		angleOffset;	// move the null angle to a certain offsen (used for upperarm)
	float 		minAngle;		// minimum limit in degree
	float 		maxAngle;		// maximum limit in degree
	int 		maxSpeed;		// maximum speed in [RPM]
	int 		maxAcc;		    // maximum acceleration in [RPM/s]
};

typedef ActuatorConfiguration AllActuatorsConfigType[7];	// all actuators
extern AllActuatorsConfigType actuatorConfigType; 			// defined in ActuatorProperty.cpp

#endif /* ACTUATORPROPERTY_H_ */
