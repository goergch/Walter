/*
 * ActuatorProperty.h
 *
 * Defines settings of all actuators, used in uC and in TrajectoryCode
 *
 *  Created on: 05.09.2016
 *      Author: JochenAlt
 */

#ifndef ACTUATORPROPERTY_H_
#define ACTUATORPROPERTY_H_

// defining the absolute limits, gear ratio and limits of joints
struct ActuatorConfigType {
	float gearRatio; 	// gear reduction
	float minAngle;		// minimum limit in radian
	float maxAngle;		// maximum limit in radian
	float maxSpeed;		// maximum speed in rpm
	float maxAcc;		// maximum acceleration in rpm/s
};

typedef ActuatorConfigType AllActuatorsConfigType[7];	// all actuators
extern AllActuatorsConfigType actuatorConfigType; 			// defined in ActuatorProperty.cpp

#endif /* ACTUATORPROPERTY_H_ */
