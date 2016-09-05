/*
 * ActuatorProperty.h
 *
 *  Created on: 05.09.2016
 *      Author: JochenAlt
 */

#ifndef ACTUATORPROPERTY_H_
#define ACTUATORPROPERTY_H_

// defining the absolute limits of joints
struct ActuatorLimitType {
	float gearRatio;
	float minAngle;
	float maxAngle;
	float maxSpeed;
	float maxAcc;
};

typedef ActuatorLimitType ActuatorLimitsType[7];

extern ActuatorLimitsType actuatorLimits;	// defined in setup.cpp

#endif /* ACTUATORPROPERTY_H_ */
