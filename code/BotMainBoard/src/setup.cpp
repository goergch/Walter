/*
 * setup.cpp
 *
 *  Created on: 07.08.2016
 *      Author: JochenAlt
 */


#include "setup.h"

ActuatorLimitsType actuatorLimits =  {
	{ -90.0,90.0},  // Hip
	{ -90.0,90.0},	// Upperarm
	{ -90.0,90},	// Forearm
	{ -91.0,-91.0}, // Ellbow
	{ 100.0,100.0}, // Wrist
	{ 120.0,120.0}, // Handturn
	{ 0.0,75.0 } 	// Gripper
};
