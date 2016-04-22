/*
* MotorDriver.h
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#ifndef __MOTORDRIVER_H__
#define __MOTORDRIVER_H__


#include <Arduino.h>
#include "setup.h"

class MotorDriverConfig {
	public:
		static void setDefaults();
		void println();

		float  nullAngle;
		float  maxSpeed;         //  [°/ms];
		float  maxAcceleration;  //  [°/ms^2];
		bool reverse;
		float   maxAngle;			// [°]
		float   minAngle;			// [°]
};


class MotorDriver
{
	public:
		MotorDriver();
		MotorDriver( const MotorDriver &c );
	
		bool isInitialized() { return hasBeenInitialized;}
		void setup(int motorNumber);	

		void setAngleTarget(float angle,long pDuration_ms);
	
		virtual void setAngle(float angle,long pDuration_ms) = 0;
		virtual float getAngle() = 0;
	
		void addToNullPosition(float nullAngle);
		float getNullPosition();	
		void println();
		void print();
	
	protected:
		int myMotorNumber;
		MotorDriverConfig* config;
	private:
		bool hasBeenInitialized;
		float angleTarget;
		float currentAngle;
		uint32_t angleTargetStartTime;
		uint32_t angleTargetEndTime;

}; //MotorDriver



#endif //__MOTORDRIVER_H__
