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
#include "Space.h"
#include "PIV.h"

class MotorDriverConfig {
	public:
		static void setDefaults();
		void println();

		float  nullAngle;
		float pivKp;		
		float pivKi;
		float pivKd;

		float  maxSpeed;         //  [°/ms];
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
		virtual void loop();

		void setAngleTarget(float angle,long pDuration_ms);
	
		virtual void setRawAngle(float angle,long pDuration_ms, float nextAngle, long pNextDuration_ms) = 0;
		virtual float getRawAngle() = 0;
	
		void addToNullPosition(float nullAngle);
		float getNullPosition();	
		void println();
		void print();
		PIV* getPIV() { return &pivController;};
		void setPIVParams();
		int getMotorNumber() { return myMotorNumber;};
	protected:
		int myMotorNumber;
		MotorDriverConfig* config;
	private:
		bool hasBeenInitialized;
		AngleMovementQueue movement;
		float currentAngle;
		
		uint32_t previousLoopCall;
		PIV pivController;
		
}; //MotorDriver



#endif //__MOTORDRIVER_H__
