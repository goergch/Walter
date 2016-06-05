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

class ArmConfig {
	public:
		static void setDefaults();
		void print();

		float  nullAngle;
		float  encoderNullAngle;

		float pivKp;		
		float pivKi;
		float pivKd;

		float  maxSpeed;         //  [°/ms];
		bool reverse;
		float   maxAngle;			// [°]
		float   minAngle;			// [°]
};


class Actuator
{
	public:
		Actuator();
		Actuator( const Actuator &c );
	
		bool isInitialized() { return hasBeenInitialized;}
		void setup(int motorNumber);
			
		virtual void loop(uint32_t now) = 0;

		virtual void setAngle(float angle,uint32_t pDuration_ms) = 0;
		virtual void moveToAngle(float angle,uint32_t pDuration_ms) = 0;
		virtual float getCurrentAngle() = 0;
	
		void addToNullPosition(float nullAngle);
		float getNullPosition();	
		void printConfiguration();
		PIV* getPIV() { return &pivController;};
		void setPIVParams();
		int getMotorNumber() { return myMotorNumber;};
	protected:
		int myMotorNumber;
		ArmConfig* config;
		AngleMovement movement;
		bool beforeFirstMove;
	private:
	
		bool hasBeenInitialized;
		float mostRecentAngle;
		
		uint32_t previousLoopCall;
		PIV pivController;
		
}; //MotorDriver



#endif //__MOTORDRIVER_H__
