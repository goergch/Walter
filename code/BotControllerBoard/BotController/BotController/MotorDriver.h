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
		void print();

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

		void setAngle(float angle,uint32_t pDuration_ms);
	
		virtual void moveToAngle(float angle,uint32_t pDuration_ms) = 0;
		virtual float getCurrentAngle() = 0;
	
		void addToNullPosition(float nullAngle);
		float getNullPosition();	
		void print();
		PIV* getPIV() { return &pivController;};
		void setPIVParams();
		int getMotorNumber() { return myMotorNumber;};
	protected:
		int myMotorNumber;
		MotorDriverConfig* config;
		AngleMovement movement;
	private:
		bool hasBeenInitialized;
		float mostRecentAngle;
		
		uint32_t previousLoopCall;
		PIV pivController;
		
}; //MotorDriver



#endif //__MOTORDRIVER_H__
