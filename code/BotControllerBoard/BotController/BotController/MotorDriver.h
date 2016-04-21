/*
* MotorDriver.h
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#ifndef __MOTORDRIVER_H__
#define __MOTORDRIVER_H__

#define MAX_MOTORS 6

#include <Arduino.h>

class MotorDriver
{
	public:
	MotorDriver(int number) {
		hasBeenInitialized = false;
		setMotor(number,*this);
	}
	MotorDriver( const MotorDriver &c );
	MotorDriver& operator=( const MotorDriver &c );
	
	bool isInitialized() { return hasBeenInitialized;}
	void initialize() { hasBeenInitialized = true;}
	
	virtual void setAngle(float angle,long pDuration_ms) = 0;
	virtual float getAngle() = 0;
	
	void addToNullPosition(float nullAngle);
	float getNullPosition();
	
	MotorDriver* getMotor(int motorNumber);
	void setMotor(int motorNumber, const MotorDriver& motorDriver);

	void printMenuHelp();
	void menuController();

	private:
		bool hasBeenInitialized;
		int motorNumber;
}; //MotorDriver

class MotorDriverConfig {
	public:
	static void setDefaults();
	void println();

	float nullAngle;
};


#endif //__MOTORDRIVER_H__
