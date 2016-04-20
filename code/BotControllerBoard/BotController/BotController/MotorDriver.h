/* 
* MotorDriver.h
*
* Created: 20.04.2016 15:21:40
* Author: SuperJochenAlt
*/


#ifndef __MOTORDRIVER_H__
#define __MOTORDRIVER_H__


#include <Arduino.h>
#include <Herkulex.h>

class MotorDriver
{
	public:
	MotorDriver() {
		hasBeenInitialized = false;
	}
	MotorDriver( const MotorDriver &c );
	MotorDriver& operator=( const MotorDriver &c );
	
	bool isInitialized() { return hasBeenInitialized;}
	void initialize() { hasBeenInitialized = true;}
		
	virtual void setAngle(float angle,long pDuration_ms) = 0;
	virtual float getAngle() = 0;

	private:
		bool hasBeenInitialized; 
}; //MotorDriver


class MotorDriverHerkulexImpl: public MotorDriver
{
//functions
public:
	MotorDriverHerkulexImpl() {
		currentAngle = 0;
	}
	
	void setup(long baudrate);
	virtual void setAngle(float angle, long pDuration_ms);
	virtual float getAngle();
		
private:	
	HerkulexClass herkulexServo; 
	float currentAngle;
}; //MotorDriver

#endif //__MOTORDRIVER_H__
