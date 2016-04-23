/* 
* MotorDriver.h
*
* Created: 20.04.2016 15:21:40
* Author: JochenAlt
*/


#ifndef __MOTORDRIVER_HERKULEX_IMPL_H__
#define __MOTORDRIVER_HERKULEX_IMPL_H__


#include <Arduino.h>
#include <Herkulex.h>
#include "MotorDriver.h"

class MotorDriverHerkulexImpl: public MotorDriver
{
//functions
public:
	MotorDriverHerkulexImpl(): MotorDriver (){
	}
	
	void setup(int motorNumber, long baudrate);
	virtual void loop();
	virtual void setRawAngle(float angle, long pDuration_ms, float nextAngle, long pNextDuration_ms);
	virtual float getRawAngle();
		
private:	
	HerkulexClass herkulexServo; 
}; //MotorDriver

#endif //__MOTORDRIVER_HERKULEX_IMPL_H__
