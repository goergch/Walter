/* 
* RotaryEncoder.h
*
* Created: 23.04.2016 23:13:21
* Author: JochenAlt
*/


#ifndef __ROTARYENCODE_H__
#define __ROTARYENCODE_H__

#include "Arduino.h"
#include "AMS_AS5048B.h"

class RotaryEncoder
{
public:
	RotaryEncoder(int number);
	void setNullPosition();
	float getAngle();

private:
	AMS_AS5048B sensor;
}; //RotaryEncode

#endif //__ROTARYENCODE_H__
