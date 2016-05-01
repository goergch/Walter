/* 
* Motors.h
*
* Created: 22.04.2016 19:26:44
* Author: JochenAlt
*/


#ifndef __MOTORS_H__
#define __MOTORS_H__


#include "Arduino.h"
#include "MotorDriver.h"
#include "MotorDriverHerkulexImpl.h"
#include "MotorDriverStepperImpl.h"

#include "TimePassedBy.h"

class Motors {
	public:
		Motors();
		Motors( const Motors&c );


		void printMenuHelp();
		void interactiveLoop();
		void interactive(bool on);
		bool interactive() { return interactiveOn;}

		void setup();
		void loop();
		void stepperLoop();

	private:
		MotorDriver* getMotor(int motorNumber);	
		MotorDriverHerkulexImpl wristMotor;
		MotorDriverStepperImpl stepper[MAX_MOTORS-1];
		// static MotorDriver* motorDriverArray[MAX_MOTORS];
		uint8_t numberOfMotors;
		MotorDriver* currentMotor;				// currently set motor used for interaction
		TimePassedBy motorKnobTimer;			// used for measuring sample rate of motor knob
		bool interactiveOn;
}; //Motors

#endif //__MOTORS_H__
