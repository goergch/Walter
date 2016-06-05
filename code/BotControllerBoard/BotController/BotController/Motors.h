/* 
* Motors.h
*
* Created: 22.04.2016 19:26:44
* Author: JochenAlt
*/


#ifndef __MOTORS_H__
#define __MOTORS_H__


#include "Arduino.h"
#include "Actuator.h"
#include "MotorDriverHerkulexImpl.h"
#include "MotorDriverStepperImpl.h"
#include "RotaryEncoder.h"

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
		bool checkEncoders();
		void setNullValues();
		void printStepperConfiguration();
		void loop();
		void stepperLoop();
		void printEncoderAngles();

	private:
		void transferEncoderAngle(int motorNumber);
		Actuator* getMotor(int motorNumber);	
		MotorDriverHerkulexImpl wristMotor;
		MotorDriverStepperImpl stepper[MAX_MOTORS-1];
		RotaryEncoder encoders[MAX_MOTORS-1];
		uint8_t numberOfMotors;
		uint8_t numberOfEncoders;

		Actuator* currentMotor;				// currently set motor used for interaction
		TimePassedBy motorKnobTimer;			// used for measuring sample rate of motor knob
		bool interactiveOn;
}; //Motors

#endif //__MOTORS_H__
