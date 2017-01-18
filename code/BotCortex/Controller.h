/* 
* Controller.h
*
* Controller running the main loop for steppers, encoders and servos.
* Author: JochenAlt
*/


#ifndef __MOTORS_H__
#define __MOTORS_H__


#include "Arduino.h"
#include "Actuator.h"
#include "HerkulexServoDrive.h"
#include "GearedStepperDrive.h"
#include "RotaryEncoder.h"

#include "TimePassedBy.h"

class Controller {
	public:
		Controller();

		bool setup();
		bool isSetup() { return setuped;};
		bool isPowered() { return powered; };
		bool isEnabled() { return enabled;}
		void enable();
		void disable();

		// check that encoders are working propery by taking some samples and checking that they delivery the same result
		bool checkEncoder(int encoderNo);

		// show configuration in logfile
		void logConfiguration();
		void logAngles();

		void loop(uint32_t now);

		// give all steppers the chance to move a step
		void stepperLoop();

		// return actuator by given actuator number
		Actuator* getActuator(uint8_t number);

		// select one actuator to be used by manuel control
		void selectActuator(uint8_t number);

		// return the currently selected actuator
		Actuator* getCurrentActuator();

		// switch manual control via knob on the panel
		void switchManualActuatorControl(bool OnOff);

		// in case the encoders are not working, you can still use
		// changeAngle which can't work with absolute position but relative changes
		void changeAngle(float incr /* [steps] */, int duration_ms);

		// switch on/off the power supply for the steppers motors (24V, 5A) via a relay
		void switchStepperPowerSupply(bool on);

		// switch on/off the power supply for the servo motors (9V, 1A) via a relay
		void switchServoPowerSupply(bool on);

	private:
		HerkulexServoDrive	servos[MAX_SERVOS];
		GearedStepperDrive	steppers[MAX_STEPPERS];
		RotaryEncoder		encoders[MAX_ENCODERS];
		Actuator			actuators[MAX_ACTUATORS];

		uint8_t numberOfActuators; // number of actuators that have been setup successfully
		uint8_t numberOfEncoders;  // number of encoders that have been setup successfully
		uint8_t numberOfSteppers;  // number of steppers that have been setup successfully
		uint8_t numberOfServos;	   // number of servos that have been setup successfully

		Actuator* currentMotor;				// currently set motor used for interaction
		TimePassedBy manualControlTimer;		// used for measuring sample rate of motor knob
		bool setuped = false;
		bool enabled = false;
		bool powered = false;
};

extern Controller controller;

#endif //__MOTORS_H__
