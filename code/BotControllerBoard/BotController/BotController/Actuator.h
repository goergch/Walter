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
#include "RotaryEncoder.h"
#include "ActuatorConfig.h"
#include "GearedStepperDrive.h"
#include "HerkulexServoDrive.h"

#include "DriveBase.h"
class Actuator
{
	public:
		Actuator();
		Actuator( const Actuator &c );
	
		bool isInitialized() { return hasBeenInitialized;}
		void setup(ActuatorConfig* pConfigData, ActuatorSetupData* pSetupData, HerkulexServoDrive* servo);
		void setup(ActuatorConfig* pConfigData, ActuatorSetupData* pSetupData, GearedStepperDrive* stepper, RotaryEncoder* encoder);

		void setup();
 			
		void loop(uint32_t now) {
			drive()->loop(now);
		}
		void setAngle(float angle,uint32_t pDuration_ms) {
			drive()->setAngle(angle,pDuration_ms);
		};
		void changeAngle(float angle,uint32_t pDuration_ms) {
			drive()->changeAngle(angle,pDuration_ms);
		};
		float getCurrentAngle() {
			return drive()->getCurrentAngle();
		}
		void enable() {
			return drive()->enable();
		}
		void disable() {
			return drive()->disable();
		}

		bool setCurrentAsNullPosition();

		void printConfiguration();

		bool hasEncoder() { return encoder != NULL; }
		bool hasStepper() { return stepperDrive != NULL; }
		bool hasServo() { return servoDrive != NULL; }

		RotaryEncoder& getEncoder () { return *encoder; }
		GearedStepperDrive& getStepper() { return *stepperDrive; }
		HerkulexServoDrive& getServo() { return *servoDrive; }
			
		void setMaxAngle(float angle);
		void setMinAngle(float angle);
		void setNullAngle(float angle);

		float getMaxAngle();
		float getMinAngle();
		float getNullAngle();
		
		ActuatorConfig& getConfig() { return *configData; };
		ActuatorSetupData& getSetup() { return *setupData; };
		void printName();

	protected:
		DriveBase* drive() { return (stepperDrive != NULL)?(DriveBase*)stepperDrive:(DriveBase*)servoDrive;};
		ActuatorConfig* configData;
		ActuatorSetupData* setupData;

		RotaryEncoder* encoder;
		GearedStepperDrive* stepperDrive;
		HerkulexServoDrive* servoDrive;
	private:
		bool hasBeenInitialized;
		
		uint32_t previousLoopCall;
}; //MotorDriver



#endif //__MOTORDRIVER_H__
