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
#include "RotaryEncoder.h"
#include "ActuatorConfig.h"

class GearedStepperDrive;
class HerkulexServoDrive;

class DriveBase {
public:
	DriveBase () {movement.setNull();};
	DriveBase (DriveBase& base) {movement = base.movement;};
	virtual void setAngle(float pAngle,uint32_t pAngleTargetDuration) = 0;
	virtual void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) = 0;
	virtual void loop(uint32_t now) = 0;
	virtual float getCurrentAngle() = 0;
	
	AngleMovement movement;
};


class Actuator
{
	public:
		Actuator();
		Actuator( const Actuator &c );
	
		bool isInitialized() { return hasBeenInitialized;}
		void setup(ActuatorConfigurator& pConfigData, ActuatorSetupData& pSetupData, HerkulexServoDrive& servo);
		void setup(ActuatorConfigurator& pConfigData, ActuatorSetupData& pSetupData, GearedStepperDrive& stepper, RotaryEncoder& encoder);

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
		
		void printConfiguration();
		PIV* getPIV() { return &pivController;};
		void setPIVParams();

		bool hasEncoder() { return encoder != NULL; }
		bool hasStepper() { return stepperDrive != NULL; }
		bool hasServo() { return servoDrive != NULL; }

		RotaryEncoder& getEncoder () { return *encoder; }
		GearedStepperDrive& getStepper() { return *stepperDrive; }
		HerkulexServoDrive& getServo() { return *servoDrive; }
			
		void setMaxAngle(float angle);
		void setMinAngle(float angle);
		float getMaxAngle();
		float getMinAngle();
		
		ActuatorConfigurator& getConfig() { return *configData; };
		ActuatorSetupData& getSetup() { return *setupData; };
		void printName();

	protected:
		DriveBase* drive() { return (stepperDrive != NULL)?(DriveBase*)stepperDrive:(DriveBase*)servoDrive;};
		ActuatorConfigurator* configData;
		ActuatorSetupData* setupData;

		RotaryEncoder* encoder;
		GearedStepperDrive* stepperDrive;
		HerkulexServoDrive* servoDrive;


	private:
		bool hasBeenInitialized;
		float mostRecentAngle;
		
		uint32_t previousLoopCall;
		PIV pivController;
		
}; //MotorDriver



#endif //__MOTORDRIVER_H__
