
/*
 * DriveBase.h
 *
 * Interface base class of stepper and servos. 
 *
 * Created: 10.06.2016 15:24:02
 *  Author: JochenAlt
 */ 


#ifndef __DRIVE_BASE_H__
#define __DRIVE_BASE_H__

class DriveBase {
	public:
	DriveBase () {movement.setNull();};
	DriveBase (DriveBase& base) {movement = base.movement;};
		
		// sets the to-be angle and the time in ms for the movement. Movement starts immediately within loop()
	virtual void setAngle(float pAngle,uint32_t pAngleTargetDuration_ms) = 0;
	virtual void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration_ms) = 0;
	virtual void loop(uint32_t now_ms) = 0;
	virtual float getCurrentAngle() = 0;
	virtual void enable() = 0;
	virtual void disable() = 0;
	virtual bool isEnabled() = 0;
	
	AngleMovement movement;
};


#endif