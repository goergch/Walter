
/*
 * DriveBase.h
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
	virtual void setAngle(float pAngle,uint32_t pAngleTargetDuration) = 0;
	virtual void changeAngle(float pAngleChange,uint32_t pAngleTargetDuration) = 0;
	virtual void loop(uint32_t now) = 0;
	virtual float getCurrentAngle() = 0;
	virtual void enable() = 0;
	virtual void disable() = 0;
	
	AngleMovement movement;
};


#endif