
/*
 * Space.h
 *
 * Created: 23.04.2016 19:41:15
 *  Author: JochenAlt
 */ 


#ifndef SPACE_H_
#define SPACE_H_

#include "Arduino.h"

class AngleMovement {
	public:
		AngleMovement () {
			angleStart = 0;
			angleEnd = 0;
			startTime = 0;
			endTime = 0;	
		}

		AngleMovement (float startAngle, uint32_t pStartTime, float endAngle, uint32_t pEndTime) {
			angleStart = startAngle;
			angleEnd = endAngle;
			startTime = pStartTime;
			endTime = pEndTime;
		}
		void operator= (const AngleMovement& p) {
			angleStart = p.angleStart;
			angleEnd = p.angleEnd;
			startTime = p.startTime;
			endTime = p.endTime;
		}
		
		void set(uint32_t now,float pStartAngle, float pEndAngle, uint32_t pDurationMs) {
			angleStart = pStartAngle;
			angleEnd = pEndAngle;
			startTime = now;
			endTime = now+pDurationMs;
		}
		
		bool isNull() {
			return startTime != 0;
		};
		void setNull() {
			startTime = 0;
		}
		
		float getRatioDone (uint32_t now) {
			return float(now - startTime)/float(endTime-startTime); // ratio in time, 0..1
		}
		float getCurrentAngle(uint32_t now) {
			float t = getRatioDone(now);
			return angleStart + t*(angleEnd-angleStart);
		}
		
		bool timeInMovement(uint32_t now) {
			if (!isNull())
				return now<= endTime;
			else
				return false;
		}

		float angleStart;
		float angleEnd;
		uint32_t startTime;
		uint32_t endTime;
};

class AngleMovementQueue {
	public:
		void set(float currentAngle, uint32_t now, float endAngle, uint32_t pDurationTime) {
			if (movement.isNull()) {
				movement.set(now, currentAngle, endAngle, now + pDurationTime);
			} else {
				// if new position is earlier than the current movement, overwrite
				if (movement.endTime + MOTOR_SAMPLE_RATE>= now+pDurationTime) {
					movement.set(now, currentAngle, endAngle, now+pDurationTime);
					nextMovement.setNull();
				} else {
					// end point is later, add next movement
					nextMovement.set(movement.angleEnd, movement.angleStart, endAngle, now + pDurationTime);
				}				
			}
		}
		
		bool isNull() {
			return movement.isNull();
		}
		
		bool timeInMovement(uint32_t now) {
			if (movement.timeInMovement(now)) 
				return true;
			else 
				return nextMovement.timeInMovement(now);
		}

		void setTime(uint32_t now) {
			if (!movement.isNull()) {
				if (movement.endTime> now) {
					if (!nextMovement.isNull()) {
						movement = nextMovement;
						if (nextMovement.endTime < now)
							nextMovement.setNull();
					} else 
						movement.setNull();
				}
			}
		}
		
		float getCurrentAngle(uint32_t now) {
			if (movement.timeInMovement(now))
				return movement.getCurrentAngle(now);
			if (nextMovement.timeInMovement(now))
				return nextMovement.getCurrentAngle(now);
			return 0;
		}
		AngleMovement movement;	
		AngleMovement nextMovement;
};
#endif