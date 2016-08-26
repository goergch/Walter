
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
			timeDiffRezi = 0;	

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
		
		void print(uint8_t no) {
			if (!isNull()) {
				Serial.print(F("move("));
				Serial.print(no);Serial.print("[");
				Serial.print(angleStart);
				Serial.print("(");
				Serial.print(startTime);
				Serial.print(F(")..."));
				Serial.print(angleEnd);
				Serial.print("(");
				Serial.print(endTime);
				Serial.print(")]");
			}
			else
				Serial.print(F("move=NULL"));
		}
		
		void set(float pStartAngle, float pEndAngle, uint32_t now, uint32_t pDurationMs) {
			/*
			Serial.print("move.set(");
			Serial.print(pStartAngle);
			Serial.print(",");
			Serial.print(pEndAngle);
			Serial.print(",");
			Serial.print(now);
			Serial.print(",");
			Serial.print(pDurationMs);
			Serial.println(")");
			*/
			angleStart = pStartAngle;
			angleEnd = pEndAngle;
			startTime = now;
			endTime = now+pDurationMs;
			timeDiffRezi = 1.0/float(endTime-startTime);
		}
		
		bool isNull() {
			return startTime == 0;
		};
		void setNull() {
			startTime = 0;
			angleStart = 0;
			angleEnd = 0;
			endTime = 0;
			timeDiffRezi = 0;
		}
		
		float getRatioDone (uint32_t now) {
			return float(now - startTime)*timeDiffRezi; // ratio in time, 0..1
		}
		float getCurrentAngle(uint32_t now) {
			float position = 0;
			if (now>=endTime)
				position = angleEnd;
			else {
				float t = float(now - startTime)*timeDiffRezi; // ratio in time, 0..1
				position = angleStart + t*(angleEnd-angleStart); 				
			}
		
			return position;
		}
		bool isForward() {
			return angleEnd>angleStart;
		}
		
		bool timeInMovement(uint32_t now) {
			if (!isNull())
				return now<= endTime;
			else
				return false;
		}

		float angleStart;
		float angleEnd;
		float timeDiffRezi;
		uint32_t startTime;
		uint32_t endTime;

};

class AngleMovementQueue {
	public:
		void print() {
			Serial.print(F("move{"));
			if (!movement.isNull()) {
				Serial.print(F("1["));
				Serial.print(movement.angleStart);
				Serial.print("(");
				Serial.print(movement.startTime);
				Serial.print(F(")..."));
				Serial.print(movement.angleEnd);
				Serial.print("(");
				Serial.print(movement.endTime);
				Serial.print(")]");
			}
			if (!nextMovement.isNull()) {
				Serial.print("2[");
				Serial.print(nextMovement.angleStart);
				Serial.print("(");
				Serial.print(nextMovement.startTime);
				Serial.print(")...");
				Serial.print(nextMovement.angleEnd);
				Serial.print("(");
				Serial.print(nextMovement.endTime);
				Serial.print(")]");
			}
			Serial.print("}");
		}
		void set(float currentAngle,  float endAngle, uint32_t now, uint32_t pDurationTime) {
			if (movement.isNull()) {
				movement.set(currentAngle, endAngle, now, pDurationTime);
			} else {
				// if new position is earlier than the current movement, overwrite
				if (movement.endTime>= now+pDurationTime) {
					movement.set(currentAngle, endAngle, now, pDurationTime);
					nextMovement.setNull();
				} else {
					// end point is later, add next movement
					nextMovement.set(movement.angleEnd, endAngle, movement.endTime, pDurationTime+now-movement.endTime);
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
				if (movement.endTime< now) {
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
			float result = 0;
			if (movement.timeInMovement(now))
				result  = movement.getCurrentAngle(now);
			if (nextMovement.timeInMovement(now))
				result  = nextMovement.getCurrentAngle(now);
	
			return result;
		}
		AngleMovement movement;	
		AngleMovement nextMovement;
};
#endif