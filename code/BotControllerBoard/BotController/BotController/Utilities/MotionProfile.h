#ifndef __MOTIONPROFILE_H__
#define __MOTIONPROFILE_H__

#include "Arduino.h"

/**
 * Provides a simple trapezoidal motion profile generator.
 *
 * <p>
 * Usage:
 * // Includes
 * #include "MotionProfile.h"
 *
 * </p>
 *
 * @author      Wilbert van de Ridder <l.w.vanderidder @ student.utwente.nl>
 * @version     1.0 
 * @since       2014-05-12
 */

	
class MotionProfile {		
	public:	
	enum MotionProfiletype {TRAPEZOIDAL_PROFILE, LINEAR_PROFILE };

		/**	
		 * Constructor
		 * 
		 * @param int aVelocityMax maximum velocity
		 * @param int aAccelerationMax maximum acceleration
		 * @param short aMethod method of profile generation
		 */
		MotionProfile( ) {
			maxVelocity = 0;
			maxAcceleration = 0;
			method = LINEAR_PROFILE;

			// Time variables
			lastTime = 0;
			delta = 0;
		}
		void setup(float aVelocityMax, float aAccelerationMax, MotionProfiletype aMethod);
						
		/**	
		 * Updates the state, generating new setpoints
		 *
		 * @param aSetpoint The current setpoint.
		 */
		float update(float aSetpoint);
		
		bool getFinished();
		void pause();
		void reset();
	private:
		void calculateConstantVelocityProfile(float);
		void calculateTrapezoidalProfile(float);
				
		float maxVelocity;
		float maxAcceleration;
		short method;

		float position;
		float oldPosition;
		float velocity;
		float oldVelocity;
		
		uint32_t lastTime;
		float delta;	
};
#endif
