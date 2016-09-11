/*
 * SpeedProfile.h
 *
 * Implementation of a linear and a trapezoidal speed profile.
 * Computes a speed profile between two points a and b
 * defined by start speed, end speed, distance and to-be
 * duration that is used to move from a to b.
 *
 * get(t) returns that speed profile, it takes a number [0..1] representing the time between a and b and returns
 * [0..1] representing the ratio of the distance. A linear profile returns the same number, a trapezoidal profile
 * start with starting speed, increases by maximum acceleration to a middle speed and accelerates to end speed such
 * that the given duration of the movement is fulfilled.
 *
 *  Created on: 09.09.2016
 *      Author: JochenAlt
 */

#ifndef SPEEDPROFILE_H_
#define SPEEDPROFILE_H_

#include "setup.h"

class SpeedProfile {
public:
	enum SpeedProfileType {LINEAR, TRAPEZOIDAL };

	const mmPerMillisecondPerMillisecond acceleration = 0.0005;

	SpeedProfile() {
		null();
	}
	SpeedProfile(const SpeedProfile& par) {
		operator=(par);
	}


	void operator=(const SpeedProfile& par) {
		startSpeed = par.startSpeed;
		endSpeed = par.endSpeed;
		distance = par.distance;
		duration = par.duration;

	}

	bool isNull() {
		return ((distance == 0) && (duration == 0));
	}
	void null() {
		startSpeed = 0;
		endSpeed = 0;
		distance = 0.0;
		duration = 0.0;
	}

	// set parameters necessary for speed profile.
	void set(mmPerMillisecond pStartSpeed, mmPerMillisecond pEndSpeed, millimeter pDistance, rational pDuration);

	// computes the adapted parameter t=[0..1] implementing the speed profile. Returns [0..1]
	rational get(SpeedProfileType type, rational t);

	// return true, if profile is possible
	bool isValid();

	rational getMinDuration(SpeedProfileType type);

private:
	rational getT1();
	rational getMonotonousT0();
	rational getNonMonotonousT0();
	rational getT0();

	mmPerMillisecond startSpeed;
	mmPerMillisecond endSpeed;
	rational distance;
	rational duration;

};

#endif /* SPEEDPROFILE_H_ */
