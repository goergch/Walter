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

	static const mmPerMillisecondPerMillisecond acceleration;

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
		t0 = par.t0;
		t1 = par.t1;

	}

	bool isNull() {
		return ((distance == 0) && (duration == 0));
	}
	void null() {
		startSpeed = 0;
		endSpeed = 0;
		distance = 0.0;
		duration = 0.0;
		t0= 0.0;
		t1= 0.0;
	}

	bool computeSpeedProfile(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational& pDuration);
	static bool getRampProfileDuration(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational &pDuration);

	// set parameters necessary for speed profile.
	bool computeSpeedProfileImpl(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational& pT0, rational& pT1, rational& pDuration);

	// computes the adapted parameter t=[0..1] implementing the speed profile. Returns [0..1]
	rational get(SpeedProfileType type, rational t);

	// return true, if profile is possible. if invalid, profile is set to null, which is an linear profile
	bool isValid();

private:
	static bool computeRampProfile(const rational pStartSpeed, rational& pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, rational& pDuration);
	static void computeStairwaysProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, const rational pDuration);
	static bool computeTrapezoidProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, const rational pDuration);
	static bool computeNegativeTrapezoidProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, const rational pDuration);
	static bool computePeakUpProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, rational& pDuration);
	static bool computePeakDownProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, rational& pDuration);

	static rational computeDistance(rational pStartSpeed, rational pEndSpeed, rational pT0, rational pT1, rational pDuration);
	rational getDistanceSoFar(rational t0, rational t1, rational t);
	static bool isValidImpl(rational pStartSpeed, rational pEndSpeed, rational pT0, rational pT1, rational pDuration, rational pDistance);


	mmPerMillisecond startSpeed;
	mmPerMillisecond endSpeed;
	rational distance;
	rational duration;
	rational t0;
	rational t1;
};

#endif /* SPEEDPROFILE_H_ */
