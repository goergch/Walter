/*
 * SpeedProfile.h
 *
 * Implementation of a linear and a trapezoidal speed profile.
 * Computed between two points defined by start speed, end speed, distance and to-be
 * duration that is used to move from a to b.
 *
 * Author: JochenAlt
 */

#ifndef SPEEDPROFILE_H_
#define SPEEDPROFILE_H_

#include "setup.h"

class SpeedProfile {
public:
	enum SpeedProfileType {LINEAR, TRAPEZOIDAL };

	SpeedProfile();
	SpeedProfile(const SpeedProfile& par);
	void operator=(const SpeedProfile& par);
	bool isNull();
	void null();

	// compute a trapezoidal speed profile. Depending on startspeed/endspeed/distance,
	// select the appropriate shape and return the duration
	bool computeSpeedProfile(rational& pStartSpeed /* mm/s */, rational& pEndSpeed /* mm/s */, rational pDistance /* mm */, rational& pDuration /* ms */);

	// returns true, if a ramp profile can be achieved with given startspeed/endspeed/distance. If yes, the duration is computed.
	static bool getRampProfileDuration(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational &pDuration);

	// return true, if profile is possible. if invalid, profile is set to null, which is an linear profile
	bool isValid();

	// use a valid speed profile by modifying t=[0..1] such the
	// returned number [0..1] can be used as input parameter
	// representing the position within a trajectory piece.
	rational apply(SpeedProfileType type, rational t);

private:
	static bool computeSpeedProfileImpl(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational& pT0, rational& pT1, rational& pDuration);
	static bool computeRampProfile(const rational pStartSpeed, rational& pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, rational& pDuration);
	static void getLazyRampProfileDuration(const rational pStartSpeed, const rational pEndSpeed, rational pDistance, rational &pDuration);
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
	rational t0;		 // time diff of first phase (negative when going down)
	rational t1;		 // time diff of last phase (also might be negative)
};

#endif /* SPEEDPROFILE_H_ */
