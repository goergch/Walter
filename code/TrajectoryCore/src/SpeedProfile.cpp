/*
 * SpeedProfile.cpp
 *
 *  Created on: 09.09.2016
 *      Author: JochenAlt
 */

#include "SpeedProfile.h"
#include "util.h"


void SpeedProfile::set(mmPerMillisecond pStartSpeed, mmPerMillisecond pEndSpeed, millimeter pDistance, rational pDuration) {
	startSpeed = pStartSpeed;
	endSpeed = pEndSpeed;
	distance = pDistance;
	duration = pDuration;
}

bool SpeedProfile::isValid() {
	return (!isNull() && (fabs(getT0()) < (duration - fabs(getT1()))));
}

rational SpeedProfile::getT0() {
	rational t0 = (distance - startSpeed*duration - 0.5 * sqr(endSpeed - startSpeed)/acceleration )
			/
			(acceleration * duration + startSpeed - endSpeed);
	return t0;
}

rational SpeedProfile::getT1() {
	rational t1 = (endSpeed - startSpeed)/acceleration - getT0();
	return t1;
}

// compute distance by given speed and acceleration
rational getDistance(rational startSpeed, rational acc, rational t) {
	rational distance = startSpeed*t + 0.5 * acc * sqr(t);
	return distance;
}

rational SpeedProfile::get(SpeedProfileType type, rational t) {
	if (isNull() || (type == LINEAR))
		return t;

	rational t0 = getT0();
	rational t1 = getT1();

	rational middleSpeed = startSpeed + acceleration * t0;

	rational absT = t*duration;
	rational position = 0;

	rational durationFirstBlock = fabs(t0);
	rational durationThirdBlock = fabs(t1);
	rational durationSecondBlock = duration - durationFirstBlock - durationThirdBlock;

	if (absT < fabs(t0)) {
		// first part, increase speed from startSpeed to middleSpeed with constant acceleration
		rational tInBlock = absT;
		position = getDistance(startSpeed, sgn(t0)* acceleration, tInBlock);
	} else {
		if (absT < (durationFirstBlock+durationSecondBlock)) {
			// second part, constant middleSpeed
			rational tInBlock = absT - durationFirstBlock;
			position = getDistance(startSpeed, sgn(t0)* acceleration, durationFirstBlock)
 					 + getDistance(middleSpeed, 0, tInBlock);
		} else {
			// third part, increase speed from middleSpeed to endSpeed;
			rational tInBlock = absT - durationSecondBlock - durationFirstBlock;
			position = getDistance(startSpeed, sgn(t0)* acceleration, durationFirstBlock)
						+ getDistance(middleSpeed, 0,durationSecondBlock)
						+ getDistance(middleSpeed, sgn(t1) * acceleration, tInBlock);
		}
	}

	return position/distance;
}
