/*
 * SpeedProfile.cpp
 *
 *  Created on: 09.09.2016
 *      Author: JochenAlt
 */

#include "SpeedProfile.h"
#include "util.h"

const mmPerMillisecondPerMillisecond SpeedProfile::acceleration = 0.0005;

rational getDistance(rational startSpeed, rational acc, rational t) {
    rational distance = startSpeed*t + 0.5 * acc * sqr(t);
    return distance;
}

bool SpeedProfile::isValid() {
	return isValidImpl(startSpeed, endSpeed, t0,t1, duration, distance);
}


bool SpeedProfile::isValidImpl(rational pStartSpeed, rational pEndSpeed, rational pT0, rational pT1, rational pDuration, rational pDistance) {
	return almostEqual(computeDistance(pStartSpeed, pEndSpeed, pT0,pT1, pDuration), pDistance,0.01);
}

// the rampup profile starts with startSpeed, immediately accelerates to the end speed and remains there. If
// the end speed is lower than the start speed, the speed remains on the start speed as long as possible, then
// goes down to the end speed with maximum negative acceleration.
//
//    |      |           |      |
//    |  ----|v1       v0|----  |
//    | /    |           |    \ |
// v0 |/     |           |     \|v1
//   -|------|-         -|------|---
//
// if the distance is too short to reach the end speed, the end speed is adapted to the maximum resp.
// minimum possible end speed:
//
//    |      /|vmax    v0|\     |
//    |     / |          | \    |
//    |    /  |          |  \   |
//    |   /   |          |   \  |
//    |  /    |          |    \ |
//    | /     |          |     \|
// v0 |/      |          |      |vmin
//   -|-------|-        -|------|---
//
// returns false, if input parameters have been amended
bool SpeedProfile::computeRampProfile(const rational pStartSpeed, rational &pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, rational& pDuration) {
	bool changedParameters = false;
	// Note: pT1 becomes negative if negative aceleration is to be used
	if (startSpeed < endSpeed) {
		// check if endSpeed is possible

		// limit to maximum end speed
		rational maxEndSpeed = sqrt( pStartSpeed*pStartSpeed + 2*distance*acceleration);
		if (pEndSpeed > maxEndSpeed) {
			pEndSpeed = maxEndSpeed;
			changedParameters = true;
		}

		pT0= (endSpeed - startSpeed)/acceleration;
		pT1 = 0;
		pDuration = (pDistance - startSpeed*abs(pT0) - 0.5*acceleration*sqr(pT0))/endSpeed + abs(pT0);
	} else {
		// limit to minimum end speed
		rational minEndSpeed = sqrt( pEndSpeed*pEndSpeed - 2*distance*acceleration);
		if (pEndSpeed < minEndSpeed) {
			pEndSpeed = minEndSpeed;
			changedParameters = true;
		}

		pT0 = 0;
		pT1 = (startSpeed - endSpeed)/acceleration;
		pDuration = (pDistance - endSpeed*abs(pT1) - 0.5*acceleration*sqr(pT1))/startSpeed+ abs(pT1);
	}
	return changedParameters;
}

// the trapezoid profile starts with startSpeed, immediately accelerates to the middle speed and deccelerates to the
// end speed at the latest point in time such that the given distance is met.
//
//    |       |           |      |
//    |  ---- |         v0| ---  |
//    | /    \|v1         |/   \ |
// v0 |/      |           |     \|v1
//   -|-------|-         -|------|---
//
bool SpeedProfile::computeTrapezoidProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, const rational pDuration) {
	rational u = (pStartSpeed-pEndSpeed)/acceleration;
	// abc formula
	rational c = pStartSpeed*pDuration - 0.5*acceleration*sqr(u) - pDistance;
	rational b = acceleration * ( pDuration - u );
	rational a = -acceleration;
	rational t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/acceleration - pT0;
	return true;
}

// A negative traezoid profile is returned
//
// v0 |       |           |      |
//    |\      |v1         |     /|v1
//    | \    /|         v0|\   / |
//    |  ---  |           |  --  |
//   -|-------|-         -|------|---
//
// returns true, if solution exists
bool SpeedProfile::computeNegativeTrapezoidProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, const rational pDuration) {
	rational u = (pStartSpeed-pEndSpeed)/acceleration;
	// abc formula
	rational c = pStartSpeed*pDuration + 0.5*acceleration*sqr(u) - pDistance;
	rational b = acceleration * ( pDuration + u );
	rational a = -acceleration;
	rational t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/acceleration - pT0;
	return true;
}


// the peak profile starts with startSpeed, immediately accelerates and deccelerates to the
// end speed such that the given distance is met.
//
//    |   /\  |           |     |
//    |  /  \ |         v0|\    |
//    | /    \|v1         | \  /|
// v0 |/      |           |  \/ |v1
//   -|-------|-         -|-----|---
//
//
// returns true if solution exists
bool SpeedProfile::computePeakUpProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, rational& pDuration) {
	rational u = (pStartSpeed-pEndSpeed)/acceleration;
	// abc formula
	rational c = pEndSpeed*u + 0.5*acceleration*sqr(u) - pDistance;
	rational b = (pStartSpeed + pEndSpeed + acceleration * u);
	rational a = acceleration;
	rational t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/acceleration - pT0;

	// ignore input of duration, overwrite
	pDuration = abs(pT0) + abs(pT1);
	return true;
}

bool SpeedProfile::computePeakDownProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, rational& pDuration) {
	rational u = (pStartSpeed-pEndSpeed)/acceleration;
	// abc formula
	rational c = -pEndSpeed*pDuration + 0.5*acceleration*sqr(u) - pDistance;
	rational b = (pStartSpeed + pEndSpeed + acceleration * u);
	rational a = -acceleration;
	rational t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/acceleration - pT0;

	// ignore input of duration, overwrite
	pDuration = abs(pT0) + abs(pT1);
	return true;
}


// the stairways profile starts with startSpeed, immediately accelerates to the middle speed and deccelerates to the
// end speed at the latest point in time such that the given distance is met.
//
//    |      /|v1       v0|\     |
//    |  ---/ |           | \--  |
//    | /     |           |    \ |
// v0 |/      |           |     \|v1
//   -|-------|-         -|------|---
//
void SpeedProfile::computeStairwaysProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, const rational pDuration) {
	pT0= (pDistance - pStartSpeed*pDuration - 0.5 * sqr(pEndSpeed - pStartSpeed)/acceleration )
				/
				(acceleration * pDuration + pStartSpeed - pEndSpeed);
	pT1 = (pEndSpeed - pStartSpeed)/acceleration - pT0;
}

bool SpeedProfile::computeSpeedProfileImpl(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational& pT0, rational& pT1, rational& pDuration) {
	if (pStartSpeed <= pEndSpeed) {

	   // compute ramp profile to decide which profile to use
	   rational rampDuration;
	   bool rampModified = computeRampProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, rampDuration);

	   if (rampModified) {
		   // even with max acceleration, end speed cannot be reached, so amend end speed to maximum
		   pDuration = rampDuration; // overwrite input of duration
		   return false; // adaption was necessary
	   }

	   // if the to-be duration is smaller than the ramp's duration, we need to speed up to a trapezoid profile
	   if (pDuration < rampDuration) {
		   // trapezoid profile is possible if duration is longer than peakDuration
		   rational peakDuration;
		   bool peakExists = computePeakUpProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, peakDuration);
		   if (peakExists) {
			   if (pDuration >= peakDuration) { // trapezoid profile possible
				   bool trapezoidSolutionValid = computeTrapezoidProfile(pStartSpeed,pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (trapezoidSolutionValid) {
					   if (!isValidImpl(pStartSpeed, pEndSpeed, pT0,pT1, pDuration, pDistance))
						   LOG(ERROR) << "BUG: trapezoid profile invalid";
					   return true; // everything fine
				   } else
					   LOG(ERROR) << "BUG: solution for trapezoid profile expected but not found";
			   } else {
				   // trapezoid profile not possible, adapt to peak profile
				   pDuration = peakDuration; // overwrite input of duration
				   return false; // amendment happened
			   }
		   } else
			   LOG(ERROR) << "BUG:peak profile solution expected";
	   } else {  // pDuration >= rampDuration
		   // we have more time than ramp profile. We need to slow down,
		   // either with stairways or negative trapezoid profile
		   if (pDuration > rampDuration) {
			   // check if stairways or neg. trapezoid by comparing duration with lazy ramp profile  (reverse ramp)
			   rational lazyrampEndSpeed = pStartSpeed;
			   rational lazyrampDuration;
			   bool lazyrampModified = computeRampProfile(pEndSpeed, lazyrampEndSpeed, pDistance, pT0, pT1, lazyrampDuration);
			   if (lazyrampModified) {
				   if (pDuration < lazyrampDuration) {
					   // use stairways profile
					   computeStairwaysProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, pDuration);
					   if (!isValidImpl(pStartSpeed, pEndSpeed, pT0,pT1, pDuration, pDistance))
						   LOG(ERROR) << "BUG: stairways profile invalid";
					   return true; // everything fine
				   }
				   else {
					   // use negative trapezoid profile
					   bool negtrapezoidValid= computeNegativeTrapezoidProfile(pStartSpeed,pEndSpeed, pDistance, pT0, pT1, pDuration);
					   if (!negtrapezoidValid) {
						   if (!isValidImpl(pStartSpeed, pEndSpeed,pT0,pT1, pDuration, pDistance))
							   LOG(ERROR) << "BUG: negative trapezoid profile invalid";
						   return true; // everything fine
					   } else
						   LOG(ERROR) << "BUG: solution for negative trapezoid profile expected ";
				   }
			   }
			   else
				   LOG(ERROR) << "BUG: lazy ramp profile solution expected";
	   	   } else {
	   		   // fits exactly into a ramp profile. Very unlikely
			   pDuration = rampDuration;
			   return true; // nothing changed
	   	   }
	   }
   } else { // pStartSpeed > pEndspeed
	   // compute ramp profile to decide which profile to use
	   rational rampEndSpeed = pEndSpeed;
	   rational rampDuration;
	   bool rampModified = computeRampProfile(pStartSpeed, rampEndSpeed, pDistance, pT0, pT1, rampDuration);

	   if (rampModified) {
		   // even with max decceleration, end speed cannot be reached, so amend end speed to minimum
		   pEndSpeed = rampEndSpeed;
		   pDuration = rampDuration; // overwrite input of duration
		   return false; // adaption was necessary
	   }

	   // if the to-be duration is smaller than the ramp's duration, we need to speed up to a trapezoid profile
	   if (pDuration < rampDuration) {
		   // trapezoid profile is possible if duration is longer than peakDuration
		   rational peakDuration;
		   bool peakExists = computePeakDownProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, peakDuration);
		   if (peakExists) {
			   if (pDuration >= peakDuration) { // trapezoid profile possible
				   bool trapezoidSolutionValid = computeNegativeTrapezoidProfile(pStartSpeed,pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (trapezoidSolutionValid) {
					   if (!isValidImpl(pStartSpeed, pEndSpeed, pT0,pT1, pDuration, pDistance))
						   LOG(ERROR) << "BUG: neg trapezoid profile invalid";
					   return true; // everything fine
				   } else
					   LOG(ERROR) << "BUG: solution for negative trapezoid profile expected but not found";
			   } else {
				   // trapezoid profile not possible, adapt to peak profile
				   pDuration = peakDuration; // overwrite input of duration
				   return false; // amendment happened
			   }
		   } else
			   LOG(ERROR) << "BUG:peak down profile solution expected";
	   }

	   // speed profile is possible now. Dont compute it separately but reverse it
	   SpeedProfile reversedProfile;
	   rational startSpeed = pEndSpeed;
	   rational endSpeed = pStartSpeed;
	   rational t0, t1;
	   rational duration = pDuration;
	   bool amended = reversedProfile.computeSpeedProfileImpl(startSpeed, endSpeed, pDistance, t0, t1, duration);
	   if (amended) {
		  LOG(ERROR) << "BUG: no amendmend of reversed profile expected";
	   } else {
		   pT0 = -t1;
		   pT1 = -t0;
		   return true; // no amendmend
	   }
   }
   LOG(ERROR) << "BUG:no speed profile found";
   return false;
}

bool SpeedProfile::computeSpeedProfile(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational& pDuration) {
	distance = pDistance;
	bool result = computeSpeedProfileImpl(pStartSpeed, pEndSpeed, pDistance, t0, t1, pDuration);
	duration = pDuration;
	if (!isValid())
		LOG(ERROR) << "returned speed profile invalid";
	return result;
}


rational SpeedProfile::computeDistance(rational pStartSpeed, rational pEndSpeed, rational pT0, rational pT1, rational pDuration) {
	rational middleSpeed = pStartSpeed + acceleration * pT0;
	rational position =
			getDistance(pStartSpeed, sgn(pT0)* acceleration, fabs(pT0))
				+ getDistance(middleSpeed, 0,pDuration - fabs(pT0) - fabs(pT1))
				+ getDistance(middleSpeed, sgn(pT1) * acceleration, fabs(pT1));
	return position;
}

rational SpeedProfile::getDistanceSoFar(rational t0, rational t1, rational t) {
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

	return position;
}


// compute distance by given speed and acceleration
rational SpeedProfile::get(SpeedProfileType type, rational t) {
	if (isNull() || (type == LINEAR))
		return t;

	rational distanceSoFar = getDistanceSoFar(t0,t1,t);


	rational result = distanceSoFar/distance;
	if ((result<0.0) || (result > 1.0))
		LOG(ERROR) << "BUG: speedprofile (" << t << " returns t=" << result;
	return result;
}


