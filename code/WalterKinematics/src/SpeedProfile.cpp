/*
 * SpeedProfile.cpp
 *
 *  Created on: 09.09.2016
 *      Author: JochenAlt
 */

#include "SpeedProfile.h"
#include "util.h"
#include "logger.h"


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
// returns true if end speed has been adapted
bool SpeedProfile::computeRampProfile(const rational pStartSpeed, rational &pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, rational& pDuration) {
	bool endSpeedFine = true;
	// Note: pT1 becomes negative if negative acceleration is to be used
	if (pStartSpeed <= pEndSpeed) {
		// check if endSpeed is possible

		// limit to maximum end speed
		rational maxEndSpeed = sqrt( pStartSpeed*pStartSpeed + 2*pDistance*maxAcceleration_mm_msms);
		if (pEndSpeed > maxEndSpeed+floatPrecision) {
			pEndSpeed = maxEndSpeed;
			endSpeedFine = false;
		}

		pT0= (pEndSpeed - pStartSpeed)/maxAcceleration_mm_msms;
		pT1 = 0;
		pDuration = (pDistance - pStartSpeed*abs(pT0) - 0.5*maxAcceleration_mm_msms*sqr(pT0))/pEndSpeed + abs(pT0);
	} else {
		// limit to minimum end speed by distance
		rational minEndSpeed = sqrt( pStartSpeed*pStartSpeed - 2*pDistance*maxAcceleration_mm_msms);
		if (pEndSpeed < minEndSpeed-floatPrecision) {
			pEndSpeed = minEndSpeed;
			endSpeedFine = false;
		}

		pT0 = 0;
		pT1 = (pStartSpeed - pEndSpeed)/maxAcceleration_mm_msms;
		pDuration = (pDistance - pEndSpeed*abs(pT1) - 0.5*maxAcceleration_mm_msms*sqr(pT1))/pStartSpeed+ abs(pT1);
	}
	return endSpeedFine;
}

bool SpeedProfile::getRampProfileDuration(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational &pDuration) {
	rational t0, t1;
	pDuration = 0;
	bool endSpeedFine = computeRampProfile(pStartSpeed, pEndSpeed, pDistance, t0, t1, pDuration);
	return endSpeedFine;
}

// compute duration if we stay as long as possible at startspeed
void  SpeedProfile::getLazyRampProfileDuration(const rational pStartSpeed, const rational pEndSpeed, rational pDistance, rational &pDuration) {
	// what is the duration that would make t0 = 0 ?
	pDuration = (pDistance - 0.5*sqr(pEndSpeed-pStartSpeed)/maxAcceleration_mm_msms)/pStartSpeed;
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
	rational u = (pStartSpeed-pEndSpeed)/maxAcceleration_mm_msms;
	// abc formula
	rational c = pStartSpeed*pDuration - 0.5*maxAcceleration_mm_msms*sqr(u) - pDistance;
	rational b = maxAcceleration_mm_msms * ( pDuration - u );
	rational a = -maxAcceleration_mm_msms;
	rational t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/maxAcceleration_mm_msms - pT0;
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
	rational u = (pStartSpeed-pEndSpeed)/maxAcceleration_mm_msms;
	// abc formula
	rational c = pStartSpeed*pDuration + 0.5*maxAcceleration_mm_msms*sqr(u) - pDistance;
	rational b = maxAcceleration_mm_msms * ( pDuration + u );
	rational a = maxAcceleration_mm_msms;
	rational t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/maxAcceleration_mm_msms - pT0;
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
	rational u = (pStartSpeed-pEndSpeed)/maxAcceleration_mm_msms;
	// abc formula
	rational c = pEndSpeed*u + 0.5*maxAcceleration_mm_msms*sqr(u) - pDistance;
	rational b = (pStartSpeed + pEndSpeed + maxAcceleration_mm_msms * u);
	rational a = maxAcceleration_mm_msms;
	rational t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/maxAcceleration_mm_msms - pT0;

	// ignore input of duration, overwrite
	pDuration = abs(pT0) + abs(pT1);
	return true;
}

bool SpeedProfile::computePeakDownProfile(const rational pStartSpeed, const rational pEndSpeed, const rational pDistance, rational& pT0, rational& pT1, rational& pDuration) {
	rational u = (pStartSpeed-pEndSpeed)/maxAcceleration_mm_msms;
	// abc formula
	rational c = -pEndSpeed*pDuration + 0.5*maxAcceleration_mm_msms*sqr(u) - pDistance;
	rational b = (pStartSpeed + pEndSpeed + maxAcceleration_mm_msms * u);
	rational a = -maxAcceleration_mm_msms;
	rational t0_1, t0_2;
	bool solutionExists = polynomRoot2ndOrder(a,b,c, t0_1, t0_2);
	if (!solutionExists)
		return false;

	pT0 = t0_1; // in this situation, it is always the first solution
	pT1 = (pEndSpeed - pStartSpeed)/maxAcceleration_mm_msms - pT0;

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
	pT0= (pDistance - pStartSpeed*pDuration - 0.5 * sqr(pEndSpeed - pStartSpeed)/maxAcceleration_mm_msms )
				/
				(maxAcceleration_mm_msms * pDuration + pStartSpeed - pEndSpeed);
	pT1 = (pEndSpeed - pStartSpeed)/maxAcceleration_mm_msms - pT0;
}

bool SpeedProfile::computeSpeedProfileImpl(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational& pT0, rational& pT1, rational& pDuration) {
	if (pStartSpeed <= pEndSpeed) {

	   // compute ramp profile to decide which profile to use
	   rational rampDuration;
	   bool endEndSpeedFine= computeRampProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, rampDuration);

	   if (!endEndSpeedFine) {
		   // even with max acceleration, end speed cannot be reached, it has been modified by computeRampProfile
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
			   // check if stairways or neg. trapezoid by comparing duration with lazy ramp profile
			   // (lazy ramp = ramp that stays as long as possible on startspeed)
			   rational lazyrampDuration;
			   getLazyRampProfileDuration(pStartSpeed, pEndSpeed, pDistance, lazyrampDuration);
			   if (pDuration < lazyrampDuration) {
				   // use stairways profile
				   computeStairwaysProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (!isValidImpl(pStartSpeed, pEndSpeed, pT0,pT1, pDuration, pDistance))
					   LOG(ERROR) << "BUG: stairways profile invalid";
				   return true; // everything fine
			   }
			   else {
				   // use negative trapezoid profile
				   bool solutionExists= computeNegativeTrapezoidProfile(pStartSpeed,pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (solutionExists) {
					   if (!isValidImpl(pStartSpeed, pEndSpeed,pT0,pT1, pDuration, pDistance))
						   LOG(ERROR) << "BUG: negative trapezoid profile invalid";
					   return true; // everything fine
				   } else
					   LOG(ERROR) << "BUG: solution for negative trapezoid profile expected ";
			   }
	   	   } else {
	   		   // fits exactly into a ramp profile. Very unlikely
			   pDuration = rampDuration;
			   return true; // nothing changed
	   	   }
	   }
   } else { // pStartSpeed > pEndspeed
	   // compute ramp profile to decide which profile to use
	   rational rampDuration;
	   bool noEndSpeedAmendment = computeRampProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, rampDuration);

	   if (!noEndSpeedAmendment) {
		   // even with max decceleration, end speed cannot be reached, so amend end speed to minimum
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
	   } else {
		   // we have more time than ramp profile. We need to slow down,
		   // either with stairways or negative trapezoid profile
		   if (pDuration > rampDuration) {
			   // check if stairways or neg. trapezoid by comparing duration with lazy ramp profile
			   // (lazy ramp = ramp that stays as long as possible on startspeed)
			   rational lazyrampDuration;
			   getLazyRampProfileDuration(pStartSpeed, pEndSpeed, pDistance, lazyrampDuration);
			   if (pDuration < lazyrampDuration) {
				   // use stairways profile
				   computeStairwaysProfile(pStartSpeed, pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (!isValidImpl(pStartSpeed, pEndSpeed, pT0,pT1, pDuration, pDistance))
					   LOG(ERROR) << "BUG: stairways profile invalid";
				   return true; // everything fine
			   }
			   else {
				   // use negative trapezoid profile
				   bool solutionExists= computeNegativeTrapezoidProfile(pStartSpeed,pEndSpeed, pDistance, pT0, pT1, pDuration);
				   if (solutionExists) {
					   if (!isValidImpl(pStartSpeed, pEndSpeed,pT0,pT1, pDuration, pDistance))
						   LOG(ERROR) << "BUG: negative trapezoid profile invalid";
					   return true; // everything fine
				   } else
					   LOG(ERROR) << "BUG: solution for negative trapezoid profile expected ";
			   }
	   	   } else {
	   		   // fits exactly into a ramp profile. Very unlikely
			   pDuration = rampDuration;
			   return true; // nothing changed
	   	   }
	   }
   }
   LOG(ERROR) << "BUG:no speed profile found";
   return false;
}



bool SpeedProfile::computeSpeedProfile(rational& pStartSpeed, rational& pEndSpeed, rational pDistance, rational& pDuration) {
	bool possibleWithoutAmendments = computeSpeedProfileImpl(pStartSpeed, pEndSpeed, pDistance, t0, t1, pDuration);
	distance = pDistance;
	duration = pDuration;
	startSpeed = pStartSpeed;
	endSpeed = pEndSpeed;

	if (!isValid())
		LOG(ERROR) << "returned speed profile invalid";
	return possibleWithoutAmendments;
}


rational SpeedProfile::computeDistance(rational pStartSpeed, rational pEndSpeed, rational pT0, rational pT1, rational pDuration) {
	rational middleSpeed = pStartSpeed + maxAcceleration_mm_msms * pT0;
	rational position =
			getDistance(pStartSpeed, sgn(pT0)* maxAcceleration_mm_msms, fabs(pT0))
				+ getDistance(middleSpeed, 0,pDuration - fabs(pT0) - fabs(pT1))
				+ getDistance(middleSpeed, sgn(pT1) * maxAcceleration_mm_msms, fabs(pT1));
	return position;
}

rational SpeedProfile::getDistanceSoFar(rational t0, rational t1, rational t) {
	rational middleSpeed = startSpeed + maxAcceleration_mm_msms * t0;

	rational absT = t*duration;
	rational position = 0;

	rational durationFirstBlock = fabs(t0);
	rational durationThirdBlock = fabs(t1);
	rational durationSecondBlock = duration - durationFirstBlock - durationThirdBlock;

	if (absT < fabs(t0)) {
		// first part, increase speed from startSpeed to middleSpeed with constant acceleration
		rational tInBlock = absT;
		position = getDistance(startSpeed, sgn(t0)* maxAcceleration_mm_msms, tInBlock);
	} else {
		if (absT < (durationFirstBlock+durationSecondBlock)) {
			// second part, constant middleSpeed
			rational tInBlock = absT - durationFirstBlock;
			position = getDistance(startSpeed, sgn(t0)* maxAcceleration_mm_msms, durationFirstBlock)
 					 + getDistance(middleSpeed, 0, tInBlock);
		} else {
			// third part, increase speed from middleSpeed to endSpeed;
			rational tInBlock = absT - durationSecondBlock - durationFirstBlock;
			position = getDistance(startSpeed, sgn(t0)* maxAcceleration_mm_msms, durationFirstBlock)
						+ getDistance(middleSpeed, 0,durationSecondBlock)
						+ getDistance(middleSpeed, sgn(t1) * maxAcceleration_mm_msms, tInBlock);
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
	if (result < floatPrecision)
		result = 0.0;
	if ((result >= 1.0) && (result < 1.0 + floatPrecision))
		result = 1.0;

	if ((result < 0.0) || (result > 1.0))
		LOG(ERROR) << "BUG: speedprofile (" << t << " returns t=" << result;
	return result;
}


