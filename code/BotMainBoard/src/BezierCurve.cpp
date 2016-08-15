/*
 * BezierCurve.cpp
 *
 *  Created on: 26.02.2015
 *      Author: JochenAlt
 */

#include <iostream>
#include <string>
#include "setup.h"
#include "Util.h"
#include "BezierCurve.h"
#include "Kinematics.h"
#include "Trajectory.h"

// the support points in our cubic bezier curves are at one third of the length of the interpolated distance
#define BEZIER_CURVE_SUPPORT_POINT_SCALE (1.0/3.0)

bool useDynamicBezierSupportPoint = true;
bool useLinearOrientation = true;


float BezierCurve::computeBezier(InterpolationType ipType, float a, float supportA,  float b, float supportB, float t) {
	if ((t>1.0) || (t<0.0)) {
		LOG(ERROR) << "BUG t!=[0..1]:" << t;
	}

	if (ipType == LINEAR)
		return (1-t)*a + t*b;
	else
		return (1-t)*(1-t)*(1-t)*a + 3*t*(1-t)*(1-t)*supportA + 3*t*t*(1-t)*supportB + t*t*t*b;
}


Pose BezierCurve::computeBezier(InterpolationType ipType, const Pose& a, const Pose& supportA,  const Pose& b, const Pose& supportB, float t) {
	Pose result;
	for (int i = 0;i<3;i++)
		result.position[i] = computeBezier(ipType,a.position[i], supportA.position[i], b.position[i], supportB.position[i],t);

	if (useLinearOrientation) {
		for (int i = 0;i<3;i++)
			result.orientation[i] = computeBezier(LINEAR,a.orientation[i], supportA.orientation[i], b.orientation[i], supportB.orientation[i],t);
		result.gripperAngle = computeBezier(LINEAR,a.gripperAngle, supportA.gripperAngle, b.gripperAngle, supportB.gripperAngle,t);
	} else {
		for (int i = 0;i<3;i++)
			result.orientation[i] = computeBezier(ipType,a.orientation[i], supportA.orientation[i], b.orientation[i], supportB.orientation[i],t);
		result.gripperAngle = computeBezier(ipType,a.gripperAngle, supportA.gripperAngle, b.gripperAngle, supportB.gripperAngle,t);
	}

	return result;
}

void BezierCurve::set(TrajectoryNode& pPrev, TrajectoryNode& pA, TrajectoryNode& pB, TrajectoryNode& pNext) {
	supportB = pB.pose;
	if (!pNext.isNull()) {
		supportB =  getSupportPoint(pA,pB,pNext);
	}

	supportA = pA.pose;
	if (!pPrev.isNull()) {
		supportA =  getSupportPoint(pB,pA,pPrev);
	}

	a = pA;
	b = pB;
}

// compute b's support point
Pose  BezierCurve::getSupportPoint(const TrajectoryNode& a, const TrajectoryNode& b, const TrajectoryNode& c) {
	// support point for bezier curve is computed by
	// BC' = mirror BC at B with length(BC') = length(AB)

	// mirror C at B = mirrorC
	Point mirroredC(c.pose.position);
	mirroredC.mirrorAt(b.pose.position);

	// compute length of BmirrorC and AB and translate point along BmirrorC such that its length equals len(AB)
	float lenBmC = mirroredC.distance(b.pose.position);
	float lenAB = a.pose.position.distance(b.pose.position);
	Point mirroredNormedC (b.pose.position);
	Point t  = b.pose.position - mirroredC;

	if (lenBmC > 1 ) { // [mm]
		t *=lenAB/lenBmC;
		mirroredNormedC  -= t;
	}// otherwise mirroredNormedC equals b (at least it is very close)
	// compute the middle point of A and mirrored C
	Point midOfA_mC = (a.pose.position + mirroredNormedC) * 0.5;

	// if the speed of the current and the next piece is the same, take 1/3 as support point distance
	// if the speed is doubled, take the 0.66 as support point
	// if the speed is halfed, take end point as support point

	rational speedAB = 0, speedBC = 0;
	if (b.time_ms != a.time_ms)
		speedAB = lenAB/(b.time_ms-a.time_ms);
	rational lenBC = a.pose.distance(b.pose);
	if (c.time_ms != b.time_ms)
		speedBC = lenBC/(c.time_ms-b.time_ms);

	rational ratioBCcomparedToAB = 1.0;
	if (useDynamicBezierSupportPoint) {
		if (abs(speedAB) > 1.0) {
			ratioBCcomparedToAB = speedBC/speedAB;
			ratioBCcomparedToAB = constrain(ratioBCcomparedToAB,0.2,2.0);
		}
	}

	// now move the point towards B such that its length is like BEZIER_CURVE_SUPPORT_POINT_SCALE
	t = b.pose.position - midOfA_mC;
	rational lent = midOfA_mC.distance(b.pose.position);
	if (lent > 1)
		t *= BEZIER_CURVE_SUPPORT_POINT_SCALE*ratioBCcomparedToAB*lenAB/lent;
	else {
		t.null();
	}
	Pose supportB;
	supportB.position = b.pose.position - t;

	// all this is done for the position only,
	// the rotation becomes the point itself (dont have a good model yet for beziercurves of orientations)
	supportB.orientation = b.pose.orientation;


	return supportB;
}

TrajectoryNode BezierCurve::getCurrent(float t) {
	InterpolationType interpolType = a.interpolationType;
	TrajectoryNode result;
	result.pose = computeBezier(interpolType,a.pose,supportA,b.pose, supportB, t);
	result.time_ms= a.time_ms + t*(b.time_ms-a.time_ms);
	return result;
}


float BezierCurve::distance(float dT1, float dT2) {
	TrajectoryNode last = getCurrent(dT1);
	TrajectoryNode prev = getCurrent(dT2);
	return last.pose.distance(prev.pose);
}

// return a number 0..1 representing the position of t between a and b
float intervalRatio(unsigned long a, unsigned long t, unsigned long b) {
	return ((float)t-(float)a)/((float)b-(float)a);
}

TrajectoryNode BezierCurve::getPointOfLine(unsigned long time) {
	float t = intervalRatio(a.time_ms,time, b.time_ms);
	TrajectoryNode result = getCurrent(t);
	return result;
}


void BezierCurve::amend(float t, TrajectoryNode& pNewB, TrajectoryNode& pNext) {

	// compute current and next curve point. This is used later on to compute
	// the current bezier support point which has the same derivation, assuming
	// that we start from the current position
	static float dT = 0.01;										// arbitrary value, needed to compute the current support points
	TrajectoryNode current = getCurrent(t);						// compute curve point for t(which is now)
	TrajectoryNode currentPoint_plus_dT = getCurrent(t + dT);	// and for t+dT (which is an arbitrary point in time)

	// since we already passed a small piece of t, the next interval is now smaller (actually 1-t)
	// Later on the need dT that has the same distance but relatively to the (shorter) remaining piece
	static float dTNew = dT/(1.0-t);	// compute new dt assuming that we start from current position

	// now we set the new bezier curve. The current position becomes the new starting point
	// and the end point and end support point remains the same. But we need a new support point,
	// which we get out of the current tangent of the current and next, enhanced to the new time frame dTNew
	//
	// the following equation is derived out of the condition
	// 		Bezier (current, current-support,end,end-support, dT) = currentPoint-for-dT
	//
	// out of that, we separate the Bezier terms
	// 		Bezier(0,1,0,0,dT)*supportA = currentPoint-for-dT - Bezier(current,0,end,end-support)
	//
	// and end up in the equation
	// 		supportA = (currentPoint-for-dTNew - Bezier(current,0,end,end-support, dTNew) / Bezier(0,1,0,0,dTNew)
	float supportABezierTermRezi = 1.0/(computeBezier(CUBIC_BEZIER, 0, 1, 0,0,dTNew)); // take the bezier term of support a only

	Pose newSupportA =
			(currentPoint_plus_dT.pose - computeBezier(CUBIC_BEZIER, current.pose, Pose(), b.pose, supportB, dTNew))*supportABezierTermRezi;

	// set the new curve
	Pose newSupportPointB;
	if (pNext.isNull())
		newSupportPointB = pNewB.pose;
	else
		newSupportPointB =  getSupportPoint(current,pNewB,pNext);

	a = current; // this sets current time as new point in time as well
	a.interpolationType = pNewB.interpolationType;
	b = pNewB;
	b.interpolationType = pNewB.interpolationType;

	supportB = newSupportPointB;
	supportA = newSupportA;

	// alternative with small jump in curve
	// set(a,currentTensor, pNewB, pNext);
}



