/*
 * BezierCurve.h
 *
 * Implementation of a cubic bezier curve (which looks really nice)
 *
 * Author: JochenAlt
 */

#ifndef BEZIERCURVE_H_
#define BEZIERCURVE_H_

#include "BezierCurve.h"
#include "spatial.h"
#include "Util.h"
#include "SpeedProfile.h"

class BezierCurve  {
	public:
		BezierCurve();
		BezierCurve(const BezierCurve& par);
		void operator=(const BezierCurve& par);
		void reset();

		TrajectoryNode& getStart();
		TrajectoryNode& getEnd();

		void set(TrajectoryNode& pPrev, TrajectoryNode& pA, TrajectoryNode& pB, TrajectoryNode& pNext);
		float curveLength();
		Pose getSupportPoint(InterpolationType interpType, const TrajectoryNode& a, const TrajectoryNode& b, const TrajectoryNode& c);
		TrajectoryNode getCurrent(float t);
		float distance(float dT1, float dT2);
		TrajectoryNode  getPointOfLine(unsigned long time);
		void amend(float t, TrajectoryNode& pB, TrajectoryNode& pNext);

		void patchB(const TrajectoryNode& pB, const TrajectoryNode& pSupportB) {
			supportB = pSupportB.pose;
			b = pB;
		}

	private:
		float computeBezier(InterpolationType ipType,float a,float supportA,  float b, float supportB, float t);
		TrajectoryNode computeBezier(InterpolationType ipType, const TrajectoryNode& a, const TrajectoryNode& supportA,  const TrajectoryNode& b, const TrajectoryNode& supportB, float t);
		Pose computeBezier(InterpolationType ipType, const Pose& a, const Pose& supportA,  const Pose& b, const Pose& supportB, float t);

		TrajectoryNode a;
		Pose supportA;
		TrajectoryNode b;
		Pose supportB;
};

#endif /* BEZIERCURVE_H_ */
