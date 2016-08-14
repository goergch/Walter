/*
 * BezierCurve.h
 *
 *  Created on: 26.02.2015
 *      Author: JochenAlt
 */

#ifndef BEZIERCURVE_H_
#define BEZIERCURVE_H_

#include "BezierCurve.h"
#include "spatial.h"
#include "Util.h"

class BezierCurve  {
	public:
		BezierCurve() {
		};
		BezierCurve(const BezierCurve& par) {
			a = par.a;
			supportA = par.supportA;
			b = par.b;
			supportB = par.supportB;
		};
		void operator=(const BezierCurve& par) {
			a = par.a;
			supportA = par.supportA;
			b = par.b;
			supportB = par.supportB;
		};


		void reset() {
			a.null();
			b.null();
			supportA.null();
			supportB.null();
		}

		Pose& getStart() { return a.pose; };
		Pose& getEnd() { return b.pose; };

		void set(TrajectoryNode& pPrev, TrajectoryNode& pA, TrajectoryNode& pB, TrajectoryNode& pNext);
		Pose getSupportPoint(const TrajectoryNode& a, const TrajectoryNode& b, const TrajectoryNode& c);
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
		Pose computeBezier(InterpolationType ipType, const Pose& a, const Pose& supportA,  const Pose& b, const Pose& supportB, float t);

		TrajectoryNode a;
		Pose supportA;
		TrajectoryNode b;
		Pose supportB;
};

#endif /* BEZIERCURVE_H_ */
