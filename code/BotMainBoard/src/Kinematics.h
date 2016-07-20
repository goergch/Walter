/*
 * Kinematics.h
 *
 *  Created on: 27.06.2016
 *      Author: JochenAlt
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_


#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#include "cmatrix"
#pragma GCC diagnostic pop

typedef techsoft::matrix<float>  fMatrix;
typedef techsoft::matrix<float>  HomMatrix;
typedef std::valarray<float> HomVector;
typedef std::valarray<float> JointAngleType;


// Kinematics constants
const int Actuators = 6;
const float HipHeight = 300;
const float UpperArmLength = 300;
const float ForearmLength = 200;
const float WristLength  = 50;

#include "Util.h"

class Pose {
	public:
		Pose() {
			orientation = { 0,0,0,1.0};
			position = { 0,0,0,1.0};
		};
		Pose(Pose& pose) {
			position = pose.position;
			position = pose.orientation;
		};
		Pose(HomVector& pPosition, HomVector& pOrientation) {
			position = pPosition;
			position = pOrientation;
		};

		void operator= (const Pose& pose) {
			position = pose.position;
			orientation = pose.orientation;
		}

	HomVector position;
	HomVector orientation;
};


// Class that represents Denavit-Hardenberg parameters. Precomputes sin/cos that will be required
class DenavitHardenbergParams{
public:
	DenavitHardenbergParams() {};
	DenavitHardenbergParams(const float pAlpha, const float pA, const float pD) {
		init(pAlpha, pA, pD);
	};

	void init(const float pAlpha, const float pA, const float pD) {
		a = pA;
		d = pD;
		alpha = pAlpha;

		ca = cos(alpha);
		if (almostEqual(ca,0))
			ca = 0;
		if (almostEqual(ca,1))
			ca = 1.0;
		if (almostEqual(ca,-1))
			ca = -1.0;

		sa = sin(alpha);
		if (almostEqual(sa,0))
			sa = 0;
		if (almostEqual(sa,1))
			sa = 1.0;
		if (almostEqual(sa,-1))
			sa = -1.0;


	};

	float a;
	float d;
	float alpha;

	float ca;
	float sa;
};

class Kinematics {
public:
	// inverse kinematics delivers all possible solutions, the right one is
	// selected later on by trajectory planning
	struct IKSolutionType {
		// types or arm position
		enum PoseDirectionType {FRONT, BACK }; /* look to front or to the back (axis 0) */
		enum PoseFlipType{ FLIP, NO_FLIP}; /* elbow axis is above or below */ ;

		PoseDirectionType poseDirection;
		PoseFlipType poseFlip;

		JointAngleType angles;
	};

	Kinematics();

	static Kinematics& getInstance() {
			static Kinematics instance;
			return instance;
	}

	void setup();
	void computeForwardKinematics(const JointAngleType angles, Pose& pose);
	void computeInverseKinematics(const Pose& pose, std::vector<IKSolutionType> &solutions);

private:
	static void computeDHMatrix(const DenavitHardenbergParams& DHparams, float pTheta, HomMatrix& dh);
	DenavitHardenbergParams DHParams[Actuators];
};

#endif /* KINEMATICS_H_ */
