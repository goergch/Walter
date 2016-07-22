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

#include "Util.h"

using techsoft::mslice;

typedef techsoft::matrix<rational>  Matrix;
typedef techsoft::matrix<rational>  HomMatrix;
typedef std::valarray<rational> HomVector;
typedef std::valarray<rational> Vector;
typedef std::valarray<rational> JointAngleType;


// Kinematics constants
const int Actuators = 6;
const rational HipHeight = 300;
const rational UpperArmLength = 300;
const rational ForearmLength = 200;
const rational WristLength  = 50;


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
	DenavitHardenbergParams(const rational  pAlpha, const rational pA, const rational pD) {
		init(pAlpha, pA, pD);
	};

	void init(const rational pAlpha, const rational pA, const rational pD);

	rational a;
	rational d;
	rational alpha;

	rational ca;
	rational sa;
};

class Kinematics {
public:
	// inverse kinematics delivers all possible solutions, the right one is
	// selected later on by trajectory planning
	struct IKSolutionType {
		// types or arm position
		enum PoseDirectionType {FRONT, BACK }; /* look to front or to the back (axis 0) */
		enum PoseFlipType{ FLIP, NO_FLIP}; /* elbow axis is above or below */ ;
		enum PoseForearmType{ UP, DOWN}; /* elbow axis is above or below */ ;

		PoseDirectionType poseDirection;
		PoseFlipType poseFlip;
		PoseForearmType poseTurn;

		JointAngleType angles;
	};

	Kinematics();

	static Kinematics& getInstance() {
			static Kinematics instance;
			return instance;
	}

	void setup();
	void computeForwardKinematics(const JointAngleType angles, Pose& pose);
	bool computeInverseKinematics(const std::vector<ActuatorStateType>& current, const Pose& pose, IKSolutionType &solutions);
private:

	void computeIKUpperAngles(IKSolutionType::PoseDirectionType poseDirection, IKSolutionType::PoseFlipType poseFlip, rational angle0, rational angle1, rational angle2, const HomMatrix &T06,
			IKSolutionType &angles_up, IKSolutionType &angles_down);
	bool isIKValid(const Pose& pose, const IKSolutionType& sol);
	bool isIKInBoundaries(const std::vector<ActuatorStateType> &boundaries, const IKSolutionType &sol);
	bool chooseIKSolution(const std::vector<ActuatorStateType>& current, const Pose& pose, std::vector<IKSolutionType> &solutions, int &choosenSolution);
	void computeInverseKinematicsCandidates(const Pose& pose, std::vector<IKSolutionType> &solutions);

	static void computeDHMatrix(const DenavitHardenbergParams& DHparams, rational pTheta, HomMatrix& dh);
	DenavitHardenbergParams DHParams[Actuators];
};

#endif /* KINEMATICS_H_ */
