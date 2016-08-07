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
#include "spatial.h"
#include "DenavitHardenbergParam.h"

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
