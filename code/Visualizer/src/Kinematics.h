/*
 * Kinematics.h
 *
 * Most complex part. Does the forward kinematics and the inverse kinematics.
 * By reading code only, this is ununderstandable. Please check the spreadhseet
 * kinematics.xlsx, where I derived all the formulars
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

// a configuration is a valid solution of a given position. There are mathematical 8 solutions
// max., not necessarily all valid all the time. Different solutions can be obtained when
// considering the bot to look forward or backward (hip joint), flipping or not flipping the triangle
// as defined by angles 1,2, and 3; or by considering different the forearm being upwards or downwards.

struct PoseConfigurationType {
	// types or arm position
	enum PoseDirectionType {FRONT, BACK }; /* look to front or to the back (axis 0) */
	enum PoseFlipType{ FLIP, NO_FLIP}; /* elbow axis is above or below */ ;
	enum PoseForearmType{ UP, DOWN}; /* elbow axis is above or below */ ;

	PoseDirectionType poseDirection;
	PoseFlipType poseFlip;
	PoseForearmType poseTurn;
};

// A solution is determined by a configuration and a set of angles
class KinematicsSolutionType {
public:
	KinematicsSolutionType () { angles.resize(NumberOfActuators);};
	KinematicsSolutionType (const KinematicsSolutionType& par) { config = par.config; angles.resize(NumberOfActuators);angles = par.angles; };
	void operator=(const KinematicsSolutionType& par) { config = par.config; angles.resize(NumberOfActuators);angles = par.angles; };

	PoseConfigurationType config;
	JointAngleType angles;
};

// Computation class, doing forward and inverse kinematics
class Kinematics {
public:
	Kinematics();

	static Kinematics& getInstance() {
			static Kinematics instance;
			return instance;
	}

	// call me upfront
	void setup();
	// compute a pose out of joint angles
	void computeForwardKinematics(const JointAngleType angles, Pose& pose);
	// compute joint angles out of a pose. Returns all possible solutions and a recommended one that
	// differs the least from the current bot position
	bool computeInverseKinematics(
			JointAngleType current,
			const Pose& pose, KinematicsSolutionType &solutions, std::vector<KinematicsSolutionType> &validSolution);
	// computes the configuration type of a given solution
	void computeConfiguration(const JointAngleType angles, PoseConfigurationType &config);
	// the gripper is non-linear, so we use kinematics to compensate the non-linearity. This function returns the real hand length depending on the gripper angle
	static float getHandLength(float gripperAngle);
private:
	void computeIKUpperAngles(const Pose& tcp, const JointAngleType& current, PoseConfigurationType::PoseDirectionType poseDirection, PoseConfigurationType::PoseFlipType poseFlip, rational angle0, rational angle1, rational angle2, const HomMatrix &T06,
			KinematicsSolutionType &angles_up, KinematicsSolutionType &angles_down);
	bool isSolutionValid(const Pose& pose, const KinematicsSolutionType& sol, rational &precision);
	bool isIKInBoundaries(const KinematicsSolutionType &sol, int & actuatorOutOfBound);
	bool chooseIKSolution(const JointAngleType& current, const Pose& pose, std::vector<KinematicsSolutionType> &solutions, int &choosenSolution,std::vector<KinematicsSolutionType>& validSolutions);
	void computeInverseKinematicsCandidates(const Pose& pose, const JointAngleType& current, std::vector<KinematicsSolutionType> &solutions);

	void computeDHMatrix(int actuatorNo, rational pTheta, float d, HomMatrix& dh);
	void computeDHMatrix(int actuatorNo, rational pTheta, HomMatrix& dh);

	void computeRotationMatrix(rational x, rational y, rational z, HomMatrix& m);
	void computeInverseRotationMatrix(rational x, rational y, rational z, HomMatrix& m);

	DenavitHardenbergParams DHParams[Actuators];	// DH params of actuators
	HomMatrix hand2View; 							// rotation matrix for rotating the original gripper coord to a handy one that has a zero position of (0,0,0)
	HomMatrix view2Hand; 							// inverse rotation matrix
	HomMatrix hand2toolendpoint; 					// matrix from tcp to tool end point
	HomMatrix toolendpoint2hand; 					// matrix from tool end point to tcp

};


#endif /* KINEMATICS_H_ */
