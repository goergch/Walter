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

	bool operator==(const PoseConfigurationType par) {
		return ((poseDirection == par.poseDirection) &&
				(poseFlip== par.poseFlip) &&
				(poseTurn == par.poseTurn));
	}
	bool operator!=(const PoseConfigurationType par) {
		return (!((*this) == par));
	}
};

// A solution is determined by a configuration and a set of angles
class KinematicsSolutionType {
public:
	KinematicsSolutionType () {};
	KinematicsSolutionType (const KinematicsSolutionType& par) { config = par.config;angles = par.angles; };
	void operator=(const KinematicsSolutionType& par) { config = par.config;angles = par.angles; };

	PoseConfigurationType config;
	JointAngles angles;
};

// Computation class, doing forward and inverse kinematics
class Kinematics {
public:
	Kinematics();

	static Kinematics& getInstance() {
			static Kinematics instance;
			return instance;
	}

	static JointAngles getNullPositionAngles();

	// call me upfront
	void setup();

	// compute a pose out of joint angles
	void computeForwardKinematics(Pose& pose);

	// compute joint angles out of a pose. Returns all possible solutions and a recommended one that
	// differs the least from the current bot position
	bool computeInverseKinematics(
			const Pose& pose, KinematicsSolutionType &solutions, std::vector<KinematicsSolutionType> &validSolution);

	// short form of inverse kinematics, simpy set the angles corresponding to the pose, assume that
	// the currently set angles represent the current position (necessary for choosing the best solution)
	bool computeInverseKinematics(Pose& pose);

	// computes the configuration type of a given solution
	static PoseConfigurationType computeConfiguration(const JointAngles angles);

	// the gripper is non-linear, so we use kinematics to compensate the non-linearity. This function returns the real hand length depending on the gripper angle
	static float getHandLength(float gripperAngle);

	// functions for speed and acceleration
	static float anglesDistance(const JointAngles& angleSet1, const JointAngles& angleSet2);
	static float getAngularAcceleration(rational angle1, rational angle2, rational angle3, int timeDiff_ms);
	static float getAngularSpeed(rational angle1, rational angle2, int timeDiff_ms);

	// returns percentage of speed compared with maximum speed of actuator
	static float maxSpeed(const JointAngles& angleSet1, const JointAngles& angleSet2, int timeDiff_ms, int& jointNo);

	// returns percentage of acceleration compared with maximum acceleration of actuator
	static float maxAcceleration(const JointAngles& angleSet1, const JointAngles& angleSet2,  const JointAngles& angleSet3, int timeDiff_ms,int& jointNo);

	// set the relative deviation of the handview coordinate system, i.e. the central point the
	// gripper moves around when using nick/roll/yaw
	void setHandviewCoordinates(Point relativeDevitationFromTCP);

	// get was has been set by setHandviewCoordinates
	Point getHandviewCoordinates();

private:
	void computeIKUpperAngles(const Pose& tcp, const JointAngles& current, PoseConfigurationType::PoseDirectionType poseDirection, PoseConfigurationType::PoseFlipType poseFlip, rational angle0, rational angle1, rational angle2, const HomMatrix &T06,
			KinematicsSolutionType &angles_up, KinematicsSolutionType &angles_down);
	bool isSolutionValid(const Pose& pose, const KinematicsSolutionType& sol, rational &precision);
	bool isIKInBoundaries(const KinematicsSolutionType &sol, int & actuatorOutOfBound);
	bool chooseIKSolution(const JointAngles& current, const Pose& pose, std::vector<KinematicsSolutionType> &solutions, int &choosenSolution,std::vector<KinematicsSolutionType>& validSolutions);
	void computeInverseKinematicsCandidates(const Pose& pose, const JointAngles& current, std::vector<KinematicsSolutionType> &solutions);

	void computeDHMatrix(int actuatorNo, rational pTheta, float d, HomMatrix& dh);
	void computeDHMatrix(int actuatorNo, rational pTheta, HomMatrix& dh);

	void computeRotationMatrix(rational x, rational y, rational z, HomMatrix& m);
	void computeInverseRotationMatrix(rational x, rational y, rational z, HomMatrix& m);

	DenavitHardenbergParams DHParams[NumberOfActuators]; 	// DH params of actuators
	HomMatrix hand2View; 									// rotation matrix for rotating the original gripper coord to a handy one that has a zero position of (0,0,0)
	HomMatrix view2Hand; 									// inverse rotation matrix
};


#endif /* KINEMATICS_H_ */
