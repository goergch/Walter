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

struct KinematicConfigurationType {
	// types or arm position
	enum PoseDirectionType {FRONT, BACK }; /* look to front or to the back (axis 0) */
	enum PoseFlipType{ FLIP, NO_FLIP}; /* elbow axis is above or below */ ;
	enum PoseForearmType{ UP, DOWN}; /* elbow axis is above or below */ ;

	PoseDirectionType poseDirection;
	PoseFlipType poseFlip;
	PoseForearmType poseTurn;
};

class KinematicsSolutionType {
public:
	KinematicsSolutionType () { angles.resize(NumberOfActuators);};
	KinematicsSolutionType (const KinematicsSolutionType& par) { config = par.config; angles.resize(NumberOfActuators);angles = par.angles; };
	void operator=(const KinematicsSolutionType& par) { config = par.config; angles.resize(NumberOfActuators);angles = par.angles; };

	KinematicConfigurationType config;
	JointAngleType angles;
};

class Kinematics {
public:
	// inverse kinematics delivers all possible solutions, the right one is
	// selected later on by trajectory planning


	Kinematics();

	static Kinematics& getInstance() {
			static Kinematics instance;
			return instance;
	}

	void setup();
	void computeForwardKinematics(const JointAngleType angles, Pose& pose);
	bool computeInverseKinematics(
			ActuatorLimitsType limits,JointAngleType current,
			const Pose& pose, KinematicsSolutionType &solutions, std::vector<KinematicsSolutionType> &validSolution);
	void computeConfiguration(const JointAngleType angles, KinematicConfigurationType &config);
	static float getHandLength(float angle);

private:

	void computeIKUpperAngles(const Pose& tcp, KinematicConfigurationType::PoseDirectionType poseDirection, KinematicConfigurationType::PoseFlipType poseFlip, rational angle0, rational angle1, rational angle2, const HomMatrix &T06,
			KinematicsSolutionType &angles_up, KinematicsSolutionType &angles_down);
	bool isSolutionValid(const Pose& pose, const KinematicsSolutionType& sol);
	bool isIKInBoundaries(ActuatorLimitsType limits, const KinematicsSolutionType &sol);
	bool chooseIKSolution(ActuatorLimitsType limits, JointAngleType current, const Pose& pose, std::vector<KinematicsSolutionType> &solutions, int &choosenSolution,std::vector<KinematicsSolutionType>& validSolutions);
	void computeInverseKinematicsCandidates(const Pose& pose, std::vector<KinematicsSolutionType> &solutions);

	void computeDHMatrixImpl(int actuatorNo, rational pTheta, float d, HomMatrix& dh);
	void computeDHMatrix(int actuatorNo, rational pTheta, HomMatrix& dh);
	void computeDHMatrixGripper(int actuatorNo, rational pTheta, rational gripperAngle, HomMatrix& dh);

	void logSolution(string prefix, const KinematicsSolutionType& sol_up);

	DenavitHardenbergParams DHParams[Actuators];
	bool isSetup;
};


#endif /* KINEMATICS_H_ */
