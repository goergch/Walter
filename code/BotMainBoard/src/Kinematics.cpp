/*
 * Kinematics.cpp
 *
 *  Created on: 27.06.2016
 *      Author: JochenAlt
 */

#include "Kinematics.h"
#include "Util.h"

Kinematics::Kinematics() {

}


void Kinematics::setup() {

	// define and precompute Denavit Hardenberg Parameter
	// check Kinematics.xls with kinematics documentation
	DHParams[0] = DenavitHardenbergParams(radians(-90.0), 	0, 				HipHeight);
	DHParams[1] = DenavitHardenbergParams(0, 			  	UpperArmLength, 0);
	DHParams[2] = DenavitHardenbergParams(radians(-90.0), 	0, 				0);
	DHParams[3] = DenavitHardenbergParams(radians(90.0), 	0, 				ForearmLength);
	DHParams[4] = DenavitHardenbergParams(radians(-90.0), 	0, 				0);
	DHParams[5] = DenavitHardenbergParams(0, 				0, 				WristLength);
}


// use DenavitHardenberg parameter and compute the Dh-Transformation matrix with a given joint angle (theta)
void Kinematics::computeDHMatrix(const DenavitHardenbergParams& DHparams, float pTheta, HomMatrix& dh) {

	float ct = cos(pTheta);
	float st = sin(pTheta);

	float a = DHparams.a;
	float d = DHparams.d;
	float sa = DHparams.sa; // = sin (alpha)
	float ca = DHparams.ca; // = cos (alpha)


	dh = HomMatrix(4,4,
			{ ct, 	-st*ca,  st*sa,  a*ct,
			  st, 	 ct*ca, -ct*sa,	 a*st,
			  0,	 sa,		ca,		d,
			  0,	 0,		     0,		1});

}


// compute forward kinematics, i.e. by given joint angles compute the
// position and orientation of the gripper center
void Kinematics::computeForwardKinematics(const JointAngleType pAngle, Pose& pose ) {
	LOG(DEBUG) << "computeForwardKinematics (" << setprecision(1)
			<< pAngle[0] << ", " << pAngle[1] << ", " << pAngle[2] << ", "
			<< pAngle[3] << ", " << pAngle[4] << ", " << pAngle[5] << ")";

	// convert angles first
	float angle[Actuators];
	angle[0] = pAngle[0];
	angle[1] = pAngle[1]-radians(90);
	angle[2] = pAngle[2];
	angle[3] = pAngle[3];
	angle[4] = pAngle[4];
	angle[5] = pAngle[5];

	HomMatrix current;
	computeDHMatrix(DHParams[0], angle[0], current);
	LOG(DEBUG) << "current " << endl << setprecision(4) << current;

	for (int i = 1;i<=5;i++) {
		HomMatrix currDHMatrix;
		computeDHMatrix(DHParams[i], angle[i], currDHMatrix);

		current *= currDHMatrix;
	}

	LOG(DEBUG) << "forward transformation" << endl << setprecision(4) << current;

	// position of hand is given by last row of transformation matrix
	pose.position = current.column(3);

	// compute orientations out of homogeneous transformation matrix
	// (as given in https://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel)
	float beta = atan2(-current[2][0], sqrt(current[0][0]*current[0][0] + current[1][0]*current[1][0]));
	float gamma = 0;
	float alpha = 0;
	if (almostEqual(beta, HALF_PI)) {
		alpha = 0;
		gamma = atan2(current[0][1], current[1][1]);
	} else {
			if (almostEqual(beta, -HALF_PI)) {
				alpha = 0;
				gamma = -atan2(current[0][1], current[1][1]);
			} else {
				alpha = atan2(current[1][0],current[0][0]);
				gamma = atan2(current[2][1], current[2][2]);
			}
	}

	// assign to return param
	pose.orientation[0] = gamma;
	pose.orientation[1] = beta;
	pose.orientation[2] = alpha;

	LOG(DEBUG) << "pose position" << endl << setprecision(4)
			<< "{p=(" << pose.position[0] << "," << pose.position[1] << "," << pose.position[2] << ");("
			<< pose.orientation[0] << "," << pose.orientation[1] << "," << pose.orientation[2] << ")}";

}


// compute reverse kinematics, i.e. by position and orientation compute the
// angles of the joints
void Kinematics::computeInverseKinematics(const Pose& tcp, std::vector<IKSolutionType> &solutions) {
	LOG(DEBUG) << "computeReverseKinematics(" << endl << setprecision(4)
			<< "{p=(" << tcp.position[0] << "," << tcp.position[1] << "," << tcp.position[2] << ");("
			<< tcp.orientation[0] << "," << tcp.orientation[1] << "," << tcp.orientation[2] << ")})";


	// compute base angle by wrist position
	// we have two possible solutions, looking forward and looking backward
	// depending on the sign of the tcp x-coordinate, we assign the two solutions to
	// the forward-angle and the backward-angle
	float angle0_solution1 = atan2(tcp.position[1], tcp.position[0]);
	float angle0_solution2 = atan2(-tcp.position[1], -tcp.position[0]);
	float angle0_forward = 0;
	float angle0_backward = 0;

	if (tcp.position[0] >= 0) {
		angle0_forward =  angle0_solution1;
		angle0_backward = angle0_solution2;
	} else {
		angle0_forward =  angle0_solution2;
		angle0_backward = angle0_solution1;
	}

	LOG(DEBUG) << "angle0_forward=" << angle0_forward << " angle0_backward=" << angle0_backward;


	//
	/*
	LOG(DEBUG) << "computeReverseKinematics (" << setprecision(1)
			<< pAngle[0] << ", " << pAngle[1] << ", " << pAngle[2] << ", "
			<< pAngle[3] << ", " << pAngle[4] << ", " << pAngle[5] << ")";
			*/
}
