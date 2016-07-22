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
void Kinematics::computeDHMatrix(const DenavitHardenbergParams& DHparams, rational pTheta, HomMatrix& dh) {

	rational ct = cos(pTheta);
	rational st = sin(pTheta);

	rational a = DHparams.a;
	rational d = DHparams.d;
	rational sa  = DHparams.sa; // = sin (alpha)
	rational ca = DHparams.ca; // = cos (alpha)


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
	rational angle[Actuators];
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
	rational beta = atan2(-current[2][0], sqrt(current[0][0]*current[0][0] + current[1][0]*current[1][0]));
	rational gamma = 0;
	rational alpha = 0;
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


	// 1. Step compute angle0 (base)
	// - compute Transformationsmatrix T0-6 out of tool centre point (tcp) pose
	// - translate tcp in the direction of tcp's orientation by - wristlength -> wrist centre point (wcp)
	// - compute angle0 by arctan of wcp's projection to the floor
	// results in two possible solutions, called forward solution and backward solution

	// start with T0-6
	rational sinx = sin(tcp.orientation[0]);
	rational cosx = cos(tcp.orientation[0]);
	rational siny = sin(tcp.orientation[1]);
	rational cosy = cos(tcp.orientation[1]);
	rational sinz = sin(tcp.orientation[2]);
	rational cosz = cos(tcp.orientation[2]);

	// Set transformation matrix T06
	// left upper 3x3 part is rotation matrix out of three euler angles in zy'x'' model
	// (http://www-home.htwg-konstanz.de/~bittel/ain_robo/Vorlesung/02_PositionUndOrientierung.pdf)
	// (actually only columns 3 and 4 are required, but compute everything for debugging)
	HomMatrix T06 = HomMatrix(4,4,
		{ 		cosz*siny,	cosz*siny*sinx-sinz*cosx,	cosz*siny*cosx+sinz*sinx,	tcp.position[0],
				sinz*cosy,	sinz*siny*sinx+cosz*cosx,	sinz*siny*cosx-cosz*sinx,	tcp.position[1],
				-siny,		cosy*sinx,					cosy*cosx,					tcp.position[2],
				0,			0,							0,							1 });

	HomVector wcp_from_tcp_perspective = { 0,0,-WristLength,1 };
	HomVector wcp = T06 * wcp_from_tcp_perspective; // matrix multiplication
	// this was really inefficient, due to all those 0 in wcp_from_tcp_perspective, thew folliwing is much better
	// but less understandable
	/* HomVector wcp =
	 	 	 {			tcp.position[0] + wcp_from_tcp_perspective[3]*-siny,
			 	 	 	tcp.position[1] + wcp_from_tcp_perspective[3]*-sinx*cosy,
						tcp.position[2] + wcp_from_tcp_perspective[3]*cosx*cosy,
						1
					};
	*/

	// compute base angle by wrist position
	// we have two possible solutions, looking forward and looking backward
	// depending on the sign of the tcp x-coordinate, we assign the two solutions to
	// the forward-angle and the backward-angle
	rational angle0_solution1 = atan2(tcp.position[1], tcp.position[0]);
	rational angle0_solution2 = atan2(-tcp.position[1], -tcp.position[0]);
	rational angle0_forward = 0;
	rational angle0_backward = 0;

	if (tcp.position[0] >= 0) {
		angle0_forward =  angle0_solution1;
		angle0_backward = angle0_solution2;
	} else {
		angle0_forward =  angle0_solution2;
		angle0_backward = angle0_solution1;
	}
	LOG(DEBUG) << "angle0_forward=" << angle0_forward << " angle0_backward=" << angle0_backward;

	// 2. Compute angle1 and angle2
	// use triangle of joint1(A=base + baseheight),joint2(C),and joint3(WCP)
	// - triangle sides are a=forearmlength, b=upperarmlength, c =distance(wcp, base)
	// - compute angles with cosinus sentence

	/*
	LOG(DEBUG) << "computeReverseKinematics (" << setprecision(1)
			<< pAngle[0] << ", " << pAngle[1] << ", " << pAngle[2] << ", "
			<< pAngle[3] << ", " << pAngle[4] << ", " << pAngle[5] << ")";
			*/
}
