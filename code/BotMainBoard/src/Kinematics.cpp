/*
 * Kinematics.cpp
 *
 *  Created on: 27.06.2016
 *      Author: JochenAlt
 */

#include "Kinematics.h"
#include "Util.h"

#define X 0
#define Y 1
#define Z 2


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
	DHParams[5] = DenavitHardenbergParams(0, 				0, 				HandLength);
}


// use DenavitHardenberg parameter and compute the Dh-Transformation matrix with a given joint angle (theta)
void Kinematics::computeDHMatrix(const DenavitHardenbergParams& DHparams, rational pTheta, HomMatrix& dh) {

	rational ct = cos(pTheta);
	rational st = sin(pTheta);

	rational a = DHparams.getA();
	rational d = DHparams.getD();
	rational alpha = DHparams.getAlpha();

	rational sa = DHparams.sinalpha();
	rational ca = DHparams.cosalpha();

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
void Kinematics::computeInverseKinematicsCandidates(const Pose& tcp, std::vector<IKSolutionType> &solutions) {
	LOG(DEBUG) << "computeReverseKinematics(" << endl << setprecision(4)
			<< "{p=(" << tcp.position[0] << "," << tcp.position[1] << "," << tcp.position[2] << ");("
			<< tcp.orientation[0] << "," << tcp.orientation[1] << "," << tcp.orientation[2] << ")})";


	// 1. Step compute angle0 (base)
	// - compute Transformationsmatrix T0-6 out of tool centre point (tcp) pose
	// - translate tcp in the direction of tcp's orientation by - wristlength -> wrist centre point (wcp)
	// - compute angle0 by arctan of wcp's projection to the floor
	// results in two possible solutions, called forward solution and backward solution

	// start with T0-6
	rational sinx = sin(tcp.orientation[X]);
	rational cosx = cos(tcp.orientation[X]);
	rational siny = sin(tcp.orientation[Y]);
	rational cosy = cos(tcp.orientation[Y]);
	rational sinz = sin(tcp.orientation[Z]);
	rational cosz = cos(tcp.orientation[Z]);

	// Set transformation matrix T06
	// left upper 3x3 part is rotation matrix out of three euler angles in zy'x'' model
	// (http://www-home.htwg-konstanz.de/~bittel/ain_robo/Vorlesung/02_PositionUndOrientierung.pdf)
	// (actually only columns 3 and 4 are required, but compute everything for debugging)
	HomMatrix T06 = HomMatrix(4,4,
		{ 	cosz*siny,	cosz*siny*sinx-sinz*cosx,	cosz*siny*cosx+sinz*sinx,	tcp.position[0],
			sinz*cosy,	sinz*siny*sinx+cosz*cosx,	sinz*siny*cosx-cosz*sinx,	tcp.position[1],
			-siny,		cosy*sinx,					cosy*cosx,					tcp.position[2],
			0,			0,							0,							1 });

	HomVector wcp_from_tcp_perspective = { 0,0,-HandLength,1 };
	HomVector wcp = T06 * wcp_from_tcp_perspective;

	// compute base angle by wrist position
	// we have two possible solutions, looking forward and looking backward
	// depending on the sign of the tcp x-coordinate, we assign the two solutions to
	// the forward-angle and the backward-angle
	rational angle0_solution1 = atan2(tcp.position[Y], tcp.position[X]);
	rational angle0_solution2 = atan2(-tcp.position[Y], -tcp.position[X]);
	rational angle0_forward = 0;
	rational angle0_backward = 0;
	bool tcpXPositive = tcp.position[X] >= 0;

	if (tcpXPositive) {
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

	// distance of joint 1 and wcp
	rational z_distance_joint1_wcp = wcp[Z]-HipHeight;
	// distance of base and wcp when looked at from top
	rational distance_base_wcp_from_top = hypothenuseLength(wcp[X],wcp[Y]);
	// side c of the triangle
	rational c = hypothenuseLength(z_distance_joint1_wcp,distance_base_wcp_from_top);
	rational b = UpperArmLength;
	rational a = ForearmLength;
	bool error;
	rational alpha = triangleAlpha(a,b,c, error);
	if (error)
		LOG(ERROR) << "triangle alpha computation invalid";
	rational gamma = triangleGamma(a,b,c, error);
	if (error)
		LOG(ERROR) << "triangle alpha computation invalid";

	// flip flags states whether triangle is in non-flipping or flipping position
	rational flipFlag_forward = tcpXPositive?1.0:-1.0;
	rational flipFlag_backward = tcpXPositive?-1.0:1.0;

	rational delta_forward = atan2(flipFlag_forward*distance_base_wcp_from_top, wcp[Z]);
	rational delta_backward = atan2(flipFlag_backward*distance_base_wcp_from_top, wcp[Z]);

	rational angle1_forward_sol1 = HALF_PI - ( delta_forward + alpha);
	rational angle1_forward_sol2 = HALF_PI - ( delta_forward - alpha);
	rational angle1_backward_sol1 = HALF_PI - ( delta_backward + alpha);
	rational angle1_backward_sol2 = HALF_PI - ( delta_backward - alpha);

	rational angle2_sol2 = gamma - (PI*3.0/2.0);
	rational angle2_sol1 = HALF_PI - gamma;

	LOG(DEBUG) << "triangle (a,b,c)=(" << setprecision(1) << a << "," << b << "," << c << ")"
			<< "angle1_fwd_1= " << angle1_forward_sol1
			<< "angle1_fwd_2= " << angle1_forward_sol2
			<< "angle1_bck_1= " << angle1_backward_sol1
			<< "angle1_bck_2= " << angle1_backward_sol2
			<< "angle2_1= " << angle2_sol1
			<< "angle2_2= " << angle2_sol2;

	// 3. compute angle3, angle4, angle5
	// - compute rotation matrix R0-3
	// - compute Inverse(R0-3), ( which equals the transposed matrix)
	// - take R0-6 aus of T0-6(which we already have)
	// - derive R3-6 by inverse(R0-3)*R0-6
	// - compute angle3,4,5 by solving R3-6

	IKSolutionType up, down;
	computeIKUpperAngles(IKSolutionType::PoseDirectionType::FRONT, IKSolutionType::PoseFlipType::NO_FLIP,
			angle0_forward, angle1_forward_sol1, angle2_sol1, T06,	solutions[0], solutions[1]);

	computeIKUpperAngles(IKSolutionType::PoseDirectionType::FRONT, IKSolutionType::PoseFlipType::FLIP,
			angle0_forward, angle1_forward_sol2, angle2_sol2, T06, solutions[2], solutions[3]);

	computeIKUpperAngles(IKSolutionType::PoseDirectionType::BACK, IKSolutionType::PoseFlipType::NO_FLIP,
			angle0_backward, angle1_backward_sol1, angle2_sol1, T06,	solutions[4], solutions[5]);

	computeIKUpperAngles(IKSolutionType::PoseDirectionType::BACK, IKSolutionType::PoseFlipType::FLIP,
			angle0_backward, angle1_backward_sol2, angle2_sol2, T06, solutions[6], solutions[7]);
	/*
	LOG(DEBUG) << "computeReverseKinematics (" << setprecision(1)
			<< pAngle[0] << ", " << pAngle[1] << ", " << pAngle[2] << ", "
			<< pAngle[3] << ", " << pAngle[4] << ", " << pAngle[5] << ")";
	*/
}

void Kinematics::computeIKUpperAngles(
		IKSolutionType::PoseDirectionType poseDirection, IKSolutionType::PoseFlipType poseFlip,
		rational angle0, rational angle1, rational angle2, const HomMatrix &T06,
		IKSolutionType &sol_up, IKSolutionType &sol_down) {
	sol_up.poseFlip = poseFlip;
	sol_up.poseDirection = poseDirection;
	sol_up.angles[0] = angle0;
	sol_up.angles[1] = angle1;
	sol_up.angles[2] = angle2;

	sol_down.poseFlip = poseFlip;
	sol_down.poseDirection = poseDirection;
	sol_down.angles[0] = angle0;
	sol_down.angles[1] = angle1;
	sol_down.angles[2] = angle2;

	// 3. compute angle3, angle4, angle5
	// - compute rotation matrix R0-3
	// - compute Inverse(R0-3), ( which equals the transposed matrix)
	// - take R0-6 aus of T0-6(which we already have)
	// - derive R3-6 by inverse(R0-3)*R0-6
	// - compute angle3,4,5 by solving R3-6
	HomMatrix T01, T12, T23;
	computeDHMatrix(DHParams[0], angle0, T01);
	computeDHMatrix(DHParams[1], angle1, T12);
	computeDHMatrix(DHParams[2], angle2, T23);

	Matrix R01 = T01[mslice(0,0,3,3)];
	Matrix R12 = T12[mslice(0,0,3,3)];
	Matrix R23 = T23[mslice(0,0,3,3)];
	Matrix R03 = R01*R12*R23;

	// compute inverse by transposing manually
	Matrix R03_inv(R03);
	mswap(R03_inv[0][1], R03_inv[1][0]);
	mswap(R03_inv[0][2], R03_inv[2][0]);
	mswap(R03_inv[1][2], R03_inv[2][1]);

	Matrix R06 = T06[mslice(0,0,3,3)];
	Matrix R36 = R03_inv*R06;

	sol_up.angles[4]   = acos(R36[2][2]);
	sol_down.angles[4] = -sol_up.angles[4];

	rational sin_angle4_1 = sin(sol_up.angles[4]);
	rational sin_angle4_2 = -sin_angle4_1;

	if (almostEqual(sin_angle4_1,0)) {
		sol_up.angles[5]   = atan2(R36[2][1], R36[2][0]);
		sol_down.angles[5] = sol_up.angles[5];
	}
	else {
		sol_up.angles[5]   = atan2( R36[2][1]/sin_angle4_1, R36[2][0]/sin_angle4_1);
		sol_down.angles[5] = atan2( R36[2][1]/sin_angle4_2, R36[2][0]/sin_angle4_2);
	}

	if (almostEqual(sin_angle4_1,0)) {
		sol_up.angles[3]   = atan2(R36[1][3], R36[0][2]);
		sol_down.angles[3] = sol_up.angles[3];
	}
	else {
		sol_up.angles[3]   = atan2(R36[1][3]/sin_angle4_1, R36[0][2]/sin_angle4_1);
		sol_down.angles[3] = atan2(R36[1][3]/sin_angle4_2, R36[0][2]/sin_angle4_2);
	}
}


bool Kinematics::isIKValid(const Pose& pose, const IKSolutionType& sol) {
	Pose computedPose;
	computeForwardKinematics(sol.angles,computedPose);
	return (almostEqual(computedPose.position[X], pose.position[X]) &&
			almostEqual(computedPose.position[Y], pose.position[Y]) &&
			almostEqual(computedPose.position[Z], pose.position[Z]));
}

bool Kinematics::isIKInBoundaries(const std::vector<ActuatorStateType> &boundaries, const IKSolutionType &sol) {
	bool ok = false;
	for (unsigned i = 0;i<sol.angles.size();i++) {
		if ((sol.angles[i] < boundaries[i].minAngle) || (sol.angles[i] > boundaries[0].maxAngle))
			ok = false;
	}
	return ok;
}

bool Kinematics::chooseIKSolution(const std::vector<ActuatorStateType>& current, const Pose& pose, std::vector<IKSolutionType> &solutions,
								  int &choosenSolution) {
	rational bestDistance = 0;
	for (unsigned i = 0;i<solutions.size();i++ ) {
		const IKSolutionType& sol = solutions[i];
		// check only valid solutions
		if (isIKValid(pose,sol)) {
			// check if in valid boundaries
			if (isIKInBoundaries(current, sol)) {
				// check if solution is close the current situation
				rational distance = 0;
				for (unsigned j = 0;j< Actuators;j++) {
					distance +=
							sqr(sol.angles[j] - current[j].currentAngle);
				}
				if ((distance < bestDistance) || (choosenSolution == -1)) {
					choosenSolution = i;
					bestDistance = distance;
				}
			}
			else {
				LOG(DEBUG) << "solution out of bounds dir=" << sol.poseDirection << " flip=" << sol.poseFlip << " turn=" << sol.poseTurn << ", omitted.";
			}
		} else {
			LOG(DEBUG) << "solution invalid dir=" << sol.poseDirection << " flip=" << sol.poseFlip << " turn=" << sol.poseTurn << ", omitted.";
		}
	}

	if ((choosenSolution >= 0) && (bestDistance > 1.0)) {
		LOG(ERROR) << "best solution is idx=" << setprecision(2) << choosenSolution << " dist=" << bestDistance;
		return false;
	}
	return true;
}

bool Kinematics::computeInverseKinematics(const std::vector<ActuatorStateType>& current, const Pose& pose, IKSolutionType &solution) {
	std::vector<IKSolutionType> solutions;
	computeInverseKinematicsCandidates(pose, solutions);
	int selectedIdx = -1;
	bool ok = chooseIKSolution(current, pose, solutions, selectedIdx);
	if (ok)
		solution = solutions[selectedIdx];
	return ok;
}
